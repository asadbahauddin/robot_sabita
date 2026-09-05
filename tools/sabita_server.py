"""
SABITA - Server relay (laptop)
================================
Menjembatani ESP32 (headless, WebSocket data-only di port 81) dengan
dashboard HTML yang jalan di browser laptop. ESP32 tidak lagi menyajikan
HTML sendiri -- semua UI ada di sini supaya beban ESP32 minimal.

Alur data:
    ESP32 --(WebSocket client, 'websockets')--> server ini
    server ini --(HTTP statis + WebSocket server, 'aiohttp')--> browser

Fungsi:
    a) Serve dashboard.html di http://localhost:8080
    b) Sambung ke ESP32 ws://<esp32-host>:81/ (auto-reconnect)
    c) Relay semua pesan ESP32 -> semua browser yang connect ke ws://localhost:8080/ws
    d) Relay semua perintah browser -> ESP32
    e) Log data sensor ke logs/robot_log_<waktu>.csv
    f) Endpoint /download-log untuk unduh CSV yang sedang aktif

Jalankan:
    python sabita_server.py [--esp-host 192.168.4.1] [--esp-port 81] [--port 8080]
"""

import argparse
import asyncio
import csv
import json
import os
import sys
from datetime import datetime

import websockets
from aiohttp import web, WSMsgType, ClientConnectorError

SENSOR_CSV_FIELDS = [
    "recv_iso", "t_rel_s", "state", "nav_prev", "nav_curr", "nav_next", "nav_step",
    "s1", "s2", "s3", "s4", "s6",
    "pos", "err", "corr", "kp", "ki", "kd", "arah", "last_qr",
]


class Hub:
    """State bersama antara task ESP32-link dan handler HTTP/WS aiohttp."""

    def __init__(self, esp_host, esp_port, log_path):
        self.esp_host = esp_host
        self.esp_port = esp_port
        self.esp_url = f"ws://{esp_host}:{esp_port}/"
        self.esp_ws = None            # koneksi aktif ke ESP32 (websockets client)
        self.esp_connected = False
        self.browsers = set()         # aiohttp WebSocketResponse yang aktif
        self.last_by_type = {}        # cache pesan terakhir per 'type', utk klien baru
        self.t0 = asyncio.get_event_loop().time()

        self.log_path = log_path
        self._csv_file = open(log_path, "a", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=SENSOR_CSV_FIELDS)
        if self._csv_file.tell() == 0:
            self._csv_writer.writeheader()
            self._csv_file.flush()

        # Log latar belakang (di atas) selalu jalan otomatis sejak server start --
        # itu jaring pengaman. rec_* di bawah ini terpisah: file baru yang cuma
        # aktif kalau user pencet REC:START di dashboard, buat menandai sesi tes
        # tertentu supaya gampang dicari lagi tanpa perlu filter t_rel_s manual.
        self.rec_file = None
        self.rec_writer = None
        self.rec_path = None

    def start_recording(self):
        if self.rec_file:
            self.stop_recording()
        log_dir = os.path.dirname(self.log_path)
        path = os.path.join(log_dir, f"recording_{datetime.now():%Y%m%d_%H%M%S}.csv")
        self.rec_file = open(path, "a", newline="", encoding="utf-8")
        self.rec_writer = csv.DictWriter(self.rec_file, fieldnames=SENSOR_CSV_FIELDS)
        self.rec_writer.writeheader()
        self.rec_file.flush()
        self.rec_path = path
        print(f"[rec] mulai rekam ke {path}")
        return path

    def stop_recording(self):
        path = self.rec_path
        if self.rec_file:
            self.rec_file.close()
            print(f"[rec] rekaman selesai: {path}")
        self.rec_file = None
        self.rec_writer = None
        self.rec_path = None
        return path

    def log_sensor_row(self, msg):
        row = {k: msg.get(k, "") for k in SENSOR_CSV_FIELDS}
        row["recv_iso"] = datetime.now().isoformat(timespec="milliseconds")
        row["t_rel_s"] = round(asyncio.get_event_loop().time() - self.t0, 3)
        # state/nav diambil dari cache pesan terakhir -- keduanya jarang berubah
        # (cuma saat transisi FSM), jadi nilai ini valid mewakili konteks robot
        # pada saat sampel sensor ini diambil (mis. lagi diam ARRIVED vs MOVING).
        state_msg = self.last_by_type.get("state", {})
        nav_msg = self.last_by_type.get("nav", {})
        row["state"] = state_msg.get("state", "")
        row["nav_prev"] = nav_msg.get("prev", "")
        row["nav_curr"] = nav_msg.get("curr", "")
        row["nav_next"] = nav_msg.get("next", "")
        row["nav_step"] = nav_msg.get("step", "")
        row["last_qr"] = self.last_by_type.get("qr", {}).get("data", "")
        self._csv_writer.writerow(row)
        self._csv_file.flush()
        if self.rec_writer:
            self.rec_writer.writerow(row)
            self.rec_file.flush()

    async def broadcast_to_browsers(self, obj):
        text = json.dumps(obj, ensure_ascii=False)
        dead = []
        for wsres in self.browsers:
            try:
                await wsres.send_str(text)
            except Exception:
                dead.append(wsres)
        for d in dead:
            self.browsers.discard(d)

    async def send_to_esp(self, text):
        if self.esp_ws is not None and self.esp_connected:
            try:
                await self.esp_ws.send(text)
            except Exception as e:
                print(f"[esp] gagal kirim perintah: {e}")

    def close(self):
        self._csv_file.close()


# ============================================================
#  TASK: koneksi ke ESP32 (auto-reconnect)
# ============================================================
async def esp_link_task(hub: Hub):
    while True:
        try:
            print(f"[esp] menyambung ke {hub.esp_url} ...")
            async with websockets.connect(hub.esp_url, ping_interval=10, ping_timeout=5) as wsconn:
                hub.esp_ws = wsconn
                hub.esp_connected = True
                print("[esp] TERHUBUNG.")
                await hub.broadcast_to_browsers({"type": "esplink", "connected": True})

                async for raw in wsconn:
                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    typ = msg.get("type", "?")
                    hub.last_by_type[typ] = msg
                    if typ == "sensor":
                        hub.log_sensor_row(msg)
                    await hub.broadcast_to_browsers(msg)

        except (OSError, ConnectionRefusedError, ClientConnectorError,
                websockets.exceptions.WebSocketException) as e:
            print(f"[esp] error/terputus: {e}")
        except Exception as e:
            print(f"[esp] error tak terduga: {e}")
        finally:
            hub.esp_ws = None
            hub.esp_connected = False
            await hub.broadcast_to_browsers({"type": "esplink", "connected": False})

        await asyncio.sleep(2)


# ============================================================
#  HTTP + WEBSOCKET SERVER (aiohttp) UNTUK BROWSER
# ============================================================
def make_app(hub: Hub, dashboard_path):

    async def handle_index(request):
        return web.FileResponse(dashboard_path)

    async def handle_ws(request):
        wsres = web.WebSocketResponse(heartbeat=15)
        await wsres.prepare(request)
        hub.browsers.add(wsres)
        print(f"[browser] connect (total={len(hub.browsers)})")

        await wsres.send_str(json.dumps({"type": "esplink", "connected": hub.esp_connected}))
        rec_status = {"type": "rec", "active": hub.rec_file is not None,
                      "file": os.path.basename(hub.rec_path) if hub.rec_path else ""}
        await wsres.send_str(json.dumps(rec_status))
        for typ, msg in hub.last_by_type.items():
            try:
                await wsres.send_str(json.dumps(msg, ensure_ascii=False))
            except Exception:
                pass

        try:
            async for wsmsg in wsres:
                if wsmsg.type == WSMsgType.TEXT:
                    if wsmsg.data == "REC:START":
                        path = hub.start_recording()
                        await hub.broadcast_to_browsers({"type": "rec", "active": True, "file": os.path.basename(path)})
                    elif wsmsg.data == "REC:STOP":
                        hub.stop_recording()
                        await hub.broadcast_to_browsers({"type": "rec", "active": False, "file": ""})
                    else:
                        await hub.send_to_esp(wsmsg.data)
                elif wsmsg.type == WSMsgType.ERROR:
                    break
        finally:
            hub.browsers.discard(wsres)
            print(f"[browser] disconnect (total={len(hub.browsers)})")
        return wsres

    async def handle_download_log(request):
        if not os.path.exists(hub.log_path):
            raise web.HTTPNotFound(text="Belum ada log.")
        return web.FileResponse(
            hub.log_path,
            headers={"Content-Disposition": f'attachment; filename="{os.path.basename(hub.log_path)}"'},
        )

    app = web.Application()
    app.router.add_get("/", handle_index)
    app.router.add_get("/ws", handle_ws)
    app.router.add_get("/download-log", handle_download_log)
    return app


async def main_async(args):
    here = os.path.dirname(os.path.abspath(__file__))
    dashboard_path = os.path.join(here, "dashboard.html")
    if not os.path.exists(dashboard_path):
        print(f"[fatal] dashboard.html tidak ditemukan di {dashboard_path}")
        sys.exit(1)

    log_dir = os.path.join(here, "logs")
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, f"robot_log_{datetime.now():%Y%m%d_%H%M%S}.csv")
    print(f"[log] mencatat sensor ke {log_path}")

    hub = Hub(args.esp_host, args.esp_port, log_path)

    app = make_app(hub, dashboard_path)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", args.port)
    await site.start()
    print(f"[http] dashboard di http://localhost:{args.port}")

    esp_task = asyncio.create_task(esp_link_task(hub))

    try:
        await esp_task
    finally:
        hub.close()
        await runner.cleanup()


def main():
    ap = argparse.ArgumentParser(description="SABITA relay server (ESP32 <-> dashboard browser)")
    ap.add_argument("--esp-host", default="192.168.4.1", help="IP ESP32 (default AP: 192.168.4.1)")
    ap.add_argument("--esp-port", type=int, default=81, help="Port WebSocket ESP32 (default 81)")
    ap.add_argument("--port", type=int, default=8080, help="Port HTTP/WS server lokal (default 8080)")
    args = ap.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\n[server] dihentikan.")


if __name__ == "__main__":
    main()
