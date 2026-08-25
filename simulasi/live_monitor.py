"""
SABITA - Live Monitor (telemetry wireless dari ESP32 lewat WebSocket)
========================================================================
Menyambung ke WebSocket robot (port 81, sama seperti dashboard HTML bawaan
firmware) dan menampilkan dashboard live di laptop: peta graf + node
saat ini, panel navigasi, kotak sensor S1-S6, DAN strip chart sensor
seperti logic-analyzer untuk mendiagnosis pergerakan yang aneh/muter-muter.

Semua komputasi render ada di laptop -- ESP32 hanya broadcast JSON yang
memang sudah dikirim ke dashboard bawaan (ws.broadcastTXT), jadi tidak
menambah beban ke ESP32 sama sekali.

Koneksi:
    1. Sambungkan laptop ke WiFi AP robot: SSID "SABITA_ROBOT", pass "12345678"
    2. Jalankan: python live_monitor.py
       (default host 192.168.4.1:81, sesuai default WiFi.softAP() di firmware)
    3. Untuk host/IP lain: python live_monitor.py --host 192.168.4.1 --port 81

Setiap pesan yang diterima otomatis dicatat ke simulasi/logs/*.jsonl supaya
bisa direview lagi setelah kejadian (mis. episode robot muter-muter tadi),
tanpa perlu nonton live. Baca lognya dengan:
    import pandas as pd
    df = pd.read_json("logs/sabita_log_....jsonl", lines=True)

Keterbatasan: firmware tidak mengirim posisi kontinu (tidak ada odometri),
jadi peta hanya menandai robot di node terakhir yang di-scan QR + arah
tujuan berikutnya (garis putus-putus). Yang akurat real-time adalah panel
sensor S1-S6 dan strip chart di bawah -- itu yang paling berguna untuk
debug line-follower.
"""

import argparse
import json
import math
import os
import queue
import threading
import time
from collections import deque
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation

import websocket  # pip install websocket-client

from sabita_topology import NODES, EDGE_DIST, NAMA_KARYA, POS

STATE_COLOR = {
    "IDLE": "#94a3b8", "ARRIVED": "#f59e0b", "MOVING": "#22c55e",
    "FINISHED": "#a855f7", "MANUAL": "#3b82f6",
}
SENSOR_KEYS = ['s1', 's2', 's3', 's4', 's6']  # urutan fisik kiri -> kanan
HISTORY_WINDOW_S = 20.0  # rentang strip chart (detik)
OSC_WINDOW_S = 5.0       # jendela deteksi "pembalikan cepat"

# ============================================================
#  WEBSOCKET CLIENT (thread terpisah, non-blocking terhadap GUI)
# ============================================================
class RobotLink:
    def __init__(self, host, port, log_path):
        self.url = f"ws://{host}:{port}/"
        self.msg_queue = queue.Queue()
        self.connected = False
        self._stop = False
        self._t0 = time.time()
        self.log_file = open(log_path, "a", encoding="utf-8")
        self.thread = threading.Thread(target=self._run_forever, daemon=True)

    def start(self):
        print(f"[link] menyambung ke {self.url} ...")
        self.thread.start()

    def stop(self):
        self._stop = True
        try:
            self.ws.close()
        except Exception:
            pass
        self.log_file.close()

    def _on_open(self, ws):
        self.connected = True
        print("[link] TERHUBUNG ke robot.")

    def _on_close(self, ws, code, msg):
        self.connected = False
        print("[link] Terputus dari robot, coba lagi dalam 2 detik...")

    def _on_error(self, ws, err):
        print(f"[link] error: {err}")

    def _on_message(self, ws, raw):
        t = time.time() - self._t0
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return
        data["_t"] = t
        data["_recv_iso"] = datetime.now().isoformat(timespec="milliseconds")
        self.log_file.write(json.dumps(data, ensure_ascii=False) + "\n")
        self.log_file.flush()
        self.msg_queue.put(data)

    def _run_forever(self):
        while not self._stop:
            self.ws = websocket.WebSocketApp(
                self.url,
                on_open=self._on_open,
                on_close=self._on_close,
                on_error=self._on_error,
                on_message=self._on_message,
            )
            self.ws.run_forever(ping_interval=10, ping_timeout=3)
            if self._stop:
                break
            time.sleep(2)


# ============================================================
#  STATE TERKINI + HISTORY UNTUK STRIP CHART
# ============================================================
class RobotState:
    def __init__(self):
        self.state = "IDLE"
        self.prev, self.curr, self.nxt, self.art = "-", "-", "-", "-"
        self.step, self.total = 0, 6
        self.route_str, self.route_len = "Rute belum dihitung", None
        self.sensors = {k: 0 for k in SENSOR_KEYS}
        self.raw = {'r1': 0, 'r2': 0, 'r3': 0, 'r4': 0, 'r6': 0}
        self.arah = "-"
        self.dfp_status, self.dfp_name = "unknown", ""
        self.manual = False
        self.last_qr = "-"
        self.history = deque()  # (t, {s1..s6}, arah)
        self.arah_events = deque()  # (t, arah) hanya saat arah berubah, untuk deteksi osilasi
        self.connected = False

    def apply(self, msg):
        typ = msg.get("type")
        t = msg.get("_t", 0.0)
        if typ == "state":
            self.state = msg.get("state", self.state)
        elif typ == "nav":
            self.prev = msg.get("prev", self.prev)
            self.curr = msg.get("curr", self.curr)
            self.nxt = msg.get("next", self.nxt)
            self.art = msg.get("art", self.art)
            self.step = int(msg.get("step", self.step))
            self.total = int(msg.get("total", self.total))
        elif typ == "route":
            self.route_str = msg.get("route", self.route_str)
            self.route_len = float(msg.get("length", 0))
        elif typ == "dfp":
            self.dfp_status = msg.get("status", self.dfp_status)
            self.dfp_name = msg.get("name", "")
        elif typ == "mode":
            self.manual = str(msg.get("manual")) == "1"
        elif typ == "qr":
            self.last_qr = msg.get("data", self.last_qr)
        elif typ == "sensor":
            for k in SENSOR_KEYS:
                self.sensors[k] = int(msg.get(k, 0))
            for k in ('r1', 'r2', 'r3', 'r4', 'r6'):
                if k in msg:
                    self.raw[k] = msg[k]
            self.arah = msg.get("arah", self.arah)

            self.history.append((t, dict(self.sensors), self.arah))
            while self.history and t - self.history[0][0] > HISTORY_WINDOW_S:
                self.history.popleft()

            if not self.arah_events or self.arah_events[-1][1] != self.arah:
                self.arah_events.append((t, self.arah))
            while self.arah_events and t - self.arah_events[0][0] > OSC_WINDOW_S:
                self.arah_events.popleft()

    def oscillation_count(self):
        """Hitung berapa kali arah loncat antara KIRI TAJAM <-> KANAN TAJAM
        dalam jendela OSC_WINDOW_S -- indikator kuat robot 'muter-muter'."""
        sharp = [a for _, a in self.arah_events if a in ("KIRI TAJAM", "KANAN TAJAM")]
        flips = 0
        for i in range(1, len(sharp)):
            if sharp[i] != sharp[i - 1]:
                flips += 1
        return flips


# ============================================================
#  RENDER
# ============================================================
def draw(fig, ax_map, ax_panel, ax_strip, rs: RobotState):
    ax_map.clear()
    ax_panel.clear()
    ax_strip.clear()
    ax_panel.axis("off")

    for (a, b), w in EDGE_DIST.items():
        xa, ya = POS[a]
        xb, yb = POS[b]
        ax_map.plot([xa, xb], [ya, yb], color="#3a3d4e", lw=1.5, zorder=1)

    if rs.curr in POS and rs.nxt in POS and rs.state == "MOVING":
        xa, ya = POS[rs.curr]
        xb, yb = POS[rs.nxt]
        ax_map.plot([xa, xb], [ya, yb], color="#f59e0b", lw=2.5, ls="--", zorder=2)

    for node in NODES:
        x, y = POS[node]
        is_curr = (node == rs.curr)
        color = "#22c55e" if is_curr else "#252940"
        ax_map.add_patch(mpatches.Circle((x, y), 0.24, facecolor=color,
                                          edgecolor="#3a3d4e", lw=2, zorder=3))
        ax_map.text(x, y, node, fontsize=15, fontweight="bold", color="white",
                     ha="center", va="center", zorder=4)
        ax_map.text(x, y - 0.42, NAMA_KARYA[node], fontsize=7, color="#94a3b8",
                     ha="center", va="center", zorder=6,
                     bbox=dict(boxstyle="round,pad=0.15", fc="#0f1117", ec="none", alpha=0.85))

    if rs.curr in POS:
        rx, ry = POS[rs.curr]
        pulse = 0.28 if rs.state == "MOVING" else 0.0
        wobble = abs(math.sin(time.time() * 4.0))
        ax_map.add_patch(mpatches.Circle((rx, ry), 0.30 + pulse * wobble,
                                          fill=False, edgecolor="#ef4444", lw=2, zorder=5))

    ax_map.set_xlim(-2.8, 2.8)
    ax_map.set_ylim(-2.4, 2.7)
    ax_map.set_aspect("equal")
    ax_map.axis("off")
    conn_txt = "TERHUBUNG" if rs.connected else "TERPUTUS - mencoba lagi..."
    conn_col = "#22c55e" if rs.connected else "#ef4444"
    ax_map.set_title(f"SABITA Live Monitor  [{conn_txt}]", color=conn_col,
                      fontsize=12, fontweight="bold")

    # ---- panel info ----
    y0 = 0.97
    state_col = STATE_COLOR.get(rs.state, "#94a3b8")
    ax_panel.add_patch(mpatches.FancyBboxPatch((0.02, y0 - 0.09), 0.96, 0.07,
                        boxstyle="round,pad=0.01", facecolor="#252940",
                        edgecolor=state_col, lw=1.5, transform=ax_panel.transAxes))
    ax_panel.text(0.5, y0 - 0.055, rs.state + (" (MANUAL)" if rs.manual else ""),
                  fontsize=10, fontweight="bold", color=state_col,
                  ha="center", va="center", transform=ax_panel.transAxes)

    yy = y0 - 0.17
    ax_panel.text(0.02, yy, "NAVIGASI", fontsize=9, color="#94a3b8", fontweight="bold",
                  transform=ax_panel.transAxes)
    for label, val, col in [("Previous", rs.prev, "#e0e0e0"),
                             ("Current ", rs.curr, "#22c55e"),
                             ("Next    ", rs.nxt, "#e0e0e0")]:
        yy -= 0.04
        ax_panel.text(0.04, yy, f"{label} : {val}", fontsize=9, color=col,
                      family="monospace", transform=ax_panel.transAxes)
    yy -= 0.05
    ax_panel.text(0.04, yy, f"Karya: {rs.art}   ({rs.step}/{rs.total})", fontsize=8.5,
                  color="#f59e0b", transform=ax_panel.transAxes)

    yy -= 0.07
    length_disp = f" ({rs.route_len:.2f} m)" if rs.route_len else ""
    ax_panel.text(0.02, yy, f"Rute: {rs.route_str}{length_disp}", fontsize=8.5,
                  color="#94a3b8", transform=ax_panel.transAxes)

    yy -= 0.06
    ax_panel.text(0.02, yy, f"DFPlayer: {rs.dfp_status}  {rs.dfp_name}", fontsize=8.5,
                  color="#3b82f6", transform=ax_panel.transAxes)
    yy -= 0.045
    ax_panel.text(0.02, yy, f"QR terakhir: {rs.last_qr}", fontsize=8, color="#64748b",
                  transform=ax_panel.transAxes)

    yy -= 0.08
    ax_panel.text(0.02, yy, "LINE FOLLOWER (S1-S6)", fontsize=9, color="#94a3b8",
                  fontweight="bold", transform=ax_panel.transAxes)
    yy -= 0.06
    box_w = 0.16
    for i, key in enumerate(SENSOR_KEYS):
        val = rs.sensors[key]
        x0 = 0.04 + i * (box_w + 0.02)
        fc, tc = ("#22c55e", "#0f1117") if val == 1 else ("#2a2d3e", "#94a3b8")
        num = key[1] if key != 's6' else '6'
        ax_panel.add_patch(mpatches.FancyBboxPatch((x0, yy - 0.045), box_w, 0.06,
                            boxstyle="round,pad=0.004", facecolor=fc,
                            edgecolor="#3a3d4e", lw=1, transform=ax_panel.transAxes))
        ax_panel.text(x0 + box_w / 2, yy - 0.015, f"S{num}", fontsize=7, color=tc,
                      ha="center", va="center", transform=ax_panel.transAxes)
        ax_panel.text(x0 + box_w / 2, yy - 0.033, str(val), fontsize=8, color=tc,
                      fontweight="bold", ha="center", va="center", transform=ax_panel.transAxes)

    yy -= 0.08
    ax_panel.text(0.04, yy, f"Arah: {rs.arah}", fontsize=9.5, color="#3b82f6",
                  fontweight="bold", transform=ax_panel.transAxes)

    osc = rs.oscillation_count()
    if osc >= 2:
        yy -= 0.06
        ax_panel.add_patch(mpatches.FancyBboxPatch((0.02, yy - 0.035), 0.96, 0.05,
                            boxstyle="round,pad=0.01", facecolor="#451a1a",
                            edgecolor="#ef4444", lw=1.2, transform=ax_panel.transAxes))
        ax_panel.text(0.5, yy - 0.01,
                      f"⚠ {osc}x pembalikan KIRI/KANAN TAJAM dlm {OSC_WINDOW_S:.0f}dtk terakhir",
                      fontsize=8, color="#ef4444", fontweight="bold",
                      ha="center", va="center", transform=ax_panel.transAxes)

    ax_panel.set_xlim(0, 1)
    ax_panel.set_ylim(0, 1)

    # ---- strip chart sensor (logic-analyzer style) ----
    ax_strip.set_facecolor("#0f1117")
    now_t = rs.history[-1][0] if rs.history else 0.0
    t_min = now_t - HISTORY_WINDOW_S
    for row_i, key in enumerate(SENSOR_KEYS):
        prev_t, prev_v = t_min, 0
        for t, sens, _ in rs.history:
            v = sens[key]
            if v != prev_v:
                if prev_v == 1:
                    ax_strip.add_patch(plt.Rectangle((prev_t, row_i), t - prev_t, 0.9,
                                                       facecolor="#22c55e", edgecolor="none"))
                prev_t, prev_v = t, v
        if prev_v == 1:
            ax_strip.add_patch(plt.Rectangle((prev_t, row_i), now_t - prev_t, 0.9,
                                               facecolor="#22c55e", edgecolor="none"))
    ax_strip.set_xlim(t_min, max(now_t, t_min + 1))
    ax_strip.set_ylim(0, len(SENSOR_KEYS))
    ax_strip.set_yticks([i + 0.45 for i in range(len(SENSOR_KEYS))])
    ax_strip.set_yticklabels([f"S{k[1] if k != 's6' else '6'}" for k in SENSOR_KEYS],
                              color="#94a3b8", fontsize=8)
    ax_strip.set_xlabel("waktu (detik, bergulir)", color="#64748b", fontsize=8)
    ax_strip.tick_params(colors="#64748b", labelsize=7)
    for spine in ax_strip.spines.values():
        spine.set_color("#2a2d3e")
    ax_strip.set_title("Riwayat sensor S1-S6 (hijau = kena garis) -- lihat pola loncatan di sini",
                        color="#94a3b8", fontsize=8.5, loc="left")


def main():
    ap = argparse.ArgumentParser(description="SABITA live telemetry monitor")
    ap.add_argument("--host", default="192.168.4.1", help="IP ESP32 (default AP: 192.168.4.1)")
    ap.add_argument("--port", type=int, default=81, help="Port WebSocket (default 81)")
    args = ap.parse_args()

    os.makedirs("logs", exist_ok=True)
    log_path = os.path.join("logs", f"sabita_log_{datetime.now():%Y%m%d_%H%M%S}.jsonl")
    print(f"[log] mencatat semua data ke {log_path}")

    link = RobotLink(args.host, args.port, log_path)
    rs = RobotState()
    link.start()

    fig = plt.figure(figsize=(12.5, 7), facecolor="#0f1117")
    gs = fig.add_gridspec(2, 2, width_ratios=[1.1, 1], height_ratios=[2.4, 1], wspace=0.05, hspace=0.35)
    ax_map = fig.add_subplot(gs[0, 0])
    ax_panel = fig.add_subplot(gs[0, 1])
    ax_strip = fig.add_subplot(gs[1, :])
    for ax in (ax_map, ax_panel, ax_strip):
        ax.set_facecolor("#0f1117")

    def update(_frame):
        rs.connected = link.connected
        drained = 0
        while drained < 200:
            try:
                msg = link.msg_queue.get_nowait()
            except queue.Empty:
                break
            rs.apply(msg)
            drained += 1
        draw(fig, ax_map, ax_panel, ax_strip, rs)
        return []

    def on_close(_evt):
        link.stop()

    fig.canvas.mpl_connect("close_event", on_close)

    anim = FuncAnimation(fig, update, interval=150, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
        print("[log] ditutup. Untuk review: pd.read_json(r'%s', lines=True)" % log_path)


if __name__ == "__main__":
    main()
