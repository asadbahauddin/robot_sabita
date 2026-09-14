# sabita_simulator.py
# Simulator ESP32 untuk demo/record dashboard SABITA
# tanpa robot fisik. Kirim data WebSocket ke
# sabita_server.py seolah robot beneran jalan.
#
# Cara pakai:
#   1. python tools/sabita_server.py --esp-host 127.0.0.1 --esp-port 8081
#   2. python tools/sabita_simulator.py
#   3. Buka browser: http://localhost:8080
#   4. Pilih start node di terminal simulator
#
# CATATAN ARSITEKTUR (penting, beda dari asumsi awal "simulator connect ke
# sabita_server.py"): sabita_server.py TIDAK pernah menerima koneksi masuk
# dari ESP32 -- dia yang JADI KLIEN, connect KELUAR ke ws://<esp_host>:
# <esp_port>/ (lihat esp_link_task() di sabita_server.py, sama seperti
# ESP32 asli yang buka WebSocketsServer(81) dan ditungguin). Jadi simulator
# ini justru harus BERPERAN JADI SERVER WS (persis seperti ESP32 asli),
# dan sabita_server.py-lah yang connect KE SINI -- bukan sebaliknya.
# Makanya sabita_server.py perlu dijalankan dengan --esp-host 127.0.0.1
# --esp-port <PORT_SIMULATOR_INI> supaya nyambung ke simulator, bukan ke
# ESP32 fisik. TIDAK ADA perubahan kode di sabita_server.py utk ini --
# cukup argumen CLI yang sudah tersedia.
"""
Berdiri sendiri, tidak butuh robot fisik / WiFi robot. Format JSON yang
dikirim IDENTIK dengan yang dikirim firmware ESP32 asli (lihat
jState/jNav/jRoute/jDfp/jSensor/jQR/jMode/jRouteCompare di robot_sabita.ino)
supaya dashboard.html tidak perlu diubah sama sekali.

Termasuk detail halus yang sering kelewat: pada firmware ASLI saat ini
(mode bang-bang, bukan PID), field pos/err/corr di JSON sensor SELALU 0.00
(gLastPos/gLastErr/gLastCorr adalah sisa era PID yang tidak lagi
diperbarui lineFollow()) -- simulator ini SENGAJA meniru itu apa adanya,
bukan menghitung pos/err/corr yang "kelihatan realistis" tapi menyimpang
dari perilaku firmware asli.
"""

import argparse
import asyncio
import json
import random
import time

import websockets

# ================= KARAKTERISASI MOTOR/RODA (2026-09-14, HASIL TES FISIK) =================
# Tes lab langsung (bukan lagi perkiraan dari spec motor): jalan 1 meter di
# PWM=70, 3x percobaan (16.93s, 18.10s, 19.67s) -> rata-rata 18.23s.
#   v(PWM=70) = 1m / 18.23s = 0.0548 m/s
#   v_max (PWM=255) = v(70) * 255/70 = 0.0548 * 255/70 ~= 0.1998 m/s
# INI KOREKSI BESAR dari perkiraan sebelumnya (v_max diduga 0.73 m/s,
# ternyata angka "0.20 m/s" yang tadinya dikira "v di PWM=70" itu justru
# lebih dekat ke v_max sebenarnya di PWM=255). Robot jauh lebih lambat dari
# dugaan awal -- kemungkinan varian motor 43 RPM (v_max teoritis 0.146 m/s,
# roda 65mm) yang paling dekat, BUKAN 222 RPM seperti dugaan sebelumnya --
# tapi ini tetap indikasi kasar, bukan kepastian, krn lantai tidak rata
# (dicatat user saat tes) ikut memperlambat gerak nyata robot.
MOTOR_V_MAX_MS = 0.1998   # m/s, HASIL UKUR (bukan lagi perkiraan) -- lihat catatan di atas


def speed_for_pwm(pwm):
    """Kecepatan linear robot (m/s) perkiraan pada PWM tertentu (0-255)."""
    return (pwm / 255.0) * MOTOR_V_MAX_MS


# ================= PARAMETER (bisa diubah) =================
MOTOR_SPEED = 70   # identik MOTOR_SPEED default di firmware
ROBOT_SPEED_MS    = speed_for_pwm(MOTOR_SPEED)  # m/s -- ~0.20 m/s pada PWM=70 (lihat karakterisasi di atas)
AUDIO_DURATION_S  = 8.5    # detik per audio
OVERHEAD_S        = 5.0    # detik "berhenti" tambahan sebelum audio mulai (QR+settle)
SENSOR_INTERVAL_S = 0.05   # 50ms (sama dengan SENSOR_INTERVAL_MS di ESP32)
WS_HOST = "0.0.0.0"
WS_PORT = 8081              # simulator BUKA WS SERVER di sini (ganti ESP32 fisik)

# ================= TOPOLOGI (hardcode, identik robot_sabita.ino) =================
NODES = ['A', 'B', 'C', 'D', 'E', 'F']
NAMA_KARYA = {
    'A': 'Mona Lisa', 'B': 'The Scream', 'C': 'The Kiss',
    'D': 'Starry Night', 'E': 'Sunflowers', 'F': 'Guernica',
}
EDGE_DIST = {
    ('A', 'B'): 1.07, ('A', 'E'): 1.51, ('A', 'F'): 1.21,
    ('B', 'C'): 1.80, ('B', 'F'): 1.16,
    ('C', 'D'): 1.68, ('C', 'F'): 1.41,
    ('D', 'E'): 1.49, ('D', 'F'): 1.07,
    ('E', 'F'): 1.54,
}
# Rute ACO terbaik per start node (hardcode, siklus tertutup balik ke start,
# identik dengan hasil ACO offline -- bukan dihitung ulang di sini).
ACO_ROUTES = {
    'A': ['A', 'B', 'F', 'C', 'D', 'E', 'A'],
    'B': ['B', 'F', 'C', 'D', 'E', 'A', 'B'],
    'C': ['C', 'D', 'E', 'A', 'B', 'F', 'C'],
    'D': ['D', 'E', 'A', 'B', 'F', 'C', 'D'],
    'E': ['E', 'A', 'B', 'F', 'C', 'D', 'E'],
    'F': ['F', 'A', 'B', 'C', 'D', 'E', 'F'],
}


def edge_dist(a, b):
    return EDGE_DIST.get((a, b), EDGE_DIST.get((b, a)))


def route_dist(route):
    return sum(edge_dist(route[i], route[i + 1]) for i in range(len(route) - 1))


# ================= POLA SENSOR + LOGIKA MOTOR (meniru lineFollow() & =========
# klasifikasi `arah` di loop() firmware -- prioritas s3 > s2 > s4 > s1 > s6). =

def motor_for(h1, h2, h3, h4, h6):
    if h3: return 70, 70
    if h2: return 30, 70
    if h4: return 70, 30
    if h1: return 20, MOTOR_SPEED
    if h6: return MOTOR_SPEED, 20
    return 40, 40


def arah_for(h1, h2, h3, h4, h6):
    hit_count = sum([h1, h2, h3, h4, h6])
    if hit_count >= 3: return "PERSIMPANGAN"
    if h3 and not h2 and not h4: return "LURUS"
    if h2 and h3: return "BELOK KIRI"
    if h3 and h4: return "BELOK KANAN"
    if h1 or h2: return "KIRI TAJAM"
    if h4 or h6: return "KANAN TAJAM"
    return "TIDAK ADA GARIS"


def random_hit_pattern():
    """70% lurus, 15% belok kiri (s1/s2), 15% belok kanan (s4/s6), 5% persimpangan."""
    bucket = random.choices(['lurus', 'kiri', 'kanan', 'persimpangan'], weights=[70, 15, 15, 5], k=1)[0]
    if bucket == 'lurus':
        return (False, False, True, False, False)
    if bucket == 'kiri':
        return random.choice([(False, True, False, False, False), (True, False, False, False, False)])
    if bucket == 'kanan':
        return random.choice([(False, False, False, True, False), (False, False, False, False, True)])
    return (True, False, True, True, False)  # persimpangan: 3 sensor kena


# ================= JSON BUILDER (field & tipe IDENTIK dgn firmware) =================

def j_state(state):
    return {"type": "state", "state": state}


def j_nav(prev, curr, nxt, art, step, total):
    return {"type": "nav", "prev": prev, "curr": curr, "next": nxt, "art": art, "step": step, "total": total}


def j_route(route_list):
    return {"type": "route", "route": '-'.join(route_list), "length": round(route_dist(route_list), 2)}


def j_routecompare(aco_route_list, aco_length, actual_route_list, actual_length, efficiency):
    return {
        "type": "routecompare",
        "aco_route": '-'.join(aco_route_list), "aco_length": round(aco_length, 2),
        "actual_route": '-'.join(actual_route_list), "actual_length": round(actual_length, 2),
        "efficiency": round(efficiency, 1),
    }


def j_dfp(status, name="", desc="", volume=18):
    d = {"type": "dfp", "status": status}
    if name: d["name"] = name
    if desc: d["desc"] = desc
    d["volume"] = volume
    return d


def j_sensor(h, mode, speed_r, speed_l):
    h1, h2, h3, h4, h6 = h
    return {
        "type": "sensor",
        "s1": 0 if h1 else 1, "s2": 0 if h2 else 1, "s3": 0 if h3 else 1,
        "s4": 0 if h4 else 1, "s6": 0 if h6 else 1,
        # pos/err/corr SELALU 0.00 di firmware asli saat ini (lihat catatan atas file)
        "pos": 0.0, "err": 0.0, "corr": 0.0,
        "kp": 15.0, "ki": 0.01, "kd": 8.0,
        "mode": mode,
        "speedR": speed_r, "speedL": speed_l,
        "lost_ms": 0,
        "arah": arah_for(h1, h2, h3, h4, h6),
    }


def j_qr(data):
    return {"type": "qr", "data": data}


def j_mode(manual, speed):
    return {"type": "mode", "manual": 1 if manual else 0, "speed": speed}


def j_pidmode(mode):
    return {"type": "pidmode", "mode": mode, "t": int(time.time() * 1000)}


# ================= WS SERVER (simulator = "ESP32 palsu") =================
connected = set()


async def broadcast(obj):
    if not connected:
        return
    text = json.dumps(obj, ensure_ascii=False)
    dead = []
    for ws in connected:
        try:
            await ws.send(text)
        except Exception:
            dead.append(ws)
    for d in dead:
        connected.discard(d)


async def ws_handler(websocket):
    connected.add(websocket)
    print(f"[simulator] sabita_server.py TERHUBUNG dari {websocket.remote_address}")
    try:
        # Greeting persis WStype_CONNECTED di firmware asli: kirim snapshot
        # state/dfp/nav/mode awal ke klien yang baru connect.
        await websocket.send(json.dumps(j_state("IDLE")))
        await websocket.send(json.dumps(j_dfp("ready")))
        await websocket.send(json.dumps(j_nav("-", "-", "-", "-", 0, len(NODES))))
        await websocket.send(json.dumps(j_mode(False, MOTOR_SPEED)))
        async for raw in websocket:
            # Simulator ini demo satu-arah (kirim data) -- perintah dari
            # dashboard (RESET, M:MAJU, dst via sabita_server.py) cuma
            # dicatat, tidak benar-benar mempengaruhi jalannya simulasi.
            print(f"[simulator] terima perintah dari dashboard (diabaikan): {raw}")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected.discard(websocket)
        print("[simulator] sabita_server.py terputus")


async def wait_for_connection():
    print(f"[simulator] menunggu sabita_server.py connect ke ws://<host>:{WS_PORT}/ ...")
    while not connected:
        await asyncio.sleep(0.2)
    print("[simulator] terhubung!")


# ================= SIMULASI SENSOR SELAMA DIAM / JALAN =================
async def sensor_tick_stationary(duration):
    elapsed = 0.0
    h = (True, False, True, True, False)  # diam di zona node (mirip persimpangan)
    while elapsed < duration:
        await broadcast(j_sensor(h, "OFF", 0, 0))
        await asyncio.sleep(SENSOR_INTERVAL_S)
        elapsed += SENSOR_INTERVAL_S


async def sensor_tick_moving(duration):
    elapsed = 0.0
    while elapsed < duration:
        h = random_hit_pattern()
        speed_r, speed_l = motor_for(*h)
        await broadcast(j_sensor(h, "BANGBANG", speed_r, speed_l))
        await asyncio.sleep(SENSOR_INTERVAL_S)
        elapsed += SENSOR_INTERVAL_S


# ================= ALUR MISI (FASE 1-5) =================
async def run_mission(start):
    full_route = ACO_ROUTES[start]
    # Robot ASLI berhenti (FINISHED) begitu 6 node BERBEDA sudah dikunjungi
    # -- tidak benar-benar menempuh edge terakhir yang balik ke start lagi
    # (lihat nVisited>=N di robot_sabita.ino). aco_route/aco_length tetap
    # pakai siklus penuh (persis field route/routecompare asli), tapi
    # perjalanan yang disimulasikan cuma 5 edge nyata.
    travel_route = full_route[:-1]
    n = len(travel_route)
    aco_length = route_dist(full_route)

    actual_route = []
    actual_distance = 0.0

    print(f"\n[simulator] === MULAI MISI dari start node {start} ===")
    print(f"[simulator] Rute ACO (referensi): {'-'.join(full_route)} ({aco_length:.2f} m)\n")

    # FASE 1 - IDLE
    await broadcast(j_state("IDLE"))
    await asyncio.sleep(2)

    for i, node in enumerate(travel_route):
        prev_node = travel_route[i - 1] if i > 0 else "-"
        is_last = (i == n - 1)
        nxt = "SELESAI" if is_last else ("-" if i == 0 else travel_route[i + 1])
        audio_code = f"{NODES.index(node) + 1:03d}"
        qr_text = f"Node {node}, {NAMA_KARYA[node]}, Audio {audio_code}"

        # --- FASE 2/4: SCAN QR & ARRIVED ---
        await broadcast(j_qr(qr_text))
        await broadcast(j_state("ARRIVED"))
        actual_route.append(node)
        if i > 0:
            actual_distance += edge_dist(prev_node, node)
        await broadcast(j_nav(prev_node, node, nxt, NAMA_KARYA[node], i + 1, len(NODES)))
        if i == 1:
            # Persis firmware: rute ACO baru "diketahui" & dibroadcast SEKALI
            # begitu node ke-2 tercapai (arah gerak alami robot baru diketahui).
            await broadcast(j_route(full_route))
        await broadcast(j_pidmode("OFF"))
        await broadcast(j_dfp("playing", NAMA_KARYA[node], f"Audio {audio_code}"))
        print(f"[simulator] ARRIVED {node} ({NAMA_KARYA[node]}) -- audio {AUDIO_DURATION_S}s")
        await sensor_tick_stationary(OVERHEAD_S + AUDIO_DURATION_S)
        await broadcast(j_dfp("ready"))

        if is_last:
            break

        # --- FASE 3: MOVING ke node berikutnya ---
        await broadcast(j_state("MOVING"))
        await broadcast(j_pidmode("BANGBANG"))
        d = edge_dist(node, travel_route[i + 1])
        travel_time = d / ROBOT_SPEED_MS
        print(f"[simulator] MOVING {node} -> {travel_route[i + 1]} ({d:.2f} m, ~{travel_time:.1f}s)")
        await sensor_tick_moving(travel_time)

    # FASE 5 - FINISHED
    await broadcast(j_pidmode("OFF"))
    await broadcast(j_state("FINISHED"))
    efficiency = (aco_length / actual_distance * 100.0) if actual_distance > 0 else 0.0
    await broadcast(j_routecompare(full_route, aco_length, actual_route, actual_distance, efficiency))

    print("\n=== MISI SELESAI (SIMULASI) ===")
    print(f"Start        : {start}")
    print(f"Rute aktual  : {'-'.join(actual_route)} ({actual_distance:.2f} m)")
    print(f"Rute ACO     : {'-'.join(full_route)} ({aco_length:.2f} m)")
    print(f"Efisiensi    : {efficiency:.1f}%")


# ================= ENTRYPOINT =================
def prompt_start_node():
    while True:
        choice = input("Pilih start node (A/B/C/D/E/F): ").strip().upper()
        if choice in NODES:
            return choice
        print("Input tidak valid, coba lagi (A/B/C/D/E/F).")


async def main_async(port):
    global WS_PORT
    WS_PORT = port
    async with websockets.serve(ws_handler, WS_HOST, port):
        print(f"[simulator] WS server aktif di ws://{WS_HOST}:{port}/ (berperan sbg ESP32 palsu)")
        print(f"[simulator] Arahkan sabita_server.py ke sini, contoh:")
        print(f"[simulator]   python tools/sabita_server.py --esp-host 127.0.0.1 --esp-port {port}\n")
        await wait_for_connection()
        start = await asyncio.to_thread(prompt_start_node)
        await run_mission(start)
        print("\n[simulator] Server WS tetap aktif (dashboard masih bisa lihat state terakhir). Ctrl+C utk keluar.")
        await asyncio.Future()


def main():
    ap = argparse.ArgumentParser(description="SABITA simulator (ESP32 palsu, tanpa robot fisik)")
    ap.add_argument("--port", type=int, default=WS_PORT, help=f"Port WS server simulator (default {WS_PORT})")
    args = ap.parse_args()

    print("=== SABITA SIMULATOR (ESP32 palsu, tanpa robot fisik) ===")
    try:
        asyncio.run(main_async(args.port))
    except KeyboardInterrupt:
        print("\n[simulator] dihentikan.")


if __name__ == "__main__":
    main()
