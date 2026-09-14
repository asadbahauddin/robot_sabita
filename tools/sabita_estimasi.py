# sabita_estimasi.py
# Generate data estimasi pergerakan robot SABITA
# untuk 6 start node berbeda (A-F)
# Jalankan: python tools/sabita_estimasi.py
# Output: 3 file CSV di folder logs/
"""
Tool estimasi OFFLINE, berdiri sendiri -- TIDAK butuh robot nyala, TIDAK
butuh WiFi, TIDAK import apapun dari sabita_server.py. Semua topologi
(jarak antar node, nama karya) di-hardcode di bawah, identik dengan
distMatrix/NNAME/NART di robot_sabita.ino dan simulasi/sabita_topology.py.

Catatan pemodelan (penting, biar hasil konsisten sama perilaku FSM asli):
Rute ACO yang diberikan per start node adalah SIKLUS TERTUTUP (balik lagi
ke node awal) -- itu dipakai APA ADANYA untuk kolom aco_route/aco_dist_m
di summary, persis seperti bestL di firmware yang juga menghitung jarak
siklus tertutup buat metrik ACO. TAPI robot ASLI berhenti (FINISHED) begitu
sudah mengunjungi 6 node BERBEDA (lihat robotState/nVisited>=N di
robot_sabita.ino) -- dia TIDAK benar-benar menempuh edge terakhir yang
balik ke node awal lagi. Jadi simulasi nav/sensor di sini cuma menempuh
5 edge pertama (6 node berbeda, TERMASUK node awal), bukan 6 edge penuh --
biar waktu & jarak tempuh yang diestimasi mencerminkan misi nyata, bukan
jarak metrik ACO semata.
"""

import csv
import os
from datetime import datetime, timedelta

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


# ================= PARAMETER ESTIMASI (bisa diubah) =================
MOTOR_SPEED       = 70     # PWM base line-follower (identik MOTOR_SPEED default di firmware)
ROBOT_SPEED_MS    = speed_for_pwm(MOTOR_SPEED)  # m/s kecepatan robot -- ~0.20 m/s pada PWM=70 (lihat karakterisasi di atas)
AUDIO_DURATION    = 8.5    # detik per audio
OVERHEAD_PER_NODE = 5.0    # detik overhead QR + berhenti
NOISE_TRAVEL      = 15.0   # detik rata-rata nyasar (dibagi rata ke tiap edge tempuh)

SENSOR_DT    = 0.05   # detik antar sample sensor simulasi

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

# Rute ACO terbaik per start node (hasil ACO offline, HARDCODE -- bukan
# dihitung ulang di sini). Siklus tertutup (balik lagi ke start di akhir).
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


def route_str(route):
    return '->'.join(route)


def route_dist(route):
    return sum(edge_dist(route[i], route[i + 1]) for i in range(len(route) - 1))


# ================= SIMULASI SENSOR LINE FOLLOWER (per segmen) =================
# Konvensi nilai sensor SAMA seperti CSV nyata (sSensor()/log_sensor_row di
# sabita_server.py): 0 = kena garis hitam (LOW), 1 = putih (HIGH).
# Logika arah/motor di bawah ini MENIRU PERSIS prioritas if/elif di
# lineFollow() (s3 > s2 > s4 > s1 > s6) dan klasifikasi `arah` di loop()
# firmware, supaya data simulasi realistis & konsisten dengan robot asli.

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


def pos_for(h1, h2, h3, h4, h6):
    if h3: return 0.0
    if h2: return -1.0
    if h4: return 1.0
    if h1: return -2.0
    if h6: return 2.0
    return 0.0


TURN_TYPES = ['gentle_kiri', 'gentle_kanan', 'tajam_kiri', 'tajam_kanan']
TURN_HITS = {
    'gentle_kiri':  (False, True,  False, False, False),  # s2=kena -> koreksi kiri
    'gentle_kanan': (False, False, False, True,  False),  # s4=kena -> koreksi kanan
    'tajam_kiri':   (True,  False, False, False, False),  # s1=kena -> belok kiri tajam
    'tajam_kanan':  (False, False, False, False, True),   # s6=kena -> belok kanan tajam
}
NODE_ZONE_S   = 0.25   # detik "persimpangan" di awal & akhir tiap segmen (keluar/masuk zona node)
STRAIGHT_CHUNK_S = 1.2  # detik lurus di antara tiap koreksi belok
TURN_DUR_S    = 0.20   # lama satu koreksi belok


def build_turn_intervals(duration):
    intervals = []
    t = NODE_ZONE_S + STRAIGHT_CHUNK_S
    i = 0
    while t + TURN_DUR_S < duration - NODE_ZONE_S:
        intervals.append((t, t + TURN_DUR_S, TURN_TYPES[i % len(TURN_TYPES)]))
        t += TURN_DUR_S + STRAIGHT_CHUNK_S
        i += 1
    return intervals


def gen_segment_sensor_rows(start_node, seg_label, t0, duration):
    rows = []
    intervals = build_turn_intervals(duration)
    t = 0.0
    while t < duration:
        if t < NODE_ZONE_S or t > duration - NODE_ZONE_S:
            h1, h2, h3, h4, h6 = True, False, True, True, False  # persimpangan: 3 sensor kena
        else:
            hit = None
            for a, b, ttype in intervals:
                if a <= t < b:
                    hit = TURN_HITS[ttype]
                    break
            h1, h2, h3, h4, h6 = hit if hit else (False, False, True, False, False)  # default lurus

        speed_r, speed_l = motor_for(h1, h2, h3, h4, h6)
        rows.append({
            'start_node': start_node,
            't_rel_s': round(t0 + t, 2),
            'segment': seg_label,
            's1': 0 if h1 else 1, 's2': 0 if h2 else 1, 's3': 0 if h3 else 1,
            's4': 0 if h4 else 1, 's6': 0 if h6 else 1,
            'arah': arah_for(h1, h2, h3, h4, h6),
            'pos': pos_for(h1, h2, h3, h4, h6),
            'speedR': speed_r, 'speedL': speed_l,
        })
        t = round(t + SENSOR_DT, 6)
    return rows


# ================= FIELD ORDER CSV =================
SENSOR_FIELDS = ['start_node', 't_rel_s', 'segment', 's1', 's2', 's3', 's4', 's6',
                  'arah', 'pos', 'speedR', 'speedL']
NAV_FIELDS = ['start_node', 'step', 'node', 'art', 'prev_node', 'next_node',
              't_rel_s', 't_abs', 'dist_to_node_m', 'travel_time_s',
              'audio_time_s', 'est_total_time_s', 'event']
SUMMARY_FIELDS = ['start_node', 'aco_route', 'aco_dist_m', 'total_time_s', 'total_time_menit',
                   'node_A_arrive_s', 'node_B_arrive_s', 'node_C_arrive_s',
                   'node_D_arrive_s', 'node_E_arrive_s', 'node_F_arrive_s',
                   'avg_travel_time_s', 'avg_audio_time_s']


def write_csv(path, fields, rows):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow(row)


def main():
    run_ts = datetime.now()
    ts_str = run_ts.strftime('%Y%m%d_%H%M%S')

    here = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(here, 'logs')
    os.makedirs(log_dir, exist_ok=True)

    sensor_path  = os.path.join(log_dir, f'estimasi_sensor_{ts_str}.csv')
    nav_path     = os.path.join(log_dir, f'estimasi_nav_{ts_str}.csv')
    summary_path = os.path.join(log_dir, f'estimasi_summary_{ts_str}.csv')

    sensor_rows, nav_rows, summary_rows = [], [], []

    for start in NODES:
        full_route = ACO_ROUTES[start]              # siklus tertutup (7 node, balik ke start)
        aco_dist = route_dist(full_route)            # jarak metrik ACO (siklus penuh)

        # Robot ASLI berhenti begitu 6 node BERBEDA sudah dikunjungi -- tidak
        # benar-benar menempuh edge terakhir yg balik ke start lagi.
        travel_route = full_route[:-1]                # 6 node berbeda (termasuk start)
        n_edges = len(travel_route) - 1                # 5 edge tempuh nyata
        noise_per_segment = NOISE_TRAVEL / n_edges

        cumulative_t = 0.0
        node_arrive = {}
        travel_times, audio_times = [], []

        for i, node in enumerate(travel_route):
            prev_node = travel_route[i - 1] if i > 0 else '-'
            next_node = travel_route[i + 1] if i + 1 < len(travel_route) else '-'

            if i == 0:
                d, travel_time = 0.0, 0.0
            else:
                d = edge_dist(travel_route[i - 1], node)
                travel_time = d / ROBOT_SPEED_MS + noise_per_segment
                travel_times.append(travel_time)
                seg_label = f'{travel_route[i - 1]}->{node}'
                sensor_rows.extend(gen_segment_sensor_rows(start, seg_label, cumulative_t, travel_time))

            t_scan = cumulative_t + travel_time
            nav_rows.append({
                'start_node': start, 'step': i, 'node': node, 'art': NAMA_KARYA[node],
                'prev_node': prev_node, 'next_node': next_node,
                't_rel_s': round(t_scan, 2),
                't_abs': (run_ts + timedelta(seconds=t_scan)).strftime('%H:%M:%S'),
                'dist_to_node_m': round(d, 2), 'travel_time_s': round(travel_time, 2),
                'audio_time_s': 0.0, 'est_total_time_s': round(t_scan, 2), 'event': 'SCAN_QR',
            })

            t_arrived = t_scan + OVERHEAD_PER_NODE
            audio_time = AUDIO_DURATION
            audio_times.append(audio_time)
            t_after_audio = t_arrived + audio_time
            is_last = (i == len(travel_route) - 1)
            nav_rows.append({
                'start_node': start, 'step': i, 'node': node, 'art': NAMA_KARYA[node],
                'prev_node': prev_node, 'next_node': next_node,
                't_rel_s': round(t_arrived, 2),
                't_abs': (run_ts + timedelta(seconds=t_arrived)).strftime('%H:%M:%S'),
                'dist_to_node_m': round(d, 2), 'travel_time_s': round(travel_time, 2),
                'audio_time_s': audio_time, 'est_total_time_s': round(t_after_audio, 2),
                'event': 'FINISHED' if is_last else 'ARRIVED',
            })

            if node not in node_arrive:
                node_arrive[node] = round(t_arrived, 2)

            cumulative_t = t_after_audio

        total_time_s = cumulative_t
        avg_travel = sum(travel_times) / len(travel_times) if travel_times else 0.0
        avg_audio = sum(audio_times) / len(audio_times) if audio_times else 0.0

        summary_rows.append({
            'start_node': start, 'aco_route': route_str(full_route), 'aco_dist_m': round(aco_dist, 2),
            'total_time_s': round(total_time_s, 2), 'total_time_menit': round(total_time_s / 60, 2),
            'node_A_arrive_s': node_arrive.get('A', ''),
            'node_B_arrive_s': node_arrive.get('B', ''),
            'node_C_arrive_s': node_arrive.get('C', ''),
            'node_D_arrive_s': node_arrive.get('D', ''),
            'node_E_arrive_s': node_arrive.get('E', ''),
            'node_F_arrive_s': node_arrive.get('F', ''),
            'avg_travel_time_s': round(avg_travel, 2),
            'avg_audio_time_s': round(avg_audio, 2),
        })

    write_csv(sensor_path, SENSOR_FIELDS, sensor_rows)
    write_csv(nav_path, NAV_FIELDS, nav_rows)
    write_csv(summary_path, SUMMARY_FIELDS, summary_rows)

    print('=== ESTIMASI SABITA - 6 START NODE ===')
    for row in summary_rows:
        print(f"Start {row['start_node']} | {row['aco_route']} | {row['aco_dist_m']}m "
              f"| ~{row['total_time_menit']:.1f} menit")
    print()
    print('File tersimpan di:')
    print(f'  {sensor_path}')
    print(f'  {nav_path}')
    print(f'  {summary_path}')


if __name__ == '__main__':
    main()
