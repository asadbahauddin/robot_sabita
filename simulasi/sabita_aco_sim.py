"""
SABITA - Simulasi Visual Rute ACO & Line Follower
====================================================
Mereplikasi logika ACO (aco_sabita) dan FSM robot pada robot_sabita.ino
untuk menghasilkan animasi GIF: graf 6 node, robot bergerak mengikuti
rute ACO terbaik, panel navigasi, dan simulasi sensor S1-S6.

Jalankan:
    python sabita_aco_sim.py [START_NODE]

Contoh:
    python sabita_aco_sim.py A
"""

import math
import random
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

from sabita_topology import NODES, N_NODE, NO_EDGE, EDGE_DIST, D, NAMA_KARYA, POS

# ============================================================
#  PARAMETER ACO (identik dengan firmware)
# ============================================================
ALPHA = 1.0
BETA = 2.0
RHO = 0.3
N_ANTS = 6
N_ITER = 100
TAU0 = 1.0


def construct_route(start_idx, pheromone, rng):
    """Replika constructRoute() di firmware: nearest-neighbour probabilistik
    berbasis pheromone*heuristic, dengan fallback bila node terkunci."""
    visited = [False] * N_NODE
    route = [start_idx]
    visited[start_idx] = True
    current = start_idx
    step = 1
    tries = 0
    max_tries = 100

    while step < N_NODE and tries < max_tries:
        tries += 1
        scores = np.zeros(N_NODE)
        total = 0.0
        has_candidate = False

        for j in range(N_NODE):
            if not visited[j] and D[current][j] < NO_EDGE:
                tau = pheromone[current][j] ** ALPHA
                eta = (1.0 / D[current][j]) ** BETA
                scores[j] = tau * eta
                total += scores[j]
                has_candidate = True

        if not has_candidate:
            best_fallback, best_fallback_dist = -1, NO_EDGE
            for j in range(N_NODE):
                if D[current][j] < NO_EDGE and j != current:
                    for k in range(N_NODE):
                        if not visited[k] and D[j][k] < NO_EDGE:
                            if D[current][j] < best_fallback_dist:
                                best_fallback_dist = D[current][j]
                                best_fallback = j
                            break
            if best_fallback == -1:
                return None
            current = best_fallback
            continue

        r = rng.random() * total
        cumulative = 0.0
        chosen = -1
        for j in range(N_NODE):
            if scores[j] > 0:
                cumulative += scores[j]
                if cumulative >= r:
                    chosen = j
                    break
        if chosen == -1:
            chosen = next(j for j in range(N_NODE) if scores[j] > 0)

        route.append(chosen)
        visited[chosen] = True
        current = chosen
        step += 1

    return route if step >= N_NODE else None


def route_length(route):
    total = sum(D[route[i]][route[i + 1]] for i in range(N_NODE - 1))
    total += D[route[-1]][route[0]]
    return total


def update_pheromone(pheromone, all_routes, all_lengths):
    pheromone *= (1.0 - RHO)
    pheromone[pheromone < 0.0001] = 0.0001
    for route, length in zip(all_routes, all_lengths):
        deposit = 1.0 / length
        for i in range(N_NODE - 1):
            a, b = route[i], route[i + 1]
            pheromone[a][b] += deposit
            pheromone[b][a] += deposit
        a, b = route[-1], route[0]
        pheromone[a][b] += deposit
        pheromone[b][a] += deposit


def run_aco(start_idx, seed=42):
    rng = random.Random(seed)
    pheromone = np.full((N_NODE, N_NODE), TAU0)
    best_route, best_length = None, float('inf')

    for _ in range(N_ITER):
        all_routes, all_lengths = [], []
        for _ in range(N_ANTS):
            route = construct_route(start_idx, pheromone, rng)
            if route is not None:
                length = route_length(route)
                all_routes.append(route)
                all_lengths.append(length)
                if length < best_length:
                    best_length = length
                    best_route = route
        if all_routes:
            update_pheromone(pheromone, all_routes, all_lengths)

    return best_route, best_length


# ============================================================
#  SIMULASI SENSOR LINE FOLLOWER (S1..S6, ilustratif)
#  Mengikuti konvensi firmware setelah fix: 1 = terdeteksi garis (hitam)
# ============================================================
def simulate_sensors(progress, frame_idx, at_node):
    if at_node:
        return {'s1': 0, 's2': 1, 's3': 1, 's4': 1, 's5': None, 's6': 0}, "PERSIMPANGAN (NODE)"

    near_junction = progress < 0.12 or progress > 0.88
    if near_junction:
        return {'s1': 0, 's2': 1, 's3': 1, 's4': 1, 's5': None, 's6': 0}, "MENDEKATI NODE"

    wiggle = math.sin(frame_idx * 0.6)
    if wiggle > 0.35:
        return {'s1': 0, 's2': 1, 's3': 1, 's4': 0, 's5': None, 's6': 0}, "KOREKSI KANAN"
    elif wiggle < -0.35:
        return {'s1': 0, 's2': 0, 's3': 1, 's4': 1, 's5': None, 's6': 0}, "KOREKSI KIRI"
    return {'s1': 0, 's2': 0, 's3': 1, 's4': 0, 's5': None, 's6': 0}, "LURUS"


# ============================================================
#  BANGUN TIMELINE FSM (mereplikasi alur robotState di firmware)
# ============================================================
class Frame:
    __slots__ = ('state', 'prev', 'cur', 'nxt', 'progress', 'nodes_visited',
                 'route_str', 'route_len', 'frame_idx', 'pos', 'edge_from', 'edge_to')

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


def build_timeline(full_route_idx, route_str, route_len, start_char):
    frames = []
    fi = 0

    def node_char(i):
        return NODES[i]

    # --- WAIT_START ---
    for _ in range(6):
        frames.append(Frame(state="WAIT_START", prev="-", cur=start_char, nxt="-",
                             progress=0, nodes_visited=0, route_str="Rute belum dihitung",
                             route_len=None, frame_idx=fi, pos=POS[start_char],
                             edge_from=start_char, edge_to=start_char))
        fi += 1

    # --- READY ---
    for _ in range(6):
        frames.append(Frame(state="READY", prev="-", cur=start_char, nxt="-",
                             progress=0, nodes_visited=0, route_str="Rute belum dihitung",
                             route_len=None, frame_idx=fi, pos=POS[start_char],
                             edge_from=start_char, edge_to=start_char))
        fi += 1

    # --- COMPUTING_ACO ---
    for _ in range(8):
        frames.append(Frame(state="COMPUTING", prev="-", cur=start_char, nxt="-",
                             progress=0, nodes_visited=0, route_str="Menghitung ACO...",
                             route_len=None, frame_idx=fi, pos=POS[start_char],
                             edge_from=start_char, edge_to=start_char))
        fi += 1

    # --- MOVING / ARRIVED per leg (kembali ke start di leg terakhir) ---
    full_chars = [node_char(i) for i in full_route_idx] + [start_char]
    nodes_visited = 1
    MOVE_FRAMES = 16
    PAUSE_FRAMES = 10

    for leg in range(len(full_chars) - 1):
        a, b = full_chars[leg], full_chars[leg + 1]
        is_last_leg = (leg == len(full_chars) - 2)
        nxt_char = full_chars[leg + 2] if leg + 2 < len(full_chars) else "-"

        # fase MOVING: interpolasi posisi a -> b
        for k in range(MOVE_FRAMES):
            t = (k + 1) / MOVE_FRAMES
            pos = (POS[a][0] + (POS[b][0] - POS[a][0]) * t,
                   POS[a][1] + (POS[b][1] - POS[a][1]) * t)
            frames.append(Frame(state="MOVING", prev=a, cur=b, nxt=nxt_char,
                                 progress=t, nodes_visited=nodes_visited,
                                 route_str=route_str, route_len=route_len,
                                 frame_idx=fi, pos=pos, edge_from=a, edge_to=b))
            fi += 1

        if not is_last_leg:
            # node baru dikunjungi (leg terakhir kembali ke start, sudah terhitung
            # sejak awal, jadi tidak menambah nodes_visited -- meniru nodesVisitedCount
            # firmware yang berhenti di N_NODE)
            nodes_visited += 1
            # fase ARRIVED: berhenti + audio karya
            for _ in range(PAUSE_FRAMES):
                frames.append(Frame(state="ARRIVED", prev=a, cur=b, nxt=nxt_char,
                                     progress=1.0, nodes_visited=nodes_visited,
                                     route_str=route_str, route_len=route_len,
                                     frame_idx=fi, pos=POS[b], edge_from=a, edge_to=b))
                fi += 1
        else:
            # leg terakhir: kembali ke start -> FINISHED (tanpa audio)
            for _ in range(PAUSE_FRAMES):
                frames.append(Frame(state="FINISHED", prev=a, cur=b, nxt="-",
                                     progress=1.0, nodes_visited=nodes_visited,
                                     route_str=route_str, route_len=route_len,
                                     frame_idx=fi, pos=POS[b], edge_from=a, edge_to=b))
                fi += 1

    return frames


# ============================================================
#  RENDER GIF
# ============================================================
STATE_COLOR = {
    "WAIT_START": "#94a3b8", "READY": "#f59e0b", "COMPUTING": "#3b82f6",
    "MOVING": "#22c55e", "ARRIVED": "#f59e0b", "FINISHED": "#a855f7",
}
STATE_LABEL = {
    "WAIT_START": "MENUNGGU SCAN QR START",
    "READY": "SIAP - TEKAN MULAI",
    "COMPUTING": "MENGHITUNG RUTE ACO...",
    "MOVING": "ROBOT BERGERAK",
    "ARRIVED": "SAMPAI DI NODE - MEMUTAR AUDIO",
    "FINISHED": "MISI SELESAI",
}


def draw_frame(ax_graph, ax_panel, frame, full_chars, traveled_edges):
    ax_graph.clear()
    ax_panel.clear()
    ax_panel.axis("off")

    # --- Graf: semua edge (abu-abu tipis) ---
    for (a, b), w in EDGE_DIST.items():
        xa, ya = POS[a]
        xb, yb = POS[b]
        ax_graph.plot([xa, xb], [ya, yb], color="#3a3d4e", lw=1.5, zorder=1)
        mx, my = (xa + xb) / 2, (ya + yb) / 2
        ax_graph.text(mx, my, f"{w:.2f}", fontsize=7, color="#64748b",
                       ha="center", va="center",
                       bbox=dict(boxstyle="round,pad=0.15", fc="#0f1117", ec="none"))

    # --- Edge yang sudah dilalui (hijau) ---
    for (a, b) in traveled_edges:
        xa, ya = POS[a]
        xb, yb = POS[b]
        ax_graph.plot([xa, xb], [ya, yb], color="#22c55e", lw=3, zorder=2, alpha=0.8)

    # --- Edge yang sedang dilalui (oranye, saat MOVING) ---
    if frame.state == "MOVING":
        xa, ya = POS[frame.edge_from]
        xb, yb = POS[frame.edge_to]
        ax_graph.plot([xa, xb], [ya, yb], color="#f59e0b", lw=3, zorder=2)

    # --- Node ---
    for node in NODES:
        x, y = POS[node]
        visited = node in full_chars[:full_chars.index(frame.cur) + 1] if frame.cur in full_chars else False
        color = "#22c55e" if (visited and frame.state != "WAIT_START" and frame.state != "READY" and frame.state != "COMPUTING") else "#252940"
        edge_c = "#3b82f6" if node == full_chars[0] else "#3a3d4e"
        ax_graph.add_patch(mpatches.Circle((x, y), 0.24, facecolor=color,
                                            edgecolor=edge_c, lw=2, zorder=3))
        ax_graph.text(x, y, node, fontsize=15, fontweight="bold", color="white",
                       ha="center", va="center", zorder=4)
        ax_graph.text(x, y - 0.42, NAMA_KARYA[node], fontsize=7, color="#94a3b8",
                       ha="center", va="center", zorder=6,
                       bbox=dict(boxstyle="round,pad=0.15", fc="#0f1117", ec="none", alpha=0.85))

    # --- Robot marker ---
    rx, ry = frame.pos
    ax_graph.add_patch(mpatches.RegularPolygon((rx, ry), numVertices=3, radius=0.15,
                                                orientation=math.pi, facecolor="#ef4444",
                                                edgecolor="white", lw=1, zorder=5))

    ax_graph.set_xlim(-2.8, 2.8)
    ax_graph.set_ylim(-2.4, 2.7)
    ax_graph.set_aspect("equal")
    ax_graph.axis("off")
    ax_graph.set_title("SABITA - Rute ACO Terbaik", color="#e0e0e0", fontsize=13, fontweight="bold")

    # ================= PANEL INFO =================
    y0 = 0.97
    ax_panel.text(0.02, y0, "SABITA ROBOT - SIMULASI", fontsize=13, fontweight="bold",
                  color="#3b82f6", transform=ax_panel.transAxes)

    state_col = STATE_COLOR[frame.state]
    ax_panel.add_patch(mpatches.FancyBboxPatch((0.02, y0 - 0.11), 0.96, 0.08,
                        boxstyle="round,pad=0.01", facecolor="#252940",
                        edgecolor=state_col, lw=1.5, transform=ax_panel.transAxes))
    ax_panel.text(0.5, y0 - 0.07, STATE_LABEL[frame.state], fontsize=10, fontweight="bold",
                  color=state_col, ha="center", va="center", transform=ax_panel.transAxes)

    yy = y0 - 0.20
    ax_panel.text(0.02, yy, "NAVIGASI", fontsize=9, color="#94a3b8", fontweight="bold",
                  transform=ax_panel.transAxes)
    yy -= 0.045
    ax_panel.text(0.04, yy, f"Previous : {frame.prev}", fontsize=9.5, color="#e0e0e0",
                  transform=ax_panel.transAxes, family="monospace")
    yy -= 0.045
    ax_panel.text(0.04, yy, f"Current  : {frame.cur}  ({NAMA_KARYA.get(frame.cur, '-')})",
                  fontsize=9.5, color="#22c55e", fontweight="bold",
                  transform=ax_panel.transAxes, family="monospace")
    yy -= 0.045
    ax_panel.text(0.04, yy, f"Next     : {frame.nxt}", fontsize=9.5, color="#e0e0e0",
                  transform=ax_panel.transAxes, family="monospace")

    yy -= 0.07
    ax_panel.text(0.02, yy, "RUTE ACO TERBAIK", fontsize=9, color="#94a3b8", fontweight="bold",
                  transform=ax_panel.transAxes)
    yy -= 0.045
    route_disp = frame.route_str
    length_disp = f" ({frame.route_len:.2f} m)" if frame.route_len else ""
    ax_panel.text(0.04, yy, f"{route_disp}{length_disp}", fontsize=9, color="#f59e0b",
                  transform=ax_panel.transAxes, wrap=True)

    yy -= 0.08
    ax_panel.text(0.02, yy, "PROGRESS MISI", fontsize=9, color="#94a3b8", fontweight="bold",
                  transform=ax_panel.transAxes)
    yy -= 0.05
    pct = frame.nodes_visited / N_NODE
    ax_panel.add_patch(mpatches.FancyBboxPatch((0.04, yy - 0.015), 0.92, 0.03,
                        boxstyle="round,pad=0.002", facecolor="#2a2d3e", edgecolor="none",
                        transform=ax_panel.transAxes))
    ax_panel.add_patch(mpatches.FancyBboxPatch((0.04, yy - 0.015), 0.92 * pct, 0.03,
                        boxstyle="round,pad=0.002", facecolor="#22c55e", edgecolor="none",
                        transform=ax_panel.transAxes))
    yy -= 0.05
    ax_panel.text(0.04, yy, f"{frame.nodes_visited} / {N_NODE} node dikunjungi", fontsize=8.5,
                  color="#94a3b8", transform=ax_panel.transAxes)

    # ---- Sensor line follower ----
    at_node = frame.state in ("ARRIVED", "FINISHED", "WAIT_START", "READY", "COMPUTING")
    sensors, arah = simulate_sensors(frame.progress, frame.frame_idx, at_node)

    yy -= 0.09
    ax_panel.text(0.02, yy, "LINE FOLLOWER (S1-S6)", fontsize=9, color="#94a3b8",
                  fontweight="bold", transform=ax_panel.transAxes)
    yy -= 0.06
    sensor_labels = ['s1', 's2', 's3', 's4', 's5', 's6']
    box_w = 0.14
    for i, key in enumerate(sensor_labels):
        val = sensors[key]
        x0 = 0.04 + i * (box_w + 0.015)
        if val is None:
            fc, txt, tc = "#1a1d2e", "OFF", "#4a4d5e"
        elif val == 1:
            fc, txt, tc = "#22c55e", "1", "#0f1117"
        else:
            fc, txt, tc = "#2a2d3e", "0", "#94a3b8"
        ax_panel.add_patch(mpatches.FancyBboxPatch((x0, yy - 0.045), box_w, 0.06,
                            boxstyle="round,pad=0.004", facecolor=fc,
                            edgecolor="#3a3d4e", lw=1, transform=ax_panel.transAxes))
        ax_panel.text(x0 + box_w / 2, yy - 0.015, f"S{i + 1}", fontsize=6.5, color=tc,
                      ha="center", va="center", transform=ax_panel.transAxes)
        ax_panel.text(x0 + box_w / 2, yy - 0.032, txt, fontsize=7.5, color=tc,
                      fontweight="bold", ha="center", va="center", transform=ax_panel.transAxes)

    yy -= 0.09
    ax_panel.text(0.04, yy, f"Arah: {arah}", fontsize=9, color="#3b82f6",
                  fontweight="bold", transform=ax_panel.transAxes)

    ax_panel.text(0.02, 0.02, "(S5/GPIO36 dinonaktifkan - hardware bug, lihat firmware)",
                  fontsize=6.5, color="#4a4d5e", transform=ax_panel.transAxes)
    ax_panel.set_xlim(0, 1)
    ax_panel.set_ylim(0, 1)


def main():
    start_char = sys.argv[1].upper() if len(sys.argv) > 1 else 'A'
    if start_char not in NODES:
        print(f"Node awal '{start_char}' tidak valid. Pilih salah satu dari {NODES}.")
        sys.exit(1)
    start_idx = NODES.index(start_char)

    print(f"Menjalankan ACO dari start node = {start_char} ...")
    print(f"Parameter: alpha={ALPHA}, beta={BETA}, rho={RHO}, n_ants={N_ANTS}, n_iter={N_ITER}")
    best_route, best_length = run_aco(start_idx)
    if best_route is None:
        print("ACO gagal menemukan rute (graf tidak lengkap).")
        sys.exit(1)

    route_str = "-".join(NODES[i] for i in best_route) + f"-{start_char}"
    print(f"Rute terbaik : {route_str}")
    print(f"Panjang rute : {best_length:.2f} m")

    frames = build_timeline(best_route, route_str, best_length, start_char)
    full_chars = [NODES[i] for i in best_route] + [start_char]

    fig = plt.figure(figsize=(11.5, 6.2), facecolor="#0f1117")
    gs = fig.add_gridspec(1, 2, width_ratios=[1.15, 1], wspace=0.05)
    ax_graph = fig.add_subplot(gs[0])
    ax_panel = fig.add_subplot(gs[1])
    ax_graph.set_facecolor("#0f1117")
    ax_panel.set_facecolor("#0f1117")

    traveled_edges = []
    last_moving_target = None

    def update(i):
        nonlocal last_moving_target
        frame = frames[i]
        if frame.state == "MOVING" and frame.progress >= 0.999:
            pair = tuple(sorted((frame.edge_from, frame.edge_to)))
            if pair != last_moving_target:
                traveled_edges.append((frame.edge_from, frame.edge_to))
                last_moving_target = pair
        draw_frame(ax_graph, ax_panel, frame, full_chars, traveled_edges)
        return []

    anim = FuncAnimation(fig, update, frames=len(frames), interval=100, blit=False)

    out_path = "sabita_route_simulation.gif"
    print(f"Merender {len(frames)} frame ke {out_path} ...")
    anim.save(out_path, writer=PillowWriter(fps=10))
    plt.close(fig)
    print("Selesai. GIF tersimpan di:", out_path)


if __name__ == "__main__":
    main()
