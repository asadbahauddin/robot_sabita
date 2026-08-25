"""
SABITA - Topologi graf & posisi node (dipakai bersama oleh sabita_aco_sim.py
dan live_monitor.py). Identik dengan distMatrix/NNAME/NART di robot_sabita.ino.
"""

import numpy as np

NODES = ['A', 'B', 'C', 'D', 'E', 'F']
N_NODE = len(NODES)
NO_EDGE = 999.0

EDGE_DIST = {
    ('A', 'B'): 1.07, ('A', 'E'): 1.51, ('A', 'F'): 1.21,
    ('B', 'C'): 1.80, ('B', 'F'): 1.16,
    ('C', 'D'): 1.68, ('C', 'F'): 1.41,
    ('D', 'E'): 1.49, ('D', 'F'): 1.07,
    ('E', 'F'): 1.54,
}

D = np.full((N_NODE, N_NODE), NO_EDGE)
for i in range(N_NODE):
    D[i][i] = 0.0
for (a, b), w in EDGE_DIST.items():
    i, j = NODES.index(a), NODES.index(b)
    D[i][j] = w
    D[j][i] = w

NAMA_KARYA = {
    'A': 'Mona Lisa', 'B': 'The Scream', 'C': 'The Kiss',
    'D': 'Starry Night', 'E': 'Sunflowers', 'F': 'Guernica',
}

# Posisi node sesuai layout banner fisik:
# pentagon luar A(kanan tengah)-B(kanan bawah)-C(kiri bawah)-D(kiri tengah)-E(atas)
# dengan F sebagai hub di tengah.
POS = {
    'E': (0.000, 2.000),
    'A': (1.902, 0.618),
    'B': (1.176, -1.618),
    'C': (-1.176, -1.618),
    'D': (-1.902, 0.618),
    'F': (0.000, 0.000),
}
