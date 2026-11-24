"""Roll-box clearance from the final projector table (Atlas, 2026-11-24).

Input: ../projector_envelope/table_b23317a.npz, made by projector_envelope/table.py on razer
(upstream ODM b23317a robot.xml, meshes as loaded by projclear.py <robot_dir>, 5 deg grid over full XML ranges,
nodes < 14 mm recomputed with all vertices and antenna corners; in-cell margin m = 1.73 mm -> bar 5.73 mm).

Rule (table.py): a target is allowed iff the min over its enclosing grid nodes >= 4 + m = 5.73 mm.
For a box we take every node of every cell that touches the box (floor/ceil to the grid), so it is conservative.

Usage: rollbox.py NECK_LO NECK_HI PITCH_LO PITCH_HI YAW_LO YAW_HI ROLL_LO ROLL_HI [label]
All angles in degrees, absolute joint angle (default pose = 0 for all head joints).
"""
import sys

import numpy as np

BAR = 5.73
Z = np.load("../projector_envelope/table_b23317a.npz")
T = Z["clearance_mm"]
AX = [Z[k] for k in ("neck_pitch", "head_pitch", "head_yaw", "head_roll")]


def cell_slice(a, lo, hi):
    i0 = max(0, int(np.searchsorted(a, lo, "right")) - 1)
    i1 = min(len(a), int(np.searchsorted(a, hi, "left")) + 1)
    return slice(i0, i1)


def box(b):
    s = tuple(cell_slice(AX[k], b[2 * k], b[2 * k + 1]) for k in range(4))
    sub = T[s]
    j = np.unravel_index(sub.argmin(), sub.shape)
    worst = [float(AX[k][s[k]][j[k]]) for k in range(4)]
    return float(sub.min()), worst, int((sub < BAR).sum()), int(sub.size)


if __name__ == "__main__":
    b = [float(x) for x in sys.argv[1:9]]
    label = sys.argv[9] if len(sys.argv) > 9 else ""
    mn, worst, nbad, n = box(b)
    verdict = "PASS" if mn >= BAR else "FAIL"
    print(f"{label:22s} box {b} -> min {mn:.2f} mm at (neck,pitch,yaw,roll)={worst}; "
          f"nodes<{BAR}: {nbad}/{n}; {verdict}")
