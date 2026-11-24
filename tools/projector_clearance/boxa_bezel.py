"""Box A (interim box) with the PRINTED head + both bezels (Atlas 2026-11-24).
Reuses Woz projbez.py (vendored here) FK/envelope/job (same as table.py: lens (46,-45,115.9), 8 deg down, r 7.2, s -1.2..51.2;
nodes <14 mm total or <20 mm bezel redone with all vertices x 5 antenna corners).
Grid = table.py 5 deg nodes covering box A: neck 0..65, pitch -25..45, yaw -160..160, roll -15..15.
Cell rule as rollbox.py: a roll band uses every node from floor(lo/5)*5 to ceil(hi/5)*5; bar 4 + m = 5.73 mm.
Usage: python3 boxa_bezel.py <odm_robot_dir> <print_dir>"""
import sys, os, itertools, time, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # vendored projbez.py
from multiprocessing import Pool
import projbez as P

BAR = 5.73
VAR = 'print head + bezels'
ax = [np.arange(0, 65.1, 5), np.arange(-25, 45.1, 5), np.arange(-160, 160.1, 5), np.arange(-15, 15.1, 5)]
BANDS = [('roll +-5', -5, 5), ('roll -8/+5', -8, 5), ('roll +-8', -8, 8), ('roll +-9', -9, 9),
         ('roll -10.45 (p1 -5.95 - 4.5)', -10.45, 5), ('roll -14.95 (p1 -5.95 - 9)', -14.95, 5), ('roll +-15', -15, 15)]

if __name__ == '__main__':
    t = time.time(); nodes = list(itertools.product(*ax)); shape = [len(x) for x in ax]
    with Pool(8) as p:
        res = p.map(P.job, [(VAR, v) for v in nodes], chunksize=50)
    tot = np.array([r[2] for r in res]).reshape(shape)
    bez = np.array([r[3] for r in res]).reshape(shape)
    who = np.array([r[4] for r in res], dtype=object).reshape(shape)
    np.savez('boxa_bezel.npz', total_mm=tot, bezel_mm=bez, neck=ax[0], pitch=ax[1], yaw=ax[2], roll=ax[3])
    print(f'{len(nodes)} nodes, {VAR}, {time.time()-t:.0f}s', flush=True)
    r = ax[3]
    for name, lo, hi in BANDS:
        sel = (r >= np.floor(lo / 5) * 5) & (r <= np.ceil(hi / 5) * 5)
        for lab, A in (('total', tot), ('bezel', bez)):
            sub = A[..., sel]; i = np.unravel_index(sub.argmin(), sub.shape)
            pose = (ax[0][i[0]], ax[1][i[1]], ax[2][i[2]], r[sel][i[3]])
            part = who[..., sel][i] if lab == 'total' else '-'
            ok = 'PASS' if sub.min() >= BAR else 'FAIL'
            print(f'{name:30s} {lab}: {sub.min():6.2f} mm at neck/pitch/yaw/roll {pose} part {part}  {ok} vs {BAR}', flush=True)
