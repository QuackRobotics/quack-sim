"""Projector clearance over the HAL head box (Atlas, 2026-11-24). Model: upstream ODM b23317a robot.xml.
Reuses table.py (envelope r 7.2 mm, coarse = every 7th vertex, fine = all vertices, min over 5 antenna poses).
5 deg grid: neck_pitch -15..15, head_pitch -15..15, head_yaw -50..50, head_roll -15..15.
coarse() on every node, fine() on nodes with coarse < 25 mm (table.log: coarse-fine worst 18.6 mm), the rest keep coarse.
Rule (table.py): a target is allowed iff min over its enclosing grid nodes >= 4 + m = 5.73 mm.
m = 1.73 mm is EMPIRICAL (worst cellmin-true over 3,967 random in-cell samples), not a proven bound.
Usage: python3 rollbox.py <robot_dir>"""
import sys, itertools, time, numpy as np
from multiprocessing import Pool
__file__ = 'projclear.py'
exec(open('table.py').read().split('\nif __name__')[0])
BAR = 4.0 + 1.73
bx = [np.arange(-15, 15.1, 5), np.arange(-15, 15.1, 5), np.arange(-50, 50.1, 5), np.arange(-15, 15.1, 5)]
def fine_ant(v): return [clr(q_of(v, an), 1) for an in ANT]
SETS = {'roll -5..5 (box roll +-5)': (-5, 5), 'roll -10..5 (A: -8/+5)': (-10, 5),
        'roll -10..10 (B: +-8)': (-10, 10), 'roll -15..5 (overlay to -10.45 / -14.95)': (-15, 5)}
def fmt(v, a=None):
    s = 'neck %+d pitch %+d yaw %+d roll %+d' % tuple(int(x) for x in v)
    return s + ('' if a is None else ' antennas %s' % (a,))
if __name__ == '__main__':
    t = time.time(); nodes = list(itertools.product(*bx)); n = len(nodes)
    print(f'{n} nodes, bar {BAR:.2f} mm (m empirical)', flush=True)
    C = np.empty(n)
    with Pool(4) as p:
        for i, c in enumerate(p.imap(coarse, nodes, chunksize=50)):
            C[i] = c
            if (i + 1) % 1000 == 0 or i + 1 == n: print(f'coarse {i+1}/{n} {time.time()-t:.0f}s', flush=True)
        idx = np.where(C < 25)[0]; print(f'fine on {len(idx)} nodes (coarse min {C.min():.2f})', flush=True)
        T = C.copy(); A = {}
        for k, (i, fa) in enumerate(zip(idx, p.imap(fine_ant, [nodes[i] for i in idx], chunksize=4))):
            T[i] = min(fa); A[i] = ANT[int(np.argmin(fa))]
            if (k + 1) % 100 == 0 or k + 1 == len(idx): print(f'fine {k+1}/{len(idx)} {time.time()-t:.0f}s', flush=True)
    np.savez('rollbox_b23317a.npz', clearance_mm=T.reshape([len(x) for x in bx]), neck_pitch=bx[0], head_pitch=bx[1], head_yaw=bx[2], head_roll=bx[3])
    roll = np.array([v[3] for v in nodes])
    print(f'\n=== results ({time.time()-t:.0f}s); neck/pitch/yaw nodes = box edges, so enclosing set = these nodes ===')
    for name, (lo, hi) in SETS.items():
        sel = np.where((roll >= lo) & (roll <= hi))[0]; j = sel[np.argmin(T[sel])]
        print(f'{name}: min {T[j]:.2f} mm at {fmt(nodes[j], A.get(j))} -> {"PASS" if T[j] >= BAR else "FAIL"} vs {BAR:.2f}')
    print('per roll node min:')
    for r in bx[3]:
        sel = np.where(roll == r)[0]; j = sel[np.argmin(T[sel])]
        print(f'  roll {r:+.0f}: {T[j]:.2f} mm at {fmt(nodes[j], A.get(j))}  n<{BAR:.2f}: {(T[sel] < BAR).sum()}/{len(sel)}')
