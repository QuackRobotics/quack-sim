"""Final projector clearance table (Atlas, 2026-11-24). Model: upstream ODM b23317a robot.xml.
Envelope (Woz, PR #8 90bde49): ?12x50 cylinder, lens face (46,-45,115.9) trunk_assembly mm, axis 8 deg down,
enlarged 1.2 mm for print tolerance -> radius 7.2 mm, length -1.2..51.2 mm along axis (ends treated as round = conservative).
Stage 1: 5 deg grid over full XML ranges, every 7th vertex. Stage 2: nodes < 14 mm recomputed with all vertices,
min over antennas {0,0} and the 4 (+-90,+-90) corners. Stage 3: random in-cell validation -> margin m for the rule
"target allowed iff min over the 16 enclosing grid nodes >= 4 + m". Usage: table.py <robot_dir>"""
import sys, itertools, time, numpy as np
from multiprocessing import Pool
sys.path.insert(0, '.')
d = sys.argv[1]; sys.argv = sys.argv[:2]
src = open('projclear.py').read().split('\nmodes = ')[0]
exec(src)
RC, S0, S1, CLR = 7.2, -1.2, 51.2, 4.0
a = np.array([46, -45.0, z0])
names = ['neck_pitch', 'head_pitch', 'head_yaw', 'head_roll']
ANT = [(0, 0), (-90, -90), (-90, 90), (90, -90), (90, 90)]
STEP = [7]
_v = verts
def verts(m, step=7):
    key = (m, STEP[0])
    if key not in vc:
        cache.pop(m, None); vc[key] = _v(m, STEP[0]); cache.pop(m, None)
    return vc[key]
vc = {}
def clr(q, step):
    STEP[0] = step
    pts = []; collect(neck, np.eye(3), np.zeros(3), q, pts)
    P = np.vstack(pts) - a; s = np.clip(P @ axis, S0, S1)
    return float((np.linalg.norm(P - np.outer(s, axis), axis=1) - RC).min())
def q_of(v, ant=(0, 0)):
    q = dict(zip(names, np.radians(v))); q['left_antenna'], q['right_antenna'] = np.radians(ant); return q
def coarse(v): return clr(q_of(v), 7)
def fine(v): return min(clr(q_of(v, an), 1) for an in ANT)
ax = [np.arange(-20, 65.1, 5), np.arange(-45, 45.1, 5), np.arange(-160, 160.1, 5), np.arange(-30, 30.1, 5)]
if __name__ == '__main__':
    t = time.time(); nodes = list(itertools.product(*ax))
    with Pool(8) as p:
        C = np.array(p.map(coarse, nodes, chunksize=200))
        print(f'stage1 {len(nodes)} nodes {time.time()-t:.0f}s min {C.min():.1f} mm', flush=True)
        idx = np.where(C < 14)[0]
        F = np.array(p.map(fine, [nodes[i] for i in idx], chunksize=20))
        T = C.copy(); T[idx] = F
        print(f'stage2 {len(idx)} nodes refined {time.time()-t:.0f}s; coarse-fine worst {(C[idx]-F).max():.1f} mm; table min {T.min():.1f}', flush=True)
        T = T.reshape([len(x) for x in ax])
        np.savez('table_b23317a.npz', clearance_mm=T, neck_pitch=ax[0], head_pitch=ax[1], head_yaw=ax[2], head_roll=ax[3])
        rng = np.random.default_rng(20261124)
        lo = np.array([x[0] for x in ax]); hi = np.array([x[-1] for x in ax])
        S = rng.uniform(lo, hi, (40000, 4))
        def cellmin(v):
            i = [min(int((v[k]-ax[k][0])//5), len(ax[k])-2) for k in range(4)]
            return T[i[0]:i[0]+2, i[1]:i[1]+2, i[2]:i[2]+2, i[3]:i[3]+2].min()
        M = np.array([cellmin(v) for v in S])
        near = np.where(M < 14)[0]
        Tv = np.array(p.map(fine, [tuple(S[i]) for i in near], chunksize=20))
        over = M[near] - Tv
        print(f'stage3 {len(near)} in-cell samples {time.time()-t:.0f}s; worst cellmin-true {over.max():.2f} mm', flush=True)
        m = max(0.0, float(over.max()))
        allowed_bad = ((M[near] >= CLR + m) & (Tv < CLR)).sum()
        print(f'margin m = {m:.2f} mm; rule cellmin >= {CLR+m:.2f}: samples allowed but true<4: {allowed_bad}')
        print(f'nodes >= {CLR+m:.2f}: {(T >= CLR+m).mean()*100:.1f}% ; nodes < 4: {(T<CLR).sum()} of {T.size}')
        for k, n in enumerate(names):
            print(' ', n, 'nodes<4 per value:', {float(x): int((np.take(T, j, axis=k) < CLR).sum()) for j, x in enumerate(ax[k]) if (np.take(T, j, axis=k) < CLR).any()})
        np.save('table_validate.npy', np.c_[S[near], M[near], Tv])
        box = (ax[0][:, None, None, None] >= 0) & (ax[1][None, :, None, None] >= -22.5) & (np.abs(ax[3])[None, None, None, :] <= 22.5)
        box = np.broadcast_to(box, T.shape)
        print(f'interim box nodes under final table: min {T[box].min():.1f} mm, nodes < 4: {(T[box] < CLR).sum()} of {box.sum()}')
        print(f'neutral {T[4, 9, 32, 6]:.1f} mm; done {time.time()-t:.0f}s')
