"""Per-mesh breakdown of negative projector-clearance nodes (Atlas, 2026-11-24).
Reuses table.py FK/envelope exactly (r7.2, s -1.2..51.2, all vertices). For every node with table value < 0:
clearance per (body/mesh), antenna-chain meshes min over the 5 ANT configs, others at antennas 0.
Checks overall per-node min == table value. Output permesh.npz + summary on stdout."""
import sys, time, numpy as np
from multiprocessing import Pool
NP = int(sys.argv[1]) if len(sys.argv) > 1 else 3
sys.argv = ['table.py', '/home/naoufel/quack-core/hinton/torque_v2/odm/mini_bdx/robots/open_duck_mini_v2']
exec(open('table.py').read().split("\nif __name__ == '__main__':")[0])

def collect_l(body, Rp, pp, q, out, ant=False, only_ant=False):
    Rb = Rp @ R(f3(body.get('quat'), '1 0 0 0')); pb = pp + Rp @ f3(body.get('pos'), '0 0 0')
    j = body.find('joint')
    if j is not None:
        Rb = Rb @ axang(f3(j.get('axis'), '0 0 1'), q.get(j.get('name'), 0.0))
        ant = ant or 'antenna' in j.get('name')
    for g in body.findall('geom'):
        if g.get('mesh') and (ant or not only_ant):
            v = verts(g.get('mesh'))
            Rg = Rb @ R(f3(g.get('quat'), '1 0 0 0')); pg = pb + Rb @ f3(g.get('pos'), '0 0 0')
            out.append((f"{body.get('name')}/{g.get('mesh')}", ant, (v @ Rg.T + pg) * 1000))
    for c in body.findall('body'):
        collect_l(c, Rb, pb, q, out, ant, only_ant)

def per_geom(q, only_ant=False):
    STEP[0] = 1; out = []; collect_l(neck, np.eye(3), np.zeros(3), q, out, False, only_ant); res = {}
    for lab, ant, P in out:
        P = P - a; s = np.clip(P @ axis, S0, S1); r = np.linalg.norm(P - np.outer(s, axis), axis=1) - RC
        i = r.argmin(); c = (float(r[i]), float(s[i]), *(P[i] + a).tolist(), ant)
        if lab not in res or c[0] < res[lab][0]: res[lab] = c
    return res

def node(v):
    base = per_geom(q_of(v, (0, 0)))
    for an in ANT[1:]:
        for lab, c in per_geom(q_of(v, an), True).items():
            if c[-1] and c[0] < base[lab][0]: base[lab] = c
    return base

if __name__ == '__main__':
    T = np.load('table_b23317a.npz')['clearance_mm']
    import os; idx = np.argwhere(T < 0)[:int(os.environ.get('LIM', 10**9))]; V = [tuple(float(ax[k][i[k]]) for k in range(4)) for i in idx]
    t = time.time()
    with Pool(NP) as p:
        Rs = p.map(node, V, chunksize=10)
    labs = sorted({l for r in Rs for l in r})
    C = np.array([[r[l][0] for l in labs] for r in Rs])
    W = np.array([[r[l][1:5] for l in labs] for r in Rs])
    tv = T[tuple(idx.T)]
    print(f'{len(V)} negative nodes, {time.time()-t:.0f}s; max |min_mesh - table| = {np.abs(C.min(1)-tv).max():.3f} mm')
    cul = C.argmin(1)
    for k in np.argsort([-(cul == k).sum() for k in range(len(labs))]):
        neg = C[:, k] < 0
        if not neg.any(): continue
        j = C[:, k].argmin(); w = W[j, k]
        print(f'{labs[k]}: culprit {int((cul==k).sum())} nodes, negative at {int(neg.sum())}, min {C[j,k]:.1f} mm at '
              f'{V[j]} (neck,hp,yaw,roll); closest vertex s={w[0]:.1f} mm along tube, trunk ({w[1]:.0f},{w[2]:.0f},{w[3]:.0f})')
        S = W[neg, k, 0]; print(f'    s along tube over its negative nodes: {S.min():.1f}..{S.max():.1f} mm, median {np.median(S):.1f}')
    np.savez('permesh.npz', nodes_deg=np.array(V), table_mm=tv, labels=np.array(labs), clearance_mm=C, closest_s_xyz=W)