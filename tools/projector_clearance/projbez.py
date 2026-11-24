# Vendored from Woz: razer ~/quack-core/woz/eyefit/projbez.py (not under git, so no commit hash).
# Source sha1 a59be376dd14d8a7f8514d0d221db76d22e457c3, mtime 2026-11-24 15:47 CET. Body below is byte-identical apart from the stripped UTF-8 BOM.
"""Projector-tube clearance with the PRINTED head + both bezels (Woz 2026-11-24).
Same FK and envelope as Atlas table.py (b23317a robot.xml; lens face (46,-45,115.9) trunk mm, axis 8 deg down,
radius 7.2 mm, -1.2..51.2 mm along axis). Variants: sim head (baseline), print/head.stl, print head + left_eye + right_eye
(bezels placed with the head geom transform, as print/ STLs share the head print frame; mm -> m).
Grid over the ruled HAL clamp: neck +-15, head pitch +-15, yaw +-50, roll +-9 (all >= ruled), 2.5/2.5/5/3 deg.
Stage 1 every 7th vertex; nodes < 14 mm redone with all vertices and antenna corners. Bezel-only minimum reported
separately (all vertices). Usage: python3 projbez.py <odm_robot_dir> <print_dir>"""
import sys, itertools, numpy as np, trimesh, xml.etree.ElementTree as ET
from multiprocessing import Pool
D, PR = sys.argv[1:3]
root = ET.parse(f'{D}/robot.xml').getroot()
files = {m.get('name'): m.get('file') for m in root.iter('mesh')}
neck = next(b for b in root.iter('body') if b.get('name') == 'neck_pitch_assembly')
def f3(s, d): return np.array([float(v) for v in (s or d).split()])
def R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
def axang(ax, a):
    ax = np.asarray(ax, float); ax /= np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
def V(path, scale): return np.unique(np.asarray(trimesh.load(path, force='mesh').vertices) * scale, axis=0)
MESH = {n: V(f'{D}/{fn}', 1.0) for n, fn in files.items()}
PRINT = {'head': V(f'{PR}/head.stl', 1e-3), 'left_eye': V(f'{PR}/left_eye.stl', 1e-3), 'right_eye': V(f'{PR}/right_eye.stl', 1e-3)}
VAR = {'sim head': {}, 'print head': {'head': PRINT['head']}, 'print head + bezels': dict(PRINT)}
def collect(body, Rp, pp, q, var, step, out):
    Rb = Rp @ R(f3(body.get('quat'), '1 0 0 0')); pb = pp + Rp @ f3(body.get('pos'), '0 0 0')
    j = body.find('joint')
    if j is not None: Rb = Rb @ axang(f3(j.get('axis'), '0 0 1'), q.get(j.get('name'), 0.0))
    for g in body.findall('geom'):
        m = g.get('mesh')
        if not m: continue
        Rg = Rb @ R(f3(g.get('quat'), '1 0 0 0')); pg = pb + Rb @ f3(g.get('pos'), '0 0 0')
        names = [m] + (['left_eye', 'right_eye'] if m == 'head' and 'left_eye' in VAR[var] else [])
        for n in names:
            v = VAR[var].get(n, MESH.get(n))
            out.append((n, (v[::step] @ Rg.T + pg) * 1000))
    for c in body.findall('body'): collect(c, Rb, pb, q, var, step, out)
tilt = np.radians(8); axis = np.array([-np.cos(tilt), 0, np.sin(tilt)]); a = np.array([46, -45.0, 115.9])
RC, S0, S1 = 7.2, -1.2, 51.2
J = ['neck_pitch', 'head_pitch', 'head_yaw', 'head_roll']
ANT = [(0, 0), (-90, -90), (-90, 90), (90, -90), (90, 90)]
def clr(var, v, step, ant=(0, 0)):
    q = dict(zip(J, np.radians(v))); q['left_antenna'], q['right_antenna'] = np.radians(ant)
    out = []; collect(neck, np.eye(3), np.zeros(3), q, var, step, out)
    res = {}
    for n, P in out:
        P = P - a; s = np.clip(P @ axis, S0, S1)
        res[n] = min(res.get(n, 1e9), float((np.linalg.norm(P - np.outer(s, axis), axis=1) - RC).min()))
    return res
def job(args):
    var, v = args
    r = clr(var, v, 7); tot = min(r.values())
    bez = min(r.get('left_eye', 1e9), r.get('right_eye', 1e9))
    if tot < 14 or bez < 20:
        rr = [clr(var, v, 1, an) for an in ANT]
        tot = min(min(x.values()) for x in rr); bez = min(min(x.get('left_eye', 1e9), x.get('right_eye', 1e9)) for x in rr)
        who = min(rr[0], key=rr[0].get)
    else:
        who = min(r, key=r.get)
    return var, v, tot, bez, who
ax = [np.arange(-15, 15.1, 2.5), np.arange(-15, 15.1, 2.5), np.arange(-50, 50.1, 5), np.arange(-9, 9.1, 3)]
if __name__ == '__main__':
    nodes = list(itertools.product(*ax))
    print('neutral:', {k: round(min(clr(k, (0, 0, 0, 0), 1).values()), 2) for k in VAR})
    with Pool(8) as p:
        res = p.map(job, [(k, v) for k in VAR for v in nodes], chunksize=100)
    for k in VAR:
        R_ = [r for r in res if r[0] == k]
        t = min(R_, key=lambda r: r[2]); b = min(R_, key=lambda r: r[3])
        print(f'{k:20s} {len(R_)} nodes: min {t[2]:.2f} mm at {t[1]} ({t[4]}); bezel min {b[3] if b[3] < 1e8 else float("nan"):.2f} mm at {b[1]}; nodes<5.73: {sum(r[2] < 5.73 for r in R_)}')
