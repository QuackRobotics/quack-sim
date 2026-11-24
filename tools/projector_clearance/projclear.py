"""Projector tube clearance vs the moving neck/head chain (upstream robot.xml b23317a).
Forward kinematics over a grid of neck_pitch, head_pitch, head_yaw, head_roll inside their XML ranges;
reports the minimum radial clearance between every moving-geom vertex and a Ø12x50 tube.
Tube: front face centre at (46, y0, z0) mm in trunk_assembly, axis tilted 8 deg down (rear end higher).
Vertices are subsampled (every Nth unique vertex) -> clearance is an estimate (+-~2 mm), not a CAD proof.
Usage: python3 projclear.py <robot_dir> [y0_mm ...]"""
import sys, itertools, numpy as np
import xml.etree.ElementTree as ET
sys.path.insert(0, __file__.rsplit('/', 1)[0])
_argv, sys.argv = sys.argv, sys.argv[:1]  # stlvol runs its CLI loop on import
from stlvol import load
sys.argv = _argv

def R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

def axang(ax, a):
    ax = np.asarray(ax, float); ax /= np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K

def f3(s, d): return np.array([float(v) for v in (s or d).split()])

d = sys.argv[1]
ys = [float(v) for v in sys.argv[2:]] or [0.0, -45.0]
root = ET.parse(d + '/robot.xml').getroot()
neck = [b for b in root.iter('body') if b.get('name') == 'neck_pitch_assembly'][0]
cache = {}
def verts(m, step=7):
    if m not in cache:
        t, _ = load(f'{d}/{m}.stl')
        v = np.unique(t.reshape(-1, 3).round(5), axis=0)
        cache[m] = v[::step]
    return cache[m]

def collect(body, Rp, pp, q, out):
    Rb = Rp @ R(f3(body.get('quat'), '1 0 0 0')); pb = pp + Rp @ f3(body.get('pos'), '0 0 0')
    j = body.find('joint')
    if j is not None:
        Rb = Rb @ axang(f3(j.get('axis'), '0 0 1'), q.get(j.get('name'), 0.0))
    for g in body.findall('geom'):
        if g.get('mesh'):
            v = verts(g.get('mesh'))
            Rg = Rb @ R(f3(g.get('quat'), '1 0 0 0')); pg = pb + Rb @ f3(g.get('pos'), '0 0 0')
            out.append((v @ Rg.T + pg) * 1000)
    for c in body.findall('body'):
        collect(c, Rb, pb, q, out)

joints = {j.get('name'): f3(j.get('range'), '0 0') for j in neck.iter('joint')}
grid = {n: np.linspace(r[0], r[1], 5) for n, r in joints.items() if n.startswith(('neck', 'head'))}
names = list(grid)
tilt = np.radians(8)
axis = np.array([-np.cos(tilt), 0, np.sin(tilt)])  # from front face backwards, rising
z0 = 312 - (304 - 107.9)  # Luxo beam height 312 mm, shell top 304 mm = body top z 107.9 -> 115.9
def configs(only):
    if only == 'all':
        for vals in itertools.product(*[grid[n] for n in names]):
            yield dict(zip(names, vals))
    elif only == 'neutral':
        yield {}
    else:  # one joint swept over 13 points, others at 0
        for v in np.linspace(*joints[only], 13):
            yield {only: v}

modes = ['neutral'] + names + ['all']
for y0, mode in itertools.product(ys, modes):
    a = np.array([46, y0, z0]); best = (1e9, None)
    for q in configs(mode):
        pts = []
        collect(neck, np.eye(3), np.zeros(3), q, pts)
        P = np.vstack(pts) - a
        s = np.clip(P @ axis, 0, 50)
        r = np.linalg.norm(P - np.outer(s, axis), axis=1) - 6.0
        i = r.argmin()
        if r[i] < best[0]:
            best = (r[i], {k: round(float(np.degrees(v))) for k, v in q.items()})
    print(f'tube y0 {y0:+.0f} mm, z0 {z0:.1f}, sweep {mode}: min clearance {best[0]:.1f} mm at {best[1]} (deg)')
print('joint ranges (deg):', {n: np.degrees(grid[n][[0, -1]]).round().tolist() for n in names})
