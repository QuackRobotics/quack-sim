"""FK sign check (Atlas, 2026-11-24): move each head joint +5 deg from XML zero, report how the
front-most head-mesh vertex moves in trunk_assembly mm (+x forward, +z up). Usage: fk_sign.py <robot_dir>"""
import sys, numpy as np
sys.argv = sys.argv[:2]
exec(open('table.py').read().split("\nif __name__ == '__main__':")[0])
def headpts(q):
    out = []
    def rec(b, Rp, pp, inhead):
        Rb = Rp @ R(f3(b.get('quat'), '1 0 0 0')); pb = pp + Rp @ f3(b.get('pos'), '0 0 0')
        j = b.find('joint')
        if j is not None: Rb = Rb @ axang(f3(j.get('axis'), '0 0 1'), q.get(j.get('name'), 0.0))
        inhead = inhead or b.get('name') == 'head_assembly'
        if inhead:
            for g in b.findall('geom'):
                if g.get('mesh') == 'head':
                    v = _v('head', 1); Rg = Rb @ R(f3(g.get('quat'), '1 0 0 0')); pg = pb + Rb @ f3(g.get('pos'), '0 0 0')
                    out.append((v @ Rg.T + pg) * 1000)
        for c in b.findall('body'): rec(c, Rb, pb, inhead)
    rec(neck, np.eye(3), np.zeros(3), False); return np.vstack(out)
P0 = headpts({}); i = P0[:, 0].argmax(); print('front-most head vertex at zero:', P0[i].round(1))
for jn in ['neck_pitch', 'head_pitch', 'head_yaw', 'head_roll']:
    P = headpts({jn: np.radians(5)}); dv = P[i] - P0[i]; print(f'{jn} +5 deg: front vertex dx {dv[0]:+.2f} dy {dv[1]:+.2f} dz {dv[2]:+.2f} mm')
