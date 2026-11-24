"""STL volume audit for printed-part masses. Units: STL files are mm (head.stl spans ~200 mm).
Volume = divergence-theorem signed volume; exact only if the mesh is closed, so we also
check that every edge is shared by exactly two triangles (watertight) and cross-check
with an independent ray-parity voxel estimate."""
import sys, hashlib, re, struct, numpy as np

def load(f):
    b = open(f, 'rb').read()
    n = struct.unpack('<I', b[80:84])[0]
    if len(b) != 84 + 50 * n:
        t = np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', b), float).reshape(-1, 3, 3)
    else:
        a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
        t = a['v'].reshape(-1, 3, 3).astype(float)
    return t, hashlib.sha256(b).hexdigest()

def watertight(t):
    v, inv = np.unique(t.reshape(-1, 3).round(5), axis=0, return_inverse=True)
    f = inv.reshape(-1, 3)
    e = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
    _, c = np.unique(e, axis=0, return_counts=True)
    return int((c != 2).sum())

def parity_volume(t, step=0.5):
    """Cast +z rays on an x/y grid; count crossings to get filled length per ray."""
    lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
    xs = np.arange(lo[0] + step / 2, hi[0], step); ys = np.arange(lo[1] + step / 2, hi[1], step)
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    total = 0.0
    for x in xs:
        for y in ys:
            # barycentric test in xy
            d = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
            ok = np.abs(d) > 1e-12
            l1 = ((b[:, 1] - c[:, 1]) * (x - c[:, 0]) + (c[:, 0] - b[:, 0]) * (y - c[:, 1])) / np.where(ok, d, 1)
            l2 = ((c[:, 1] - a[:, 1]) * (x - c[:, 0]) + (a[:, 0] - c[:, 0]) * (y - c[:, 1])) / np.where(ok, d, 1)
            l3 = 1 - l1 - l2
            hit = ok & (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
            if not hit.any():
                continue
            z = np.sort(l1[hit] * a[hit, 2] + l2[hit] * b[hit, 2] + l3[hit] * c[hit, 2])
            z = z[np.concatenate([[True], np.diff(z) > 1e-6])]  # drop duplicate hits on shared edges
            if len(z) % 2 == 0:
                total += (z[1::2] - z[0::2]).sum()
    return total * step * step

for f in sys.argv[1:]:
    t, h = load(f)
    vol = abs(np.einsum('ij,ij->i', t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6)
    ext = np.ptp(t.reshape(-1, 3), 0)
    print(f"{f.split('/')[-1]}: sha256 {h[:16]} tris {len(t)} bbox {ext.round(1)} mm "
          f"bad_edges {watertight(t)} vol {vol/1000:.3f} cm3 parity {parity_volume(t)/1000:.3f} cm3 "
          f"bbox_vol {np.prod(ext)/1000:.1f} cm3 solid_PLA {vol/1000*1.24:.2f} g")
