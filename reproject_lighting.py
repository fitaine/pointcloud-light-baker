"""
reproject_lighting.py — color a point cloud from real Cycles renders.

Instead of re-implementing Blender's lighting in Python (bake_lighting.py),
this samples the actual rendered pixels: every point is projected into the
orbit renders produced by gs-capture, visibility is resolved with a point
z-buffer per camera, and the visible samples are averaged. The result is
universal by construction — any light type, material, or color management
Blender can render ends up in the points with zero per-scene constants.

Usage:
  python reproject_lighting.py <input.ply> <capture_dir> <output.ply>

  <capture_dir> must contain transforms.json + images/ from gs_capture.py.
  The PLY must be in the same world frame as the Blender scene (e.g. a
  stride export from the loaded cloud).
"""

import sys
import json
import time
import os
import numpy as np
from PIL import Image

# Visibility tolerance: a point passes the z-test if it is within this band
# behind the nearest point in its z-buffer cell. Accounts for ball radius
# (points render as ~0.5 m spheres) and buffer quantization.
DEPTH_TOL_ABS = 2.0     # metres
DEPTH_TOL_REL = 0.005   # fraction of depth
ZBUF_DOWNSCALE = 2      # z-buffer at 1/2 render resolution (denser coverage)
GRAZE_MIN = 0.0         # no normal info needed — z-buffer handles occlusion

T0 = time.time()
def log(msg):
    print(f"  [{time.time()-T0:6.1f}s] {msg}", flush=True)


def read_ply(path):
    with open(path, 'rb') as f:
        n = None
        props = []
        while True:
            line = f.readline().decode('ascii', 'replace').strip()
            if line.startswith('element vertex'):
                n = int(line.split()[-1])
            elif line.startswith('property'):
                props.append(line.split()[-1])
            elif line == 'end_header':
                break
        dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                       ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
        data = np.fromfile(f, dtype=dt, count=n)
    xyz = np.stack([data['x'], data['y'], data['z']], axis=1)
    rgb = np.stack([data['r'], data['g'], data['b']], axis=1)
    return xyz, rgb


def write_ply(path, xyz, rgb):
    n = len(xyz)
    header = (f"ply\nformat binary_little_endian 1.0\nelement vertex {n}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              "end_header\n")
    dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                   ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
    out = np.empty(n, dtype=dt)
    out['x'], out['y'], out['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    out['r'], out['g'], out['b'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        out.tofile(f)


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(1)
    ply_path, capture_dir, out_path = sys.argv[1:4]

    print(f"Reading  {ply_path} …")
    xyz, rgb_orig = read_ply(ply_path)
    n = len(xyz)
    log(f"{n:,} points")

    with open(os.path.join(capture_dir, 'transforms.json')) as f:
        tr = json.load(f)
    fl = tr['fl_x']
    cx, cy = tr['cx'], tr['cy']
    W, H = tr['w'], tr['h']
    frames = tr['frames']
    log(f"{len(frames)} cameras  {W}x{H}  fl={fl:.1f}")

    # z-buffer grid (downscaled)
    bw, bh = W // ZBUF_DOWNSCALE, H // ZBUF_DOWNSCALE

    col_sum = np.zeros((n, 3), dtype=np.float64)
    cnt = np.zeros(n, dtype=np.uint16)

    for fi, fr in enumerate(frames):
        img_path = os.path.join(capture_dir, fr['file_path'])
        if not os.path.exists(img_path):
            log(f"skip {fr['file_path']} (missing)")
            continue
        c2w = np.array(fr['transform_matrix'], dtype=np.float64)
        R = c2w[:3, :3]
        C = c2w[:3, 3]

        # world → camera (Blender convention: camera looks along -Z, Y up)
        pc = (xyz - C) @ R           # R is orthonormal: inv = transpose, (p-C)@R == R.T@(p-C)
        depth = -pc[:, 2]
        front = depth > 1.0
        u = fl * pc[:, 0] / depth + cx
        v = cy - fl * pc[:, 1] / depth
        inb = front & (u >= 0) & (u < W - 1) & (v >= 0) & (v < H - 1)
        idx = np.nonzero(inb)[0]
        if len(idx) == 0:
            continue
        ud, vd = u[idx], v[idx]
        d = depth[idx]

        # point z-buffer: nearest depth per cell (sorted scatter — last write wins)
        bu = (ud / ZBUF_DOWNSCALE).astype(np.int32)
        bv = (vd / ZBUF_DOWNSCALE).astype(np.int32)
        cell = bv * bw + bu
        zbuf = np.full(bw * bh, np.inf, dtype=np.float32)
        order = np.argsort(-d)               # far first, near overwrites
        zbuf[cell[order]] = d[order]

        vis = d <= zbuf[cell] + (DEPTH_TOL_ABS + DEPTH_TOL_REL * d)
        pidx = idx[vis]
        if len(pidx) == 0:
            continue

        img = np.asarray(Image.open(img_path).convert('RGB'), dtype=np.float64)
        pu = np.clip(np.round(ud[vis]).astype(np.int32), 0, W - 1)
        pv = np.clip(np.round(vd[vis]).astype(np.int32), 0, H - 1)
        np.add.at(col_sum, pidx, img[pv, pu])
        np.add.at(cnt, pidx, 1)

        if (fi + 1) % 10 == 0 or fi == len(frames) - 1:
            seen = int((cnt > 0).sum())
            log(f"{fi+1:3d}/{len(frames)}  coverage {seen/n*100:5.1f}%")

    seen = cnt > 0
    log(f"final coverage: {seen.sum()/n*100:.2f}%  ({(~seen).sum():,} unseen)")

    rgb_out = np.empty((n, 3), dtype=np.uint8)
    rgb_out[seen] = np.clip(col_sum[seen] / cnt[seen, None], 0, 255).astype(np.uint8)
    # unseen points: heavily dimmed original color so they recede into darkness
    rgb_out[~seen] = (rgb_orig[~seen].astype(np.float64) * 0.15).astype(np.uint8)

    print(f"Writing  {out_path} …")
    write_ply(out_path, xyz, rgb_out)
    log("done")


if __name__ == '__main__':
    main()
