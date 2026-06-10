"""
reproject_copc.py — color raw IGN COPC tiles from Cycles orbit renders.

Full-quality path: the display cloud is the original full-density IGN HD
tiles (never voxel-downsampled), and their colors come straight from the
4K Cycles renders produced by gs_capture.py. No lighting physics, no
per-scene constants — the renders ARE the look.

Occlusion is resolved with per-camera z-buffers built once from the HD
reference PLY (the full-extent cloud loaded in Blender), so a ridge in one
tile correctly shadows points in another tile.

Pipeline:
  Phase A  build z-buffer per camera from <reference.ply>
  Phase B  per IGN tile: project points, z-test, sample render, write LAZ
  Phase C  (separate) pdal merge → COPC

Usage:
  python reproject_copc.py <reference.ply> <capture_dir> <tiles_dir> <out_dir>
      --origin X,Y,Z

  --origin: the Lambert-93 offset used by lidar_pipeline.py (blender = lambert - origin)
"""

import sys
import os
import json
import time
import argparse
import numpy as np
import laspy
from PIL import Image

DEPTH_TOL_ABS = 2.0
DEPTH_TOL_REL = 0.005
ZBUF_DOWNSCALE = 2
CHUNK = 8_000_000

T0 = time.time()
def log(msg):
    print(f"  [{time.time()-T0:7.1f}s] {msg}", flush=True)


def read_ply_xyz(path):
    with open(path, 'rb') as f:
        n = None
        while True:
            line = f.readline().decode('ascii', 'replace').strip()
            if line.startswith('element vertex'):
                n = int(line.split()[-1])
            elif line == 'end_header':
                break
        dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                       ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
        data = np.fromfile(f, dtype=dt, count=n)
    return np.stack([data['x'], data['y'], data['z']], axis=1)


def project(xyz, R, C, fl, cx, cy, W, H):
    """Return (u, v, depth, in_bounds_mask) for Blender-convention camera."""
    pc = (xyz - C) @ R
    depth = -pc[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = fl * pc[:, 0] / depth + cx
        v = cy - fl * pc[:, 1] / depth
    inb = (depth > 1.0) & (u >= 0) & (u < W - 1) & (v >= 0) & (v < H - 1)
    return u, v, depth, inb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('reference_ply')
    ap.add_argument('capture_dir')
    ap.add_argument('tiles_dir')
    ap.add_argument('out_dir')
    ap.add_argument('--origin', required=True,
                    help='X,Y,Z Lambert-93 origin used when building the Blender cloud')
    args = ap.parse_args()

    origin = np.array([float(v) for v in args.origin.split(',')], dtype=np.float64)
    os.makedirs(args.out_dir, exist_ok=True)

    with open(os.path.join(args.capture_dir, 'transforms.json')) as f:
        tr = json.load(f)
    fl, cx, cy = tr['fl_x'], tr['cx'], tr['cy']
    W, H = tr['w'], tr['h']
    frames = [fr for fr in tr['frames']
              if os.path.exists(os.path.join(args.capture_dir, fr['file_path']))]
    log(f"{len(frames)} cameras  {W}x{H}")

    cams = []
    for fr in frames:
        c2w = np.array(fr['transform_matrix'], dtype=np.float64)
        cams.append((c2w[:3, :3], c2w[:3, 3],
                     os.path.join(args.capture_dir, fr['file_path'])))

    bw, bh = W // ZBUF_DOWNSCALE, H // ZBUF_DOWNSCALE

    # ── Phase A: z-buffers from the reference cloud ─────────────────────────
    print(f"Phase A — z-buffers from {args.reference_ply}")
    ref = read_ply_xyz(args.reference_ply)
    log(f"{len(ref):,} reference points")
    zbufs = np.full((len(cams), bh * bw), np.inf, dtype=np.float32)
    for ci, (R, C, _) in enumerate(cams):
        for s in range(0, len(ref), CHUNK):
            u, v, d, inb = project(ref[s:s+CHUNK], R, C, fl, cx, cy, W, H)
            idx = np.nonzero(inb)[0]
            if len(idx) == 0:
                continue
            cell = ((v[idx] / ZBUF_DOWNSCALE).astype(np.int32) * bw
                    + (u[idx] / ZBUF_DOWNSCALE).astype(np.int32))
            dd = d[idx].astype(np.float32)
            order = np.argsort(-dd)
            # near overwrites far, then merge with existing buffer
            tmp = np.full(bh * bw, np.inf, dtype=np.float32)
            tmp[cell[order]] = dd[order]
            np.minimum(zbufs[ci], tmp, out=zbufs[ci])
        if (ci + 1) % 20 == 0 or ci == len(cams) - 1:
            log(f"z-buffers {ci+1}/{len(cams)}")
    del ref

    # ── Phase B: color each IGN tile ────────────────────────────────────────
    tiles = sorted(f for f in os.listdir(args.tiles_dir)
                   if f.lower().endswith(('.laz', '.las')))
    print(f"\nPhase B — {len(tiles)} tiles")
    grand_total = grand_seen = 0

    for ti, tile in enumerate(tiles):
        tpath = os.path.join(args.tiles_dir, tile)
        out_path = os.path.join(args.out_dir,
                                os.path.basename(tile).replace('.copc.laz', '')
                                .replace('.laz', '').replace('.las', '') + '_lit.laz')
        if os.path.exists(out_path):
            log(f"skip {tile} (output exists)")
            continue
        print(f"\n[{ti+1}/{len(tiles)}] {tile}")
        las = laspy.read(tpath)
        xyz = np.stack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)],
                       axis=1) - origin          # → Blender frame
        n = len(xyz)
        log(f"{n:,} points")

        col_sum = np.zeros((n, 3), dtype=np.float32)
        cnt = np.zeros(n, dtype=np.uint8)

        for ci, (R, C, img_path) in enumerate(cams):
            img = None
            for s in range(0, n, CHUNK):
                u, v, d, inb = project(xyz[s:s+CHUNK], R, C, fl, cx, cy, W, H)
                idx = np.nonzero(inb)[0]
                if len(idx) == 0:
                    continue
                cell = ((v[idx] / ZBUF_DOWNSCALE).astype(np.int32) * bw
                        + (u[idx] / ZBUF_DOWNSCALE).astype(np.int32))
                vis = d[idx] <= zbufs[ci][cell] + (DEPTH_TOL_ABS + DEPTH_TOL_REL * d[idx])
                pidx = idx[vis] + s
                if len(pidx) == 0:
                    continue
                if img is None:
                    img = np.asarray(Image.open(img_path).convert('RGB'))
                pu = np.round(u[pidx - s]).astype(np.int32)
                pv = np.round(v[pidx - s]).astype(np.int32)
                np.clip(pu, 0, W - 1, out=pu)
                np.clip(pv, 0, H - 1, out=pv)
                np.add.at(col_sum, pidx, img[pv, pu].astype(np.float32))
                # saturate-add on uint8 counts
                c = cnt[pidx]
                cnt[pidx] = np.minimum(c.astype(np.int32) + 1, 255).astype(np.uint8)
            if (ci + 1) % 30 == 0 or ci == len(cams) - 1:
                log(f"  cam {ci+1}/{len(cams)}  coverage {(cnt>0).sum()/n*100:5.1f}%")

        seen = cnt > 0
        rgb = np.zeros((n, 3), dtype=np.uint16)
        rgb[seen] = (col_sum[seen] / cnt[seen, None]).astype(np.uint16) * 257  # 8→16 bit
        log(f"coverage {seen.sum()/n*100:.2f}%")
        grand_total += n
        grand_seen += int(seen.sum())

        # write LAZ (point format 7 = xyz + rgb), keep Lambert-93 coords
        header = laspy.LasHeader(version="1.4", point_format=7)
        header.offsets = las.header.offsets
        header.scales = las.header.scales
        out = laspy.LasData(header)
        out.x, out.y, out.z = las.x, las.y, las.z
        out.red, out.green, out.blue = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        out.write(out_path)
        log(f"wrote {out_path}")
        del las, xyz, col_sum, cnt, out

    print(f"\nTotal coverage: {grand_seen/max(grand_total,1)*100:.2f}% "
          f"of {grand_total:,} points")
    print("Next: pdal merge the *_lit.laz files into a single COPC.")


if __name__ == '__main__':
    main()
