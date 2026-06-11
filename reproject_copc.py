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


MAX_ALIGN_RESIDUAL = 2.0   # metres — abort if the frames don't truly match


def auto_align(capture_dir, tiles_dir):
    """Find the lambert→blend transform by DEM cross-correlation.

    gs_capture.py exports cloud_dem.npy — a max-Z grid of the cloud actually
    loaded in the .blend, in the same world frame as the render cameras.
    We build the same grid from the raw tiles and slide one over the other.
    Returns `blend_offset` such that  blend = lambert - blend_offset.
    Aborts loudly if the best fit is worse than MAX_ALIGN_RESIDUAL — a silent
    misalignment paints the light onto the wrong terrain.
    """
    dem_path = os.path.join(capture_dir, 'cloud_dem.npy')
    meta_path = os.path.join(capture_dir, 'cloud_dem_meta.json')
    if not (os.path.exists(dem_path) and os.path.exists(meta_path)):
        sys.exit(f"ERROR: {dem_path} missing.\n"
                 "Re-run gs_capture.py (it now exports the cloud DEM), or create\n"
                 "cloud_dem.npy from the open .blend — see gs_capture.py source.")
    bdem = np.load(dem_path)
    with open(meta_path) as f:
        meta = json.load(f)
    bmn = np.array(meta['min_xy'], dtype=np.float64)
    cell = float(meta['cell'])
    bW, bH = bdem.shape
    bval = bdem > -1e8

    # raw-tile DEM at the same cell size (stride-read for speed)
    import laspy as _laspy
    tiles = sorted(f for f in os.listdir(tiles_dir)
                   if f.lower().endswith(('.laz', '.las')))
    gx0 = gy0 = np.inf
    gx1 = gy1 = -np.inf
    for t in tiles:
        h = _laspy.open(os.path.join(tiles_dir, t)).header
        gx0, gx1 = min(gx0, h.x_min), max(gx1, h.x_max)
        gy0, gy1 = min(gy0, h.y_min), max(gy1, h.y_max)
    W = int((gx1 - gx0) / cell) + 1
    H = int((gy1 - gy0) / cell) + 1
    rdem = np.full(W * H, -1e9, dtype=np.float32)
    for t in tiles:
        las = _laspy.read(os.path.join(tiles_dir, t))
        xy = np.stack([np.asarray(las.x), np.asarray(las.y)], axis=1)[::25]
        z = np.asarray(las.z)[::25]
        ij = ((xy - [gx0, gy0]) / cell).astype(np.int32)
        np.maximum.at(rdem, ij[:, 0] * H + ij[:, 1], z.astype(np.float32))
        del las
    rdem = rdem.reshape(W, H)
    log(f"alignment DEMs: blend {bW}x{bH}, tiles {W}x{H} @ {cell}m")

    best = None
    for dx in range(0, W - bW + 1):
        for dy in range(0, H - bH + 1):
            sub = rdem[dx:dx+bW, dy:dy+bH]
            m = bval & (sub > -1e8)
            if m.sum() < 5000:
                continue
            dz = sub[m] - bdem[m]
            med = np.median(dz)
            err = np.mean(np.abs(dz - med))
            if best is None or err < best[0]:
                best = (err, dx, dy, med)
    if best is None:
        sys.exit("ERROR: no DEM overlap found between blend cloud and tiles.")
    err, dx, dy, dz = best
    tx = (gx0 + dx * cell) - bmn[0]
    ty = (gy0 + dy * cell) - bmn[1]
    log(f"auto-align: blend + ({tx:.1f}, {ty:.1f}, {dz:.1f}) = lambert   "
        f"residual {err:.2f} m")
    if err > MAX_ALIGN_RESIDUAL:
        sys.exit(f"ERROR: alignment residual {err:.2f} m > {MAX_ALIGN_RESIDUAL} m.\n"
                 "The cloud in the .blend does not match these tiles (wrong scene,\n"
                 "wrong tiles_dir, or the cloud was scaled/rotated). Refusing to\n"
                 "produce a misaligned reprojection.")
    # lambert-frame footprint of the cloud actually loaded in the .blend —
    # used to select which tiles to process (the .blend may use only a
    # subset of what sits in the tiles folder)
    extent = (bmn[0] + tx, bmn[1] + ty,
              bmn[0] + bW * cell + tx, bmn[1] + bH * cell + ty)
    return np.array([tx, ty, dz], dtype=np.float64), extent


UNSEEN_DIM = 0.15


class OrthoSampler:
    """Nearest-pixel RGB from a GeoTIFF, georef read from the TIFF tags
    (ModelPixelScale 33550 + ModelTiepoint 33922) — no GDAL dependency."""

    def __init__(self, path):
        img = Image.open(path)
        scale = img.tag_v2[33550]      # (sx, sy, sz)
        tie = img.tag_v2[33922]        # (i, j, k, X, Y, Z) — pixel 0,0 → X,Y
        self.px, self.py = float(scale[0]), float(scale[1])
        self.x0, self.y0 = float(tie[3]), float(tie[4])
        self.img = np.asarray(img.convert('RGB'))
        log(f"ortho {self.img.shape[1]}x{self.img.shape[0]} @ {self.px:g} m/px "
            f"origin ({self.x0:.0f}, {self.y0:.0f})")

    def sample(self, xy_lambert):
        H, W = self.img.shape[:2]
        col = np.clip(((xy_lambert[:, 0] - self.x0) / self.px).astype(np.int32), 0, W-1)
        row = np.clip(((self.y0 - xy_lambert[:, 1]) / self.py).astype(np.int32), 0, H-1)
        return self.img[row, col]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('reference_ply')
    ap.add_argument('capture_dir')
    ap.add_argument('tiles_dir')
    ap.add_argument('out_dir')
    ap.add_argument('--origin', default=None,
                    help='Override X,Y,Z lambert→blend offset (skips auto-align)')
    ap.add_argument('--raster', default=None,
                    help='BDORTHO GeoTIFF — unseen points get dimmed satellite '
                         'albedo (x0.15) instead of black')
    args = ap.parse_args()

    blend_extent = None
    if args.origin:
        origin = np.array([float(v) for v in args.origin.split(',')], dtype=np.float64)
        log(f"manual origin {origin}")
    else:
        origin, blend_extent = auto_align(args.capture_dir, args.tiles_dir)
    os.makedirs(args.out_dir, exist_ok=True)

    # ── Tile selection — only tiles the .blend cloud actually covers ────────
    # The tiles folder may hold more tiles than the scene uses. Tiles outside
    # the blend footprint would sample black background pixels from the
    # renders → dark frame around the scene. Require > 50 m overlap.
    MARGIN = 50.0
    selected = []
    for t in sorted(os.listdir(args.tiles_dir)):
        if not t.lower().endswith(('.laz', '.las')):
            continue
        if blend_extent is not None:
            h = laspy.open(os.path.join(args.tiles_dir, t)).header
            x0, y0, x1, y1 = blend_extent
            ov_x = min(h.x_max, x1) - max(h.x_min, x0)
            ov_y = min(h.y_max, y1) - max(h.y_min, y0)
            if ov_x < MARGIN or ov_y < MARGIN:
                log(f"skip {t} — outside the blend cloud footprint")
                continue
        selected.append(t)
    if not selected:
        sys.exit("ERROR: no tiles overlap the blend cloud footprint.")
    log(f"{len(selected)} tiles selected")
    ortho = OrthoSampler(args.raster) if args.raster else None
    if ortho is None:
        print("WARNING: no --raster — unseen points will be black. "
              "Pass the BDORTHO GeoTIFF for a dimmed-albedo fallback.")

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

    # Resume fast-path: if every selected tile already has its output, skip
    # Phase A entirely (z-buffers are only needed to color tiles).
    def _out_for(t):
        return os.path.join(args.out_dir,
                            os.path.basename(t).replace('.copc.laz', '')
                            .replace('.laz', '').replace('.las', '') + '_lit.laz')
    # Stale outputs from tiles no longer selected would get merged into the
    # final COPC — remove them.
    sel_outs = {os.path.basename(_out_for(t)) for t in selected}
    for f in os.listdir(args.out_dir):
        if f.endswith('_lit.laz') and f not in sel_outs:
            os.remove(os.path.join(args.out_dir, f))
            log(f"removed stale {f} (tile not in blend footprint)")
    if all(os.path.exists(_out_for(t)) for t in selected):
        print(f"All {len(selected)} selected tiles already done — nothing to reproject.")
        return

    # ── Phase A: z-buffers ──────────────────────────────────────────────────
    # Built from the raw tiles themselves (strided), shifted into the blend
    # frame with the SAME offset as Phase B — one frame, no reference-PLY
    # frame assumptions. A 1/8 stride still gives ~50M+ points, far denser
    # than the z-buffer grid needs.
    if args.reference_ply.lower() == 'tiles':
        print("Phase A — z-buffers from strided raw tiles")
        ZSTRIDE = 8
        parts = []
        for t in selected:
            las = laspy.read(os.path.join(args.tiles_dir, t))
            parts.append((np.stack([np.asarray(las.x), np.asarray(las.y),
                                    np.asarray(las.z)], axis=1)[::ZSTRIDE]
                          - origin).astype(np.float64))
            del las
        ref = np.concatenate(parts)
        del parts
    else:
        print(f"Phase A — z-buffers from {args.reference_ply}")
        print("WARNING: the PLY must be in the BLEND world frame (e.g. exported "
              "from the open .blend). Pass 'tiles' to build from raw tiles instead.")
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

    # ── Phase B: color each selected IGN tile ───────────────────────────────
    tiles = selected
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
        if ortho is not None and (~seen).any():
            alb = ortho.sample(xyz[~seen][:, :2] + origin[:2])   # back to lambert
            rgb[~seen] = (alb.astype(np.float32) * UNSEEN_DIM).astype(np.uint16) * 257
        log(f"coverage {seen.sum()/n*100:.2f}%"
            + ("" if ortho is None else f"  (unseen → dimmed albedo)"))
        grand_total += n
        grand_seen += int(seen.sum())

        # write LAZ (point format 7 = xyz + rgb), keep Lambert-93 coords
        header = laspy.LasHeader(version="1.4", point_format=7)
        header.offsets = las.header.offsets
        header.scales = las.header.scales
        out = laspy.LasData(header)
        out.x, out.y, out.z = las.x, las.y, las.z
        out.red, out.green, out.blue = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        # write to a temp name then rename — a killed run never leaves a
        # corrupt half-written tile that the resume skip would silently
        # accept (keep .laz extension: laspy infers compression from it)
        tmp_path = out_path[:-4] + '_tmp.laz'
        out.write(tmp_path)
        os.replace(tmp_path, out_path)
        log(f"wrote {out_path}")
        del las, xyz, col_sum, cnt, out

    print(f"\nTotal coverage: {grand_seen/max(grand_total,1)*100:.2f}% "
          f"of {grand_total:,} points")
    print("Next: pdal merge the *_lit.laz files into a single COPC.")


if __name__ == '__main__':
    main()
