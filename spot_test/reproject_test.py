"""
reproject_test.py — full-resolution diagnostic on the spotlight region.

Produces three clouds so each ingredient can be inspected ALONE in Potree:
  spot_geom.laz  — geometry only, coloured by elevation
  spot_sat.laz   — satellite albedo only (sampled from the BDORTHO raster)
  spot_lit.laz   — full reproject: seen points = averaged Cycles render pixels,
                   UNSEEN points = dimmed albedo (x0.15)  ← the fix that the
                   low-res path had and the full-res COPC path was missing.

Z-buffers are built once from the full reference cloud and cached to zbufs.npy
so re-runs are fast.

Usage:
  python reproject_test.py
"""
import os, json, time, sys
import numpy as np
import laspy
from PIL import Image

Image.MAX_IMAGE_PIXELS = None

HERE      = os.path.dirname(os.path.abspath(__file__))
ROOT      = os.path.dirname(HERE)
SCENE     = "C:/Users/Tiphaine/Pictures/3D/2026-03-09_Chamechaude"
CROP_LAZ  = os.path.join(HERE, "spot_crop.laz")
REF_PLY   = SCENE + "/LIDAR/output/chamechaude-hd-055.ply"
CAP_DIR   = ROOT + "/gs-capture/output/2026-03-09_Chamechaude"
RASTER    = SCENE + "/LIDAR/output/chamechaude-hd-039_raster.tif"
# Frame mapping measured by DEM cross-correlation (residual 0.87 m):
#   lambert = blend + BLEND_TO_LAMBERT
# The .blend cloud (chamechaude-hd-029) is NOT in the lambert-origin frame —
# using the lidar_pipeline origin (917000,6468000,1253.74) was the root cause
# of the misaligned full-res reprojection.
BLEND_TO_LAMBERT = np.array([917865.5, 6468951.9, 1536.4])
# hd-055 reference PLY is in lambert-(917000,6468000,1253.74); shift to blend:
REF_TO_BLEND = np.array([917000.0, 6468000.0, 1253.74]) - BLEND_TO_LAMBERT
ZBUF_CACHE= os.path.join(HERE, "zbufs_v2.npy")

# raster geo-reference (from gdalinfo): UL=(916000,6470000), 1 m/px
RAS_X0, RAS_Y0, RAS_PX = 916000.0, 6470000.0, 1.0

DEPTH_TOL_ABS = 2.0
DEPTH_TOL_REL = 0.005
ZBUF_DOWNSCALE = 2
CHUNK = 8_000_000
UNSEEN_DIM = 0.15

T0 = time.time()
def log(m): print(f"  [{time.time()-T0:7.1f}s] {m}", flush=True)


def read_ply_xyz(path):
    with open(path, 'rb') as f:
        n = None
        while True:
            line = f.readline().decode('ascii', 'replace').strip()
            if line.startswith('element vertex'): n = int(line.split()[-1])
            elif line == 'end_header': break
        dt = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('r','u1'),('g','u1'),('b','u1')])
        data = np.fromfile(f, dtype=dt, count=n)
    return np.stack([data['x'], data['y'], data['z']], axis=1)


def project(xyz, R, C, fl, cx, cy, W, H):
    pc = (xyz - C) @ R
    depth = -pc[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = fl * pc[:, 0] / depth + cx
        v = cy - fl * pc[:, 1] / depth
    inb = (depth > 1.0) & (u >= 0) & (u < W-1) & (v >= 0) & (v < H-1)
    return u, v, depth, inb


def sample_albedo(xyz_lambert):
    """Nearest-pixel RGB from the ortho raster. Returns uint8 (n,3)."""
    img = np.asarray(Image.open(RASTER).convert('RGB'))   # (H,W,3)
    Hh, Ww = img.shape[:2]
    col = np.floor((xyz_lambert[:, 0] - RAS_X0) / RAS_PX).astype(np.int32)
    row = np.floor((RAS_Y0 - xyz_lambert[:, 1]) / RAS_PX).astype(np.int32)
    np.clip(col, 0, Ww-1, out=col)
    np.clip(row, 0, Hh-1, out=row)
    return img[row, col]


def write_laz(path, ref_las, rgb_u8):
    """Write point-format-7 LAZ, colours scaled 8→16 bit for Potree."""
    header = laspy.LasHeader(version="1.4", point_format=7)
    header.offsets = ref_las.header.offsets
    header.scales  = ref_las.header.scales
    out = laspy.LasData(header)
    out.x, out.y, out.z = ref_las.x, ref_las.y, ref_las.z
    rgb16 = rgb_u8.astype(np.uint16) * 257
    out.red, out.green, out.blue = rgb16[:,0], rgb16[:,1], rgb16[:,2]
    out.write(path)
    log(f"wrote {os.path.basename(path)}")


def main():
    # ── load the crop ───────────────────────────────────────────────────────
    las = laspy.read(CROP_LAZ)
    xyz_l = np.stack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)], axis=1)
    xyz = xyz_l - BLEND_TO_LAMBERT          # lambert → blend (camera frame)
    n = len(xyz)
    log(f"{n:,} crop points")

    # ── cameras ─────────────────────────────────────────────────────────────
    with open(os.path.join(CAP_DIR, 'transforms.json')) as f:
        tr = json.load(f)
    fl, cx, cy, W, H = tr['fl_x'], tr['cx'], tr['cy'], tr['w'], tr['h']
    frames = [fr for fr in tr['frames']
              if os.path.exists(os.path.join(CAP_DIR, fr['file_path']))]
    cams = [(np.array(fr['transform_matrix'],dtype=np.float64)[:3,:3],
             np.array(fr['transform_matrix'],dtype=np.float64)[:3,3],
             os.path.join(CAP_DIR, fr['file_path'])) for fr in frames]
    bw, bh = W//ZBUF_DOWNSCALE, H//ZBUF_DOWNSCALE
    log(f"{len(cams)} cameras {W}x{H}")

    # ── ingredient 1: geometry (elevation colour) ───────────────────────────
    z = xyz_l[:, 2]
    t = (z - z.min()) / max(z.max()-z.min(), 1e-6)
    # simple blue→white elevation ramp
    geom = np.stack([ (0.2+0.8*t), (0.3+0.7*t), (0.5+0.5*t) ], axis=1)
    geom = (np.clip(geom,0,1)*255).astype(np.uint8)
    write_laz(os.path.join(HERE,'spot_geom.laz'), las, geom)

    # ── ingredient 2: satellite albedo ──────────────────────────────────────
    albedo = sample_albedo(xyz_l)
    write_laz(os.path.join(HERE,'spot_sat.laz'), las, albedo)
    log("albedo sampled")

    # ── z-buffers from reference cloud (cached) ─────────────────────────────
    if os.path.exists(ZBUF_CACHE):
        zbufs = np.load(ZBUF_CACHE)
        log(f"loaded cached z-buffers {zbufs.shape}")
    else:
        ref = read_ply_xyz(REF_PLY) + REF_TO_BLEND   # → blend (camera frame)
        log(f"{len(ref):,} reference points — building z-buffers")
        zbufs = np.full((len(cams), bh*bw), np.inf, dtype=np.float32)
        for ci,(R,C,_) in enumerate(cams):
            for s in range(0, len(ref), CHUNK):
                u,v,d,inb = project(ref[s:s+CHUNK], R,C,fl,cx,cy,W,H)
                idx = np.nonzero(inb)[0]
                if not len(idx): continue
                cell = ((v[idx]/ZBUF_DOWNSCALE).astype(np.int32)*bw
                        + (u[idx]/ZBUF_DOWNSCALE).astype(np.int32))
                dd = d[idx].astype(np.float32)
                order = np.argsort(-dd)
                tmp = np.full(bh*bw, np.inf, dtype=np.float32)
                tmp[cell[order]] = dd[order]
                np.minimum(zbufs[ci], tmp, out=zbufs[ci])
            if (ci+1)%20==0 or ci==len(cams)-1: log(f"  zbuf {ci+1}/{len(cams)}")
        del ref
        np.save(ZBUF_CACHE, zbufs)
        log("z-buffers cached")

    # ── ingredient 3: reproject light, satellite fallback ───────────────────
    col_sum = np.zeros((n,3), dtype=np.float32)
    cnt = np.zeros(n, dtype=np.uint16)
    for ci,(R,C,img_path) in enumerate(cams):
        img = None
        for s in range(0, n, CHUNK):
            u,v,d,inb = project(xyz[s:s+CHUNK], R,C,fl,cx,cy,W,H)
            idx = np.nonzero(inb)[0]
            if not len(idx): continue
            cell = ((v[idx]/ZBUF_DOWNSCALE).astype(np.int32)*bw
                    + (u[idx]/ZBUF_DOWNSCALE).astype(np.int32))
            vis = d[idx] <= zbufs[ci][cell] + (DEPTH_TOL_ABS + DEPTH_TOL_REL*d[idx])
            pidx = idx[vis] + s
            if not len(pidx): continue
            if img is None:
                img = np.asarray(Image.open(img_path).convert('RGB'))
            pu = np.clip(np.round(u[pidx-s]).astype(np.int32),0,W-1)
            pv = np.clip(np.round(v[pidx-s]).astype(np.int32),0,H-1)
            np.add.at(col_sum, pidx, img[pv,pu].astype(np.float32))
            cnt[pidx] += 1
        if (ci+1)%30==0 or ci==len(cams)-1:
            log(f"  cam {ci+1}/{len(cams)} coverage {(cnt>0).sum()/n*100:5.1f}%")

    seen = cnt > 0
    lit = np.empty((n,3), dtype=np.uint8)
    lit[seen] = np.clip(col_sum[seen]/cnt[seen,None], 0, 255).astype(np.uint8)
    lit[~seen] = (albedo[~seen].astype(np.float32) * UNSEEN_DIM).astype(np.uint8)
    log(f"coverage {seen.sum()/n*100:.2f}%  ({(~seen).sum():,} unseen → dimmed albedo)")
    write_laz(os.path.join(HERE,'spot_lit.laz'), las, lit)
    print("\nDone. Three clouds written: spot_geom / spot_sat / spot_lit (.laz)")


if __name__ == '__main__':
    main()
