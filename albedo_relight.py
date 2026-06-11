"""
albedo_relight.py — albedo x light separation (post-process, no re-render).

The reprojected cloud's texture detail is capped by the render resolution
(~1 m/px) and softened by multi-view averaging. The ortho has sharper
texture. Split the roles:

    light  = render_color / blur(albedo)     (low-frequency artistic lighting)
    final  = albedo x light                  (high-frequency satellite texture)

Per lit tile: sample the ortho at each point (sharp albedo), divide the
reprojected color by a BLURRED albedo (matching the render's effective
resolution) to extract a lighting ratio, multiply back. Done in linear
space. Where blurred albedo is near zero the ratio is clamped.

The blur radius — "how smeared is the render compared to the ortho" — is
computed from the capture itself when --capture is given:

    ground_px = orbit_radius / fl_x          (render ground resolution, m/px)
    blur      = 2.0 x ground_px              (x2: multi-view averaging spread)

Usage:
  python albedo_relight.py <lit_tiles_dir> <raster.tif> <out_dir>
      [--capture <capture_dir>]   auto-compute blur from transforms.json
      [--blur 2.0]                manual override in metres
"""

import argparse
import json
import os
import sys
import time
import numpy as np
import laspy
from PIL import Image
from scipy.ndimage import gaussian_filter

Image.MAX_IMAGE_PIXELS = None   # native 0.20m orthos exceed PIL's bomb guard


def auto_blur(capture_dir):
    """Render ground resolution from the capture's own cameras.

    Camera positions lie on a sphere around the orbit target — fit the
    sphere (linear least squares) to get the radius without needing the
    target. blur = 2 x radius / fl_x.
    """
    with open(os.path.join(capture_dir, 'transforms.json')) as f:
        tr = json.load(f)
    P = np.array([np.array(fr['transform_matrix'])[:3, 3] for fr in tr['frames']])
    # |p - c|^2 = r^2  →  2 p·c + (r^2 - |c|^2) = |p|^2 — linear in (c, k)
    A = np.hstack([2 * P, np.ones((len(P), 1))])
    b = (P ** 2).sum(axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    c, k = sol[:3], sol[3]
    radius = float(np.sqrt(k + (c ** 2).sum()))
    ground_px = radius / tr['fl_x']
    blur = 2.0 * ground_px
    print(f"  auto-blur: orbit radius {radius:.0f} m, render {ground_px:.2f} m/px "
          f"→ blur {blur:.2f} m")
    return blur

T0 = time.time()
def log(msg):
    print(f"  [{time.time()-T0:7.1f}s] {msg}", flush=True)

# sRGB <-> linear (LUT for decode, direct for encode)
_LUT = np.where(np.arange(256)/255.0 <= 0.04045,
                np.arange(256)/255.0/12.92,
                ((np.arange(256)/255.0 + 0.055)/1.055)**2.4).astype(np.float32)

def to_srgb(lin):
    lin = np.clip(lin, 0.0, 1.0)
    return np.where(lin <= 0.0031308, lin*12.92, 1.055*lin**(1/2.4) - 0.055)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('lit_dir')
    ap.add_argument('raster')
    ap.add_argument('out_dir')
    ap.add_argument('--capture', default=None,
                    help='capture dir (transforms.json) — auto-compute blur')
    ap.add_argument('--blur', type=float, default=None,
                    help='manual blur radius in metres (overrides --capture)')
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    if args.blur is not None:
        blur_m = args.blur
    elif args.capture:
        blur_m = auto_blur(args.capture)
    else:
        blur_m = 2.0
        print("  no --capture / --blur given — using default 2.0 m")

    img = Image.open(args.raster)
    scale = img.tag_v2[33550]
    tie = img.tag_v2[33922]
    px, py = float(scale[0]), float(scale[1])
    x0, y0 = float(tie[3]), float(tie[4])
    alb = _LUT[np.asarray(img.convert('RGB'))]          # linear float32 HxWx3
    log(f"ortho {alb.shape[1]}x{alb.shape[0]} @ {px:g} m/px")

    sigma = blur_m / px
    alb_blur = np.stack([gaussian_filter(alb[:, :, c], sigma) for c in range(3)], axis=2)
    log(f"blurred albedo (sigma {sigma:.1f} px = {blur_m:.2f} m)")

    H, W = alb.shape[:2]

    def sample(arr, xy):
        col = np.clip(((xy[:, 0] - x0) / px).astype(np.int64), 0, W-1)
        row = np.clip(((y0 - xy[:, 1]) / py).astype(np.int64), 0, H-1)
        return arr[row, col]

    EPS = 0.004   # ~1/255 in linear — below this the albedo is black/no-data

    for tile in sorted(os.listdir(args.lit_dir)):
        if not tile.endswith('_lit.laz'):
            continue
        out_path = os.path.join(args.out_dir, tile)
        if os.path.exists(out_path):
            log(f"skip {tile} (exists)")
            continue
        las = laspy.read(os.path.join(args.lit_dir, tile))
        xy = np.stack([np.asarray(las.x), np.asarray(las.y)], axis=1)
        lit = _LUT[(np.stack([np.asarray(las.red), np.asarray(las.green),
                              np.asarray(las.blue)], axis=1) // 257).astype(np.uint8)]
        log(f"{tile}: {len(xy):,} pts")

        a = sample(alb, xy)                     # sharp albedo
        ab = sample(alb_blur, xy)               # render-resolution albedo
        ratio = lit / np.maximum(ab, EPS)       # lighting, texture removed
        final = a * ratio
        rgb16 = (to_srgb(final) * 255.0 + 0.5).astype(np.uint16) * 257

        header = laspy.LasHeader(version="1.4", point_format=7)
        header.offsets = las.header.offsets
        header.scales = las.header.scales
        out = laspy.LasData(header)
        out.x, out.y, out.z = las.x, las.y, las.z
        out.red, out.green, out.blue = rgb16[:, 0], rgb16[:, 1], rgb16[:, 2]
        tmp = out_path[:-4] + '_tmp.laz'
        out.write(tmp)
        os.replace(tmp, out_path)
        log(f"wrote {out_path}")

    print("Done — untwine the output dir into a COPC.")


if __name__ == '__main__':
    main()
