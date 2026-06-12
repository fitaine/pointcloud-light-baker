"""
pack_volume_web.py — pack a lit volume grid for the Potree viewer ray-marcher.

Converts <slug>_lit.npz (blend-world frame) into:
  potree/volumes/<slug>.bin   RGBA8, x-fastest (RGB = lit radiance / rgb_max,
                              A = sigma / sigma_max)
  potree/volumes/<slug>.json  shape, LAMBERT bbox, rgb_max, sigma_max

The Lambert conversion uses the capture's alignment.json (blend + offset),
so the volume lands exactly where the point cloud is.

Usage:
  python pack_volume_web.py <slug> <capture_dir> [--dir volumetrics]
"""

import argparse
import json
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
POTREE_VOLUMES = os.path.normpath(os.path.join(HERE, '..', 'potree', 'volumes'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('slug')
    ap.add_argument('capture_dir')
    ap.add_argument('--dir', default=HERE)
    ap.add_argument('--lit-only', action='store_true',
                    help='unlit fog gets zero extinction — the volume box is '
                         'completely invisible, only the lit beam interacts')
    args = ap.parse_args()

    z = np.load(os.path.join(args.dir, f'{args.slug}_lit.npz'))
    sigma = z['sigma'].astype(np.float32)
    rgb = z['rgb'].astype(np.float32)
    gmin, gmax = z['grid_min'].astype(np.float64), z['grid_max'].astype(np.float64)

    with open(os.path.join(args.capture_dir, 'alignment.json')) as f:
        off = np.array(json.load(f)['blend_to_lambert'])

    if args.lit_only:
        # extinction follows the light: fully opaque inside the beam, fading
        # smoothly to zero where the fog is unlit (soft ramp avoids a hard
        # silhouette at the beam border). The box itself becomes invisible.
        lum = rgb.max(axis=-1)
        ref = float(np.percentile(lum[lum > 0], 99.9)) if (lum > 0).any() else 1.0
        sigma = sigma * np.clip(lum / (ref * 0.01), 0, 1).astype(np.float32)
        print(f"lit-only: extinction ramped on luminance (ref {ref:.3f})")

    sigma_max = float(sigma.max()) or 1.0
    # Normalize by a high percentile of the LIT voxels, not the absolute max:
    # a handful of voxels near a light otherwise own the whole 8-bit range
    # and the rest of the volume quantizes to black. The top 0.1% clips.
    lit = rgb.max(axis=-1)
    lit = lit[lit > 0]
    rgb_max = float(np.percentile(lit, 99.9)) if lit.size else 1.0
    rgb_max = rgb_max or 1.0
    clipped = float((lit > rgb_max).mean() * 100) if lit.size else 0.0
    print(f"rgb_max {rgb_max:.3f} (99.9th pct, abs max {float(rgb.max()):.1f}, "
          f"{clipped:.2f}% voxels clip)")
    NX, NY, NZ = sigma.shape
    # WebGL 3D texture layout: x fastest, then y, then z
    rgba = np.empty((NZ, NY, NX, 4), dtype=np.uint8)
    # sqrt-encode RGB: linear 8-bit starves the dim end (visible banding in
    # light-beam falloffs); sqrt gives the low values ~16x more code points.
    # The viewer squares the sample back (meta encoding: "sqrt").
    for c in range(3):
        ch = np.clip(rgb[..., c].transpose(2, 1, 0) / rgb_max, 0, 1)
        rgba[..., c] = np.sqrt(ch) * 255
    rgba[..., 3] = np.clip(sigma.transpose(2, 1, 0) / sigma_max * 255, 0, 255)

    os.makedirs(POTREE_VOLUMES, exist_ok=True)
    rgba.tofile(os.path.join(POTREE_VOLUMES, f'{args.slug}.bin'))
    meta = {'shape': [int(NX), int(NY), int(NZ)],
            'min': (gmin + off).tolist(),
            'max': (gmax + off).tolist(),
            'sigma_max': sigma_max, 'rgb_max': rgb_max,
            'encoding': 'sqrt'}
    with open(os.path.join(POTREE_VOLUMES, f'{args.slug}.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    print(f"packed {NX}x{NY}x{NZ} -> potree/volumes/{args.slug}.bin "
          f"({rgba.nbytes/1e6:.1f} MB)  lambert bbox "
          f"{[round(v) for v in meta['min']]} .. {[round(v) for v in meta['max']]}")


if __name__ == '__main__':
    main()
