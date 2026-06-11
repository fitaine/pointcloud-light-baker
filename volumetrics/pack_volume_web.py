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
    args = ap.parse_args()

    z = np.load(os.path.join(args.dir, f'{args.slug}_lit.npz'))
    sigma = z['sigma'].astype(np.float32)
    rgb = z['rgb'].astype(np.float32)
    gmin, gmax = z['grid_min'].astype(np.float64), z['grid_max'].astype(np.float64)

    with open(os.path.join(args.capture_dir, 'alignment.json')) as f:
        off = np.array(json.load(f)['blend_to_lambert'])

    sigma_max = float(sigma.max()) or 1.0
    rgb_max = float(rgb.max()) or 1.0
    NX, NY, NZ = sigma.shape
    # WebGL 3D texture layout: x fastest, then y, then z
    rgba = np.empty((NZ, NY, NX, 4), dtype=np.uint8)
    rgba[..., 0] = np.clip(rgb[..., 0].transpose(2, 1, 0) / rgb_max * 255, 0, 255)
    rgba[..., 1] = np.clip(rgb[..., 1].transpose(2, 1, 0) / rgb_max * 255, 0, 255)
    rgba[..., 2] = np.clip(rgb[..., 2].transpose(2, 1, 0) / rgb_max * 255, 0, 255)
    rgba[..., 3] = np.clip(sigma.transpose(2, 1, 0) / sigma_max * 255, 0, 255)

    os.makedirs(POTREE_VOLUMES, exist_ok=True)
    rgba.tofile(os.path.join(POTREE_VOLUMES, f'{args.slug}.bin'))
    meta = {'shape': [int(NX), int(NY), int(NZ)],
            'min': (gmin + off).tolist(),
            'max': (gmax + off).tolist(),
            'sigma_max': sigma_max, 'rgb_max': rgb_max}
    with open(os.path.join(POTREE_VOLUMES, f'{args.slug}.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    print(f"packed {NX}x{NY}x{NZ} -> potree/volumes/{args.slug}.bin "
          f"({rgba.nbytes/1e6:.1f} MB)  lambert bbox "
          f"{[round(v) for v in meta['min']]} .. {[round(v) for v in meta['max']]}")


if __name__ == '__main__':
    main()
