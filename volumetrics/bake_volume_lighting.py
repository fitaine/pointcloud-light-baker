"""
bake_volume_lighting.py — bake scene lighting into volume voxels (numpy only).

Single scattering, generic across scenes: every light from scene_lights.json
contributes  irradiance(falloff) x self-shadow transmittance (ray-marched
through the density grid toward the light) x volume albedo.  Output is a
colored emission grid the viewer's ray-marcher integrates directly — the
browser pays nothing for lighting complexity.

    lit_rgb(voxel) = albedo x sum_l  E_l(voxel) x T_l(voxel)
    sigma(voxel)   = density x density_input        (extinction)

K_VOL is the single cross-scene constant converting Blender light units to
display radiance (same philosophy as the old bake's K_GLOBAL — calibrate
once against a Cycles render of a volume, never per scene).

Usage:
  python bake_volume_lighting.py <slug> [--dir volumetrics] [--kvol 1.0]
  reads  <slug>_density.npy, <slug>_meta.json, scene_lights.json
  writes <slug>_lit.npz  {sigma float32 (X,Y,Z), rgb float16 (X,Y,Z,3)}
"""

import argparse
import json
import os
import time
import numpy as np

T0 = time.time()
def log(m):
    print(f"  [{time.time()-T0:6.1f}s] {m}", flush=True)

K_VOL_DEFAULT = 12.0      # Blender W → display radiance for volumes
SHADOW_STEPS = 48         # transmittance march steps toward each light
MIN_DIST = 1.0


def transmittance(sigma, grid_min, cell, pts, light_dir_or_pos, is_dir):
    """exp(-integral of sigma) from each point toward the light."""
    n = len(pts)
    if is_dir:
        d = -np.asarray(light_dir_or_pos, dtype=np.float32)
        d = np.broadcast_to(d / np.linalg.norm(d), (n, 3))
        far = np.full(n, (sigma.shape * cell).max() * 1.2, dtype=np.float32)
    else:
        d = np.asarray(light_dir_or_pos, dtype=np.float32) - pts
        far = np.linalg.norm(d, axis=1)
        d = d / np.maximum(far[:, None], 1e-6)
    tau = np.zeros(n, dtype=np.float32)
    ts = (np.arange(SHADOW_STEPS, dtype=np.float32) + 0.5) / SHADOW_STEPS
    for t in ts:
        p = pts + d * (far[:, None] * t[None] if np.ndim(t) else far[:, None] * t)
        ijk = ((p - grid_min) / cell).astype(np.int32)
        ok = np.all((ijk >= 0) & (ijk < sigma.shape), axis=1)
        s = np.zeros(n, dtype=np.float32)
        s[ok] = sigma[ijk[ok, 0], ijk[ok, 1], ijk[ok, 2]]
        tau += s * (far / SHADOW_STEPS)
    return np.exp(-tau)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('slug')
    ap.add_argument('--dir', default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument('--kvol', type=float, default=K_VOL_DEFAULT)
    ap.add_argument('--ambient', type=float, default=0.25,
                    help='world-light weight; 0 = beam/sun only, the unlit '
                         'fog stays dark and absorbs (no visible volume box)')
    args = ap.parse_args()
    D = args.dir

    dens = np.load(os.path.join(D, f'{args.slug}_density.npy'))
    with open(os.path.join(D, f'{args.slug}_meta.json')) as f:
        meta = json.load(f)
    with open(os.path.join(D, 'scene_lights.json')) as f:
        lights = json.load(f)
    sh = meta.get('shader') or {}
    albedo = np.array(sh.get('color', [1, 1, 1]), dtype=np.float32)
    sigma = (dens * float(sh.get('density_input', 1.0))).astype(np.float32)

    gmin = np.array(meta['min'], dtype=np.float32)
    gmax = np.array(meta['max'], dtype=np.float32)
    shape = np.array(meta['shape'])
    cell = (gmax - gmin) / shape
    log(f"grid {tuple(shape)}  cell {cell.round(1).tolist()} m  "
        f"{len(lights)} lights")

    occ = np.argwhere(sigma > 1e-4)
    pts = gmin + (occ + 0.5) * cell
    log(f"{len(occ):,} occupied voxels")

    rgb = np.zeros((len(occ), 3), dtype=np.float32)
    for L in lights:
        col = np.array(L['color'], dtype=np.float32)
        if L['type'] == 'WORLD':
            if args.ambient > 0:
                rgb += col * L['energy'] * args.ambient   # unshadowed
            log(f"world ambient {L['energy']:.2f} x {args.ambient}")
            continue
        if L['type'] == 'SUN':
            E = np.full(len(occ), L['energy'], dtype=np.float32)
            T = transmittance(sigma, gmin, cell, pts, L['direction'], True)
        else:   # POINT / SPOT / AREA — inverse square from light position
            lp = np.array(L['location'], dtype=np.float32)
            # clamp at one cell, not 1 m — a voxel containing the light
            # otherwise blows up the dynamic range and the 8-bit pack
            # crushes the rest of the volume to black
            min_d = max(MIN_DIST, float(cell.max()))
            d2 = np.maximum(((pts - lp) ** 2).sum(1), min_d ** 2)
            E = L['energy'] / (4 * np.pi * d2)
            if L['type'] == 'SPOT':
                axis = np.array(L['direction'], dtype=np.float32)
                axis /= np.linalg.norm(axis)
                to_p = (pts - lp) / np.sqrt(d2)[:, None]
                cosang = (to_p * axis).sum(1)
                half = L['spot_size'] / 2
                edge0 = np.cos(half)
                edge1 = np.cos(half * (1 - L.get('spot_blend', 0.1)))
                E = E * np.clip((cosang - edge0) / max(edge1 - edge0, 1e-4), 0, 1)
            T = transmittance(sigma, gmin, cell, pts, lp, False)
        rgb += col[None] * (E * T)[:, None]
        log(f"{L['name']} ({L['type']}) done")

    rgb *= albedo[None] * args.kvol
    out = np.zeros((*sigma.shape, 3), dtype=np.float16)
    out[occ[:, 0], occ[:, 1], occ[:, 2]] = rgb.astype(np.float16)
    np.savez_compressed(os.path.join(D, f'{args.slug}_lit.npz'),
                        sigma=sigma, rgb=out,
                        grid_min=gmin, grid_max=gmax)
    log(f"saved {args.slug}_lit.npz   rgb max {rgb.max():.3f} mean {rgb.mean():.4f}")


if __name__ == '__main__':
    main()
