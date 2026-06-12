"""
make_test_volume.py — synthetic density grids to sanity-check the volumetrics
pipeline (bake_volume_lighting → pack_volume_web → viewer) without Blender.

Two test cases, written each into their own subdir so their scene_lights.json
never clobbers a real export:

  test-fog/    ground-fog layer with height falloff + value noise, lit by a
               copy of the real Mont Aiguille scene lights (moon point light)
  test-beam/   uniform thin fog box + synthetic SPOT shining down through it
               — the classic visible light-beam test

Both are placed in blend-world frame next to the Anneau de nuage so they can
be packed with the Mont Aiguille capture alignment and viewed over the same
terrain with ?vol=test-fog / ?vol=test-beam.

Usage:
  python make_test_volume.py
  python bake_volume_lighting.py test-fog  --dir test-fog
  python bake_volume_lighting.py test-beam --dir test-beam
  python pack_volume_web.py test-fog  "../gs-capture/output/2023-09-19_Mont aiguille - 2" --dir test-fog
  python pack_volume_web.py test-beam "../gs-capture/output/2023-09-19_Mont aiguille - 2" --dir test-beam
"""

import json
import os
import shutil
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def value_noise_3d(shape, cells, rng):
    """Cheap trilinear value noise — enough to break up a fog slab."""
    coarse = rng.random((cells, cells, cells)).astype(np.float32)
    out = coarse
    # upsample with repeated linear interpolation along each axis
    for axis, n in enumerate(shape):
        idx = np.linspace(0, out.shape[axis] - 1, n)
        lo = np.floor(idx).astype(int)
        hi = np.minimum(lo + 1, out.shape[axis] - 1)
        f = (idx - lo).astype(np.float32)
        a = np.take(out, lo, axis=axis)
        b = np.take(out, hi, axis=axis)
        fshape = [1, 1, 1]
        fshape[axis] = n
        out = a + (b - a) * f.reshape(fshape)
    return out


def write_case(slug, dens, mn, mx, shader, lights):
    d = os.path.join(HERE, slug)
    os.makedirs(d, exist_ok=True)
    np.save(os.path.join(d, f'{slug}_density.npy'), dens.astype(np.float32))
    meta = {'object': slug, 'min': list(mn), 'max': list(mx),
            'shape': list(dens.shape), 'frame': 'blend-world',
            'shader': shader}
    with open(os.path.join(d, f'{slug}_meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    with open(os.path.join(d, 'scene_lights.json'), 'w') as f:
        json.dump(lights, f, indent=1)
    occ = (dens > 0.005).mean() * 100
    print(f"{slug}: grid {dens.shape}  bbox {[round(v) for v in mn]} .. "
          f"{[round(v) for v in mx]}  occupancy {occ:.1f}%")


# ── test-fog — ground fog bank SW of the summit, real scene lights ───────────
# Anneau de nuage bbox (blend frame): [-1739,-2054,236] .. [222,-75,1159]
NX, NY, NZ = 192, 192, 40
mn = [-1700.0, -2000.0, 150.0]
mx = [200.0, -100.0, 550.0]
rng = np.random.default_rng(7)
zs = np.linspace(mn[2], mx[2], NZ, dtype=np.float32)
height = np.exp(-(zs - mn[2]) / 120.0)                     # denser near ground
noise = value_noise_3d((NX, NY, NZ), 6, rng)
dens = np.clip(noise * 1.6 - 0.45, 0, None) * height[None, None, :]
with open(os.path.join(HERE, 'scene_lights.json')) as f:
    real_lights = json.load(f)
write_case('test-fog', dens, mn, mx,
           {'color': [0.9, 0.9, 0.9], 'density_input': 0.012,
            'anisotropy': 0.0, 'emission_strength': 0.0,
            'emission_color': [1, 1, 1]},
           real_lights)

# ── test-beam — uniform fog box + synthetic spot shining down through it ─────
NX, NY, NZ = 96, 96, 120
mn = [-1100.0, -1500.0, 200.0]
mx = [-500.0, -900.0, 1400.0]
dens = np.ones((NX, NY, NZ), dtype=np.float32)
# soft edges so the box doesn't read as a hard cube
for axis, n in enumerate((NX, NY, NZ)):
    r = np.linspace(-1, 1, n, dtype=np.float32)
    edge = np.clip((1 - np.abs(r)) / 0.15, 0, 1)
    sh = [1, 1, 1]
    sh[axis] = n
    dens *= edge.reshape(sh)
spot = {'name': 'TestSpot', 'type': 'SPOT', 'energy': 8.0e6,
        'color': [1.0, 0.95, 0.85],
        'location': [-800.0, -1200.0, 2000.0],
        'direction': [0.0, 0.0, -1.0],
        'spot_size': 0.5, 'spot_blend': 0.3}
ambient = {'name': '__world__', 'type': 'WORLD',
           'color': [0.05, 0.07, 0.1], 'energy': 0.3}
write_case('test-beam', dens, mn, mx,
           {'color': [0.9, 0.9, 0.9], 'density_input': 0.004,
            'anisotropy': 0.0, 'emission_strength': 0.0,
            'emission_color': [1, 1, 1]},
           [spot, ambient])

print("done — now bake + pack (see usage in the docstring)")
