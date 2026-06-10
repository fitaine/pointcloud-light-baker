"""
bake_lighting.py — Apply Blender scene lighting to a LiDAR point cloud PLY.

Usage:
    python bake_lighting.py <input.ply> <scene_lights.json> [output.ply]

If output.ply is omitted, writes <input>-lit.ply in the same folder.
Requires: numpy

Works for any scene; all parameters come from scene_lights.json.
"""

import sys
import os
import json
import time
import numpy as np
import math

PI4 = 4.0 * math.pi
PI1 = math.pi

def _t(label, t0):
    print(f"  [{time.time()-t0:5.1f}s] {label}", flush=True)


# ── Per-point surface normals (k-NN PCA) ─────────────────────────────────
# The point cloud has no normals, but Blender's dramatic clair-obscur comes
# almost entirely from Lambertian cos(θ) shading: surfaces facing a light are
# bright, surfaces facing away are black.  Without this term every point in a
# light's cone is lit equally → the render's sculpted pool of light becomes a
# flat, uniformly-bright wash (the "too bright / washed out" bug).
#
# We recover normals from local geometry: for each point, the smallest-variance
# axis of its k nearest neighbours is the surface normal (standard PCA normal
# estimation).  Oriented upward — correct for terrain scans illuminated from above.
def estimate_normals(world, k=16, chunk=200_000):
    from scipy.spatial import cKDTree
    N = len(world)
    tree = cKDTree(world)
    normals = np.empty((N, 3), dtype=np.float64)
    for start in range(0, N, chunk):
        end = min(start + chunk, N)
        _, nbr = tree.query(world[start:end], k=k, workers=-1)
        nb   = world[nbr]                       # (B, k, 3)
        nb   = nb - nb.mean(axis=1, keepdims=True)
        cov  = np.einsum('bki,bkj->bij', nb, nb)  # (B, 3, 3)
        w, v = np.linalg.eigh(cov)              # ascending eigenvalues
        n    = v[:, :, 0]                       # smallest-variance axis = normal
        normals[start:end] = n
    # Orient consistently upward (terrain lit from above)
    flip = normals[:, 2] < 0
    normals[flip] *= -1.0
    return normals


# ── Terrain self-shadowing (heightfield ray-march) ───────────────────────
# cos θ shading alone floods a light's whole cone — but Cycles also blocks light
# wherever terrain rises between a point and the lamp.  This is decisive for
# grazing lights: a low spot throws long ridge shadows that carve the smooth
# wash into the broken light/dark pattern the render shows.
#
# The terrain is a heightfield (z ≈ f(x,y)), so we rasterise a top-down max-height
# grid, then march each point's ray toward the light and test whether the surface
# pokes above the ray.  Empty cells (missing tiles, scene edges) stay −∞ → they
# never cast a shadow, so data holes don't create dark artefacts.

SHADOW_ENABLE = True
SHADOW_CELL   = 6.0          # heightmap cell size, metres
SHADOW_STEPS  = 48           # ray samples (geometric spacing in horizontal distance)
SHADOW_DMIN   = 12.0         # start marching this far out → never self-shadow the start cell
SHADOW_DMAX   = 2000.0       # furthest occluder distance (long grazing shadows reach ~2 km)
SHADOW_BIAS   = 4.0          # metres — clearance above the ray before it counts as blocked
SHADOW_CHUNK  = 2_000_000    # points per chunk (bounds peak memory)

def build_heightmap(world, cell=SHADOW_CELL):
    x0, y0 = world[:, 0].min(), world[:, 1].min()
    x1, y1 = world[:, 0].max(), world[:, 1].max()
    nx = int((x1 - x0) / cell) + 2
    ny = int((y1 - y0) / cell) + 2
    gx = ((world[:, 0] - x0) / cell).astype(np.int64)
    gy = ((world[:, 1] - y0) / cell).astype(np.int64)
    np.clip(gx, 0, nx - 1, out=gx)
    np.clip(gy, 0, ny - 1, out=gy)
    hmap = np.full(nx * ny, -1e30, dtype=np.float64)
    np.maximum.at(hmap, gx * ny + gy, world[:, 2])    # top surface per cell
    return (hmap.reshape(nx, ny), x0, y0, cell, nx, ny)

def terrain_shadow(world, light, hpack, n_steps=SHADOW_STEPS, bias=SHADOW_BIAS):
    """Return (N,) float: 1.0 = lit, 0.0 = in terrain shadow for this light.

    Marches outward from each point by HORIZONTAL distance toward the light's
    horizontal direction, starting SHADOW_DMIN away so a point is never shadowed
    by its own heightmap cell.  At each distance the ray height is P.z + slope·d
    (slope = vertical rise per horizontal metre toward the lamp); if the terrain
    surface there pokes above the ray, the point is in shadow.
    """
    hmap, x0, y0, cell, nx, ny = hpack
    inv = 1.0 / cell
    N   = len(world)
    shadow = np.ones(N, dtype=np.float64)

    # Geometric horizontal-distance schedule: fine near the point, coarse far out.
    d_k = SHADOW_DMIN * (SHADOW_DMAX / SHADOW_DMIN) ** (np.arange(n_steps) / (n_steps - 1))

    is_sun = light['type'] == 'SUN'
    if is_sun:
        ldir = np.array(light['direction'], dtype=np.float64)
        Lh   = -ldir[:2]                                     # horizontal toward light
        hlen = math.hypot(Lh[0], Lh[1])
        if hlen < 1e-6:
            return shadow                                    # lamp overhead → no shadow
        hdir_sun = Lh / hlen
        slope_sun = (-ldir[2]) / hlen                        # rise per horizontal metre
    else:
        Lp = np.array(light['position'], dtype=np.float64)

    for s in range(0, N, SHADOW_CHUNK):
        e   = min(s + SHADOW_CHUNK, N)
        P   = world[s:e]
        B   = len(P)
        lit = np.ones(B, dtype=bool)

        if is_sun:
            hdir  = np.broadcast_to(hdir_sun, (B, 2))
            slope = np.full(B, slope_sun)
            horiz_to_light = np.full(B, np.inf)
        else:
            dxy   = Lp[:2][None, :] - P[:, :2]
            horiz_to_light = np.hypot(dxy[:, 0], dxy[:, 1])
            safe  = horiz_to_light > SHADOW_DMIN
            hdir  = np.zeros((B, 2))
            hdir[safe] = dxy[safe] / horiz_to_light[safe, None]
            slope = np.where(safe, (Lp[2] - P[:, 2]) / np.maximum(horiz_to_light, 1e-6), 0.0)

        for d in d_k:
            valid = d < horiz_to_light                       # don't march past the lamp
            qx = P[:, 0] + d * hdir[:, 0]
            qy = P[:, 1] + d * hdir[:, 1]
            ray_z = P[:, 2] + slope * d
            ix = ((qx - x0) * inv).astype(np.int64)
            iy = ((qy - y0) * inv).astype(np.int64)
            inb = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
            h   = hmap[np.where(inb, ix, 0), np.where(inb, iy, 0)]
            occ = valid & inb & (h > ray_z + bias)
            lit &= ~occ
        shadow[s:e] = lit.astype(np.float64)

    return shadow


# ── sRGB ↔ linear ──────────────────────────────────────────────────────
# LUT for uint8→linear: only 256 values, computed once at import.
# Avoids the slow ** 2.4 power on millions of floats.
_SRGB_LUT = np.where(
    np.arange(256) / 255.0 <= 0.04045,
    np.arange(256) / 255.0 / 12.92,
    ((np.arange(256) / 255.0 + 0.055) / 1.055) ** 2.4
).astype(np.float64)

def srgb_to_linear_lut(rgb_uint8):
    """uint8 (N,3) → linear float64 (N,3) via lookup table — instant."""
    return _SRGB_LUT[rgb_uint8]

def srgb_to_linear(c):
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)

def linear_to_srgb(c):
    c = np.clip(c, 0.0, None)
    return np.where(c <= 0.0031308, c * 12.92, 1.055 * c ** (1.0 / 2.4) - 0.055)


# ── ACES filmic tone mapping (approximates Blender Filmic High Contrast) ──

def aces(x):
    x = np.clip(x, 0.0, None)
    return (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14)


# ── PLY I/O ────────────────────────────────────────────────────────────

def read_ply(path):
    """
    Returns (xyz: float32 Nx3, rgb: uint8 Nx3).
    Handles binary_little_endian PLY with any property order.
    """
    with open(path, 'rb') as f:

        # ---- header ----
        lines = []
        while True:
            line = f.readline().decode('ascii', errors='replace').strip()
            lines.append(line)
            if line == 'end_header':
                break
        header = '\n'.join(lines)

        n_verts = 0
        for l in lines:
            if l.startswith('element vertex'):
                n_verts = int(l.split()[-1])
                break

        is_be = 'format binary_big_endian' in header

        # parse vertex properties
        props, in_v = [], False
        type_map = {
            'float': 'f4', 'float32': 'f4',
            'double': 'f8', 'float64': 'f8',
            'int': 'i4',   'int32': 'i4',
            'uint': 'u4',  'uint32': 'u4',
            'short': 'i2', 'int16': 'i2',
            'ushort': 'u2','uint16': 'u2',
            'char': 'i1',  'int8': 'i1',
            'uchar': 'u1', 'uint8': 'u1',
        }
        for l in lines:
            if l.startswith('element vertex'):
                in_v = True
            elif l.startswith('element') and 'vertex' not in l:
                in_v = False
            elif in_v and l.startswith('property'):
                parts = l.split()
                endian = '>' if is_be else '<'
                np_t = endian + type_map.get(parts[1], 'u1')
                props.append((parts[2], np_t))

        dtype = np.dtype([(name, t) for name, t in props])
        data  = np.frombuffer(f.read(n_verts * dtype.itemsize), dtype=dtype)

    xyz = np.stack([data['x'], data['y'], data['z']], axis=1).astype(np.float32)

    # colour field names vary by exporter
    for rn in ('red',   'diffuse_red',   'r'):
        if rn in data.dtype.names: break
    for gn in ('green', 'diffuse_green', 'g'):
        if gn in data.dtype.names: break
    for bn in ('blue',  'diffuse_blue',  'b'):
        if bn in data.dtype.names: break

    rgb = np.stack([data[rn], data[gn], data[bn]], axis=1).astype(np.uint8)
    return xyz, rgb


def write_ply(path, xyz, rgb):
    """Write binary little-endian PLY: float32 xyz + uint8 rgb."""
    n = len(xyz)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )
    dt = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),
                   ('red','u1'),('green','u1'),('blue','u1')])
    out = np.empty(n, dtype=dt)
    out['x'] = xyz[:,0]; out['y'] = xyz[:,1]; out['z'] = xyz[:,2]
    out['red'] = rgb[:,0]; out['green'] = rgb[:,1]; out['blue'] = rgb[:,2]
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(out.tobytes())


# ── Lighting ────────────────────────────────────────────────────────────

def apply_lighting(xyz_ply, rgb_uint8, scene):
    """
    xyz_ply   : float32 (N,3) — PLY-local coords (as stored in the file)
    rgb_uint8 : uint8   (N,3) — sRGB satellite colours
    scene     : dict loaded from scene_lights.json
    Returns     uint8   (N,3) — sRGB lit colours
    """
    N  = len(xyz_ply)
    t0 = time.time()
    print(f"  {N:,} points", flush=True)

    # ── PLY local → world ────────────────────────────────────────────
    # The lights are exported in Blender-world space.  The point cloud must be
    # placed in that SAME space or every light lands on the wrong terrain.
    #
    # The robust mapping is the PC object's matrix_world (exported as
    # pc_transform.matrix): the lidar pipeline keeps every crop of a scene in one
    # shared-origin frame and imports it at identity, so PLY-local coords already
    # ARE world coords (matrix = identity → no-op).  Applying the matrix is correct
    # for ANY crop/extent of the scene, which is what makes it consistent across
    # scenes.
    #
    # The old method (shift this PLY's bbox-centre onto ply_world_center) only
    # worked when the baked PLY was the exact same crop loaded in Blender — it
    # silently mis-placed every light by (bbox_center_baked − bbox_center_blender)
    # whenever you baked a different/extended crop (e.g. Chamechaude 055 vs 029,
    # an 865 m error).  Kept only as a fallback when no matrix is present.
    xyz64 = xyz_ply.astype(np.float64)
    pct   = scene['pc_transform']
    if 'matrix' in pct:
        M     = np.array(pct['matrix'], dtype=np.float64)               # 4×4 world
        world = xyz64 @ M[:3, :3].T + M[:3, 3]
        print(f"  Align: matrix_world (translation {np.round(M[:3,3],1).tolist()})", flush=True)
    else:
        bb_center = (xyz64.min(axis=0) + xyz64.max(axis=0)) / 2.0
        world_ctr = np.array(pct['ply_world_center'], dtype=np.float64)
        world     = xyz64 + (world_ctr - bb_center)
        print("  Align: bbox-center → ply_world_center (legacy fallback)", flush=True)
    _t("coords → world", t0)

    # ── Satellite colour: sRGB uint8 → linear float (LUT, instant) ───
    sat_lin = srgb_to_linear_lut(rgb_uint8)                             # N×3 float64
    _t("sRGB → linear", t0)

    # ── Per-point surface normals — the term that makes it match Blender ──
    normals = estimate_normals(world)                                   # N×3 unit
    _t("normals (k-NN PCA)", t0)

    # ── Heightmap for terrain self-shadowing ─────────────────────────────
    hpack = build_heightmap(world) if SHADOW_ENABLE else None
    if hpack is not None:
        _t(f"heightmap {hpack[0].shape}", t0)

    # ── Irradiance accumulator, start at black ───────────────────────
    # No ambient: terrain is dark by default, only actual lights illuminate it.
    # World ambient (0.051) would make every point glow with its own satellite
    # texture — exactly the "self-emitting terrain" effect we want to avoid.
    irradiance = np.zeros((N, 3), dtype=np.float64)                      # N×3

    # ── Global exposure ───────────────────────────────────────────────
    # K = 2 ** render_exposure_ev — extracted from Blender by extract_lights.py.
    # This exactly replicates the EV setting in Blender's Color Management
    # panel (Color Management → View Transform → Exposure).
    #
    # Our energy / (4πr²) formula matches Blender Cycles exactly (K=1 for 0 EV).
    # ACES ≈ Blender Filmic Medium Contrast within ~5% for all values.
    # Together these two facts mean baked output matches a Blender render
    # pixel-for-pixel for any EV setting, without any per-scene tweaking.
    #
    # Manual override: add "exposure": <multiplier> to scene_lights.json to
    # apply an additional artistic multiplier on top of the EV value.
    # K_GLOBAL = 157 is calibrated empirically: Blender Cycles (Filmic MC, 0 EV)
    # produces median linear radiance 0.521 for terrain under a 10 kW SPOT at 354 m.
    # Our formula gives L=0.001057 for the same point → K = 0.521/0.001057 = 157.
    # This constant applies to all scenes (same Blender light-unit scaling).
    K_GLOBAL = 157.0
    # SUN uses a SEPARATE constant: Blender's sun "Strength" is already irradiance
    # in W/m² (not a lamp Wattage needing the 1/4πr² + unit conversion that K_GLOBAL
    # absorbs).  So a SUN's radiance is just sat × (strength × cosθ) × K_SUN, where
    # K_SUN ≈ 0.105 is the radiance-per-irradiance ratio measured against a moon-only
    # Cycles render (Alpe d'Huez: matched mean raw radiance 0.00345).  Applying K_GLOBAL
    # to a SUN instead overshoots ~1500× and whites out the whole scene.  Global, not
    # per-scene: sun strength is physical, so the ratio is the same everywhere.
    K_SUN    = 0.105
    # Emission curves use the line-light weight (strength × segment_length), a third
    # unit system again.  K_CURVE ≈ 0.284 matches a curve-only Cycles render
    # (Alpe d'Huez bezier: mean raw radiance 0.01012).  NOTE: our line-light ignores
    # the curve's bevel/tube radius (emission area) — if a future scene's glowing
    # curve has a very different bevel_depth, this constant may need revisiting.
    K_CURVE  = 0.19   # 0.284 × 0.67: wrap lighting raises mean ndl ~50%, compensate here
    ev       = float(scene.get('render_exposure_ev', 0.0))
    EXPOSURE = (2.0 ** ev) * K_GLOBAL * float(scene.get('exposure', 1.0))
    # Per-scene curve brightness trim: add "curve_exposure": <float> to scene_lights.json.
    # Default 1.0 = calibrated value.  Use e.g. 0.5 if curves are too bright on a new scene.
    CURVE_EXPOSURE = float(scene.get('curve_exposure', 1.0))
    print(f"  K_global={K_GLOBAL:.0f}  EV={ev:+.2f} → EXPOSURE={EXPOSURE:.1f}×"
          + (f"  (exposure ×{scene.get('exposure', 1.0):.2f})" if 'exposure' in scene else "")
          + (f"  curve_exposure ×{CURVE_EXPOSURE:.2f}" if 'curve_exposure' in scene else ""),
          flush=True)

    # Minimum lamp-to-point distance (metres) to avoid division by zero when a
    # point coincides with a light position.  0.5 m is safe — no terrain point
    # sits inside a lamp.
    MIN_DIST  = 0.5
    MIN_DISTSQ = MIN_DIST * MIN_DIST

    # ── Point & directional lights ────────────────────────────────────
    print(f"  {len(scene['lights'])} light(s) …", flush=True)
    for light in scene['lights']:
        lpos   = np.array(light['position'], dtype=np.float64)           # (3,)
        lcol   = np.array(light['color'],    dtype=np.float64)           # (3,)
        energy = float(light['energy']) * float(light.get('energy_scale', 1.0))
        ltype  = light['type']

        diff     = world - lpos                                          # N×3
        dist_sq  = np.einsum('ij,ij->i', diff, diff)                    # N

        # Clamp to MIN_DIST only — no artificial saturation zone.
        # The correct physical normalisations below (4π for POINT/SPOT, π for
        # AREA) ensure the inter-type balance matches Blender.  ACES handles
        # the remaining dynamic range.
        dist_sq  = np.maximum(dist_sq, MIN_DISTSQ)

        # Lambertian receiver term cos(θ) = dot(surface_normal, surface→light).
        # diff = light→point, so surface→light = -diff/dist.  Points whose normal
        # faces away from the light get 0 → pure black, exactly as Cycles renders
        # them.  This is what turns the flat wash back into a sculpted pool of light.
        dist     = np.sqrt(dist_sq)
        v2p      = diff / dist[:, None]                                  # unit light→pt
        n_dot_l  = np.maximum(0.0, np.einsum('ij,ij->i', normals, -v2p))  # cos θ ∈ [0,1]

        if ltype == 'POINT':
            # Blender: E / (4π r²) × cos(θ)  — isotropic point source
            factor = energy / (PI4 * dist_sq) * n_dot_l

        elif ltype == 'SPOT':
            # Same as POINT, multiplied by smoothstep cone mask
            ldir  = np.array(light['direction'], dtype=np.float64)
            cos_a = v2p @ ldir                                           # N

            half      = light['spot_size'] / 2.0
            cos_outer = math.cos(half)
            cos_inner = math.cos(half * (1.0 - float(light['spot_blend'])))
            denom     = max(cos_inner - cos_outer, 1e-8)
            t_cone    = np.clip((cos_a - cos_outer) / denom, 0.0, 1.0)
            cone      = t_cone * t_cone * (3.0 - 2.0 * t_cone)         # smoothstep
            factor    = energy / (PI4 * dist_sq) * cone * n_dot_l

        elif ltype == 'AREA':
            # Blender Lambertian area emitter: E × cos(θ_source) × cos(θ_recv) / (π r²)
            ldir    = np.array(light['direction'], dtype=np.float64)
            facing  = np.maximum(0.0, v2p @ ldir)                       # cos(θ_source)
            factor  = energy / (PI1 * dist_sq) * facing * n_dot_l

        elif ltype == 'SUN':
            # Directional light — no distance falloff, parallel rays.
            # Blender: irradiance = energy × max(0, dot(normal, -light_dir)).
            # ldir points FROM the lamp TOWARD the scene (Blender convention),
            # so surface→light = -ldir and cos(θ) = dot(normal, -ldir).
            ldir    = np.array(light['direction'], dtype=np.float64)
            cos_inc = np.maximum(0.0, normals @ (-ldir))               # N
            # Scale by K_SUN/K_GLOBAL so the final ×EXPOSURE (=K_GLOBAL×ev) yields
            # energy × cosθ × K_SUN × ev — i.e. the SUN bypasses the lamp K_GLOBAL.
            factor  = energy * cos_inc * (K_SUN / K_GLOBAL)

        else:
            # Fallback: treat as isotropic point source
            factor = energy / (PI4 * dist_sq) * n_dot_l

        # Terrain self-shadow: only compute where the light could reach (cos θ>0),
        # so back-facing points don't pay for a ray-march that won't change them.
        if hpack is not None:
            lit_mask = factor > 0.0
            if lit_mask.any():
                shadow = terrain_shadow(world, light, hpack)
                factor = factor * shadow

        irradiance += factor[:, None] * lcol                            # N×3
        _t(f"  light '{light['name']}' ({ltype})", t0)

    # ── Emission curves (treated as sampled line lights) ─────────────
    # Vectorised batch computation: collect ALL midpoints first, then process
    # N points in tiles so peak memory stays bounded.
    # Uses float32 throughout — precision is sufficient for 1/r² lighting.
    curves = scene.get('emission_curves', [])
    if curves:
        seg_A     = []   # (M, 3) segment start points
        seg_BA    = []   # (M, 3) B-A direction vectors (unnormalised)
        seg_lsq   = []   # (M,)   |B-A|²  (for clamping t to segment)
        seg_w     = []   # (M,)   weight = strength × length
        seg_col   = []   # (M, 3) per-segment colour

        # Segment density is adaptive: one sample per ~SEG_SPACING metres so a long
        # hero curve glows as a CONTINUOUS line (too few segments → visible beads),
        # capped per curve and by a global budget so many-curve scenes (e.g. Grande
        # Motte, 142 curves) stay bounded.  Total weight is preserved on downsample,
        # so total emitted energy — and the K_CURVE calibration — is unchanged.
        SEG_SPACING        = 2.0     # metres between curve samples (smooth glow)
        MAX_SEGS_PER_CURVE = 400
        TOTAL_SEG_BUDGET   = 800     # across all curves combined

        # First pass: desired segment count per curve from its length.
        lengths = []
        for curve in curves:
            pts = np.array(curve['points'], dtype=np.float32)
            lengths.append(float(np.linalg.norm(pts[1:] - pts[:-1], axis=1).sum()))
        want = [min(MAX_SEGS_PER_CURVE, max(8, int(L / SEG_SPACING))) for L in lengths]
        budget_scale = min(1.0, TOTAL_SEG_BUDGET / max(1, sum(want)))

        for curve, n_want in zip(curves, [max(4, int(w * budget_scale)) for w in want]):
            ccol     = np.array(curve['color'],    dtype=np.float32)
            strength = float(curve['strength'])
            pts      = np.array(curve['points'],   dtype=np.float32)
            BA_all   = pts[1:] - pts[:-1]
            dl       = np.linalg.norm(BA_all, axis=1)
            mask     = dl >= 0.01
            if not mask.any():
                continue
            A   = pts[:-1][mask].astype(np.float32)
            BA  = BA_all[mask].astype(np.float32)
            lsq = np.maximum((BA * BA).sum(axis=1), 1e-6).astype(np.float32)
            w   = (strength * dl[mask]).astype(np.float32)
            if len(A) > n_want:
                idx   = np.round(np.linspace(0, len(A) - 1, n_want)).astype(int)
                scale = len(A) / n_want
                A = A[idx]; BA = BA[idx]; lsq = lsq[idx]; w = w[idx] * scale
            seg_A  .append(A)
            seg_BA .append(BA)
            seg_lsq.append(lsq)
            seg_w  .append(w)
            seg_col.append(np.tile(ccol, (len(A), 1)))

        if seg_A:
            all_A   = np.concatenate(seg_A,   axis=0)   # (M, 3) float32
            all_BA  = np.concatenate(seg_BA,  axis=0)   # (M, 3) float32
            all_lsq = np.concatenate(seg_lsq, axis=0)   # (M,)   float32
            all_w   = np.concatenate(seg_w,   axis=0)   # (M,)   float32
            all_col = np.concatenate(seg_col, axis=0)   # (M, 3) float32
            M       = len(all_A)

            # Segment-distance model needs ~10 (B,M) arrays at peak; budget accordingly.
            TILE = max(1, min(200_000, int(128 * 1024**2 / (M * 4))))
            world_f32   = world.astype(np.float32)
            normals_f32 = normals.astype(np.float32)

            n_tiles = (N + TILE - 1) // TILE
            print(f"  Curves: {M} segments, tile={TILE}, {n_tiles} tiles …", flush=True)

            for t, start in enumerate(range(0, N, TILE)):
                end   = min(start + TILE, N)
                batch = world_f32[start:end]          # (B, 3)
                nb    = normals_f32[start:end]         # (B, 3)

                # P − A for each (point, segment)
                pa_x = batch[:, 0, None] - all_A[None, :, 0]   # (B, M)
                pa_y = batch[:, 1, None] - all_A[None, :, 1]
                pa_z = batch[:, 2, None] - all_A[None, :, 2]

                # t_seg = clamp(dot(PA, BA) / |BA|², 0, 1) — closest param on segment
                t_seg = (pa_x * all_BA[None, :, 0]
                       + pa_y * all_BA[None, :, 1]
                       + pa_z * all_BA[None, :, 2]) / all_lsq[None, :]
                np.clip(t_seg, 0.0, 1.0, out=t_seg)

                # d = P − closest = PA − t*BA  →  true perpendicular-to-tube distance
                dx = pa_x - t_seg * all_BA[None, :, 0]   # (B, M)
                dy = pa_y - t_seg * all_BA[None, :, 1]
                dz = pa_z - t_seg * all_BA[None, :, 2]
                dsq = dx*dx + dy*dy + dz*dz               # (B, M)
                np.maximum(dsq, 0.25, out=dsq)

                # Wrap lighting: prevents zero at grazing angles (curve lying on flat terrain
                # gives cos θ ≈ 0 → dark blobs). (dot + 0.5)/1.5 maps grazing→0.33, face-on→1.
                dist = np.sqrt(dsq)
                ndl  = -(nb[:, 0, None]*dx + nb[:, 1, None]*dy + nb[:, 2, None]*dz) / dist
                ndl  = np.maximum(0.0, (ndl + 0.5) / 1.5)  # (B, M) wrapped cos θ

                contribs = all_w[None, :] / dsq * ndl    # (B, M)  w · wrapped_cos θ / r²
                irradiance[start:end] += (contribs @ all_col).astype(np.float64) * (K_CURVE * CURVE_EXPOSURE / K_GLOBAL)

                if (t + 1) % max(1, n_tiles // 10) == 0:
                    pct = (t + 1) / n_tiles * 100
                    print(f"    {pct:.0f}%", flush=True)

    # ── Modulate satellite colour by irradiance ───────────────────────
    lit = sat_lin * irradiance * EXPOSURE                              # N×3
    _t("irradiance × sat", t0)

    # ── ACES tone map → linear → sRGB → uint8 ────────────────────────
    out_srgb  = linear_to_srgb(aces(lit))
    _t("ACES + linear_to_srgb", t0)
    out_uint8 = np.clip(out_srgb * 255.0 + 0.5, 0, 255).astype(np.uint8)
    _t("→ uint8", t0)

    return out_uint8


# ── Entry point ─────────────────────────────────────────────────────────

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python bake_lighting.py <input.ply> <scene_lights.json> [output.ply]")
        sys.exit(1)

    ply_in  = sys.argv[1]
    json_in = sys.argv[2]
    if len(sys.argv) > 3:
        ply_out = sys.argv[3]
    else:
        base, ext = os.path.splitext(ply_in)
        ply_out = base + '-lit' + ext

    t_main = time.time()

    print(f"Reading  {ply_in} …", flush=True)
    xyz, rgb = read_ply(ply_in)
    print(f"  {len(xyz):,} points  ({time.time()-t_main:.1f}s)", flush=True)

    print(f"Reading  {json_in}", flush=True)
    with open(json_in, encoding='utf-8') as f:
        scene = json.load(f)
    n_lights = len(scene['lights'])
    n_curves = len(scene.get('emission_curves', []))
    print(f"  {n_lights} light(s), {n_curves} emission curve(s)", flush=True)

    print("Computing lighting …", flush=True)
    rgb_lit = apply_lighting(xyz, rgb, scene)

    print(f"Writing  {ply_out} …", flush=True)
    t_w = time.time()
    write_ply(ply_out, xyz, rgb_lit)
    print(f"Done.  Total: {time.time()-t_main:.1f}s", flush=True)
