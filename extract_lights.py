"""
extract_lights.py — Run inside Blender (Script Editor or via MCP addon)
Reads all lights, emission curves, world ambient and PC object transform
from the currently open .blend, writes scene_lights.json next to the .blend.

Works for any LiDAR scene: no hardcoded names.
"""

import bpy
import json
import math
import mathutils
import os

# ── CONFIG ─────────────────────────────────────────────────────────────
# Leave empty → writes next to the .blend file
OUTPUT_PATH = ""

# Number of sample points along each bezier spline (distributed across segments)
CURVE_SAMPLES = 200

# ── Find PC objects ─────────────────────────────────────────────────────
# Heuristic: all meshes with 0 polygons are point clouds.
# Multiple objects are supported (fg + bg splits, etc.) — their world
# bounding boxes are merged into one combined centre.
pc_objs = []
for obj in bpy.data.objects:
    if obj.type == 'MESH' and len(obj.data.polygons) == 0 and len(obj.data.vertices) > 0:
        pc_objs.append(obj)

if not pc_objs:
    raise RuntimeError("No point cloud found (expected a MESH with 0 faces).")

for o in pc_objs:
    print(f"PC object: {o.name}  ({len(o.data.vertices):,} vertices)")

# ── PC transform ────────────────────────────────────────────────────────
# Merge the world-space bounding boxes of all PC objects into one AABB,
# then take its centre.  This is the world point that corresponds to the
# bounding-box centre of the merged web PLY (which is always at local 0,0,0
# after pipeline recentering).
all_bb_world = []
for obj in pc_objs:
    mw = obj.matrix_world
    all_bb_world += [mw @ mathutils.Vector(c) for c in obj.bound_box]

xs = [v.x for v in all_bb_world]
ys = [v.y for v in all_bb_world]
zs = [v.z for v in all_bb_world]
cx = (min(xs) + max(xs)) / 2
cy = (min(ys) + max(ys)) / 2
cz = (min(zs) + max(zs)) / 2

# Use the largest object's matrix for reference (kept for debugging)
pc_obj = max(pc_objs, key=lambda o: len(o.data.vertices))
mw = pc_obj.matrix_world

pc_transform = {
    "name":            " + ".join(o.name for o in pc_objs),
    "objects":         [o.name for o in pc_objs],
    "matrix":          [list(row) for row in mw],   # largest object, kept for reference
    "ply_world_center": [cx, cy, cz],               # merged BB centre → web PLY (0,0,0)
}

# ── World ambient ───────────────────────────────────────────────────────
world = bpy.context.scene.world
world_ambient = [0.0, 0.0, 0.0]
if world and world.use_nodes:
    for node in world.node_tree.nodes:
        if node.type == 'BACKGROUND':
            c = node.inputs['Color'].default_value
            s = node.inputs['Strength'].default_value
            world_ambient = [c[0] * s, c[1] * s, c[2] * s]
            break
elif world:
    world_ambient = list(world.color[:3])

# ── Render visibility ─────────────────────────────────────────────────────
# Only export lights/curves that actually contribute to the render.  A light
# with hide_render=True (the camera/render-disable toggle) or sitting in a
# view-layer collection that's excluded does NOT light the Cycles render, so the
# bake must ignore it too — otherwise disabled lights reappear as phantom spots.
_excluded_names = set()
def _walk_lc(lc):
    if lc.exclude:
        for ob in lc.collection.all_objects:
            _excluded_names.add(ob.name)
    for c in lc.children:
        _walk_lc(c)
_walk_lc(bpy.context.view_layer.layer_collection)

def renders(obj):
    return (not obj.hide_render) and (obj.name not in _excluded_names)

# ── Lights ──────────────────────────────────────────────────────────────
lights = []
for obj in bpy.data.objects:
    if obj.type != 'LIGHT':
        continue
    if not renders(obj):
        print(f"Skipping light '{obj.name}' (disabled in render)")
        continue
    L = obj.data
    pos = list(obj.matrix_world.translation)

    entry = {
        "name":     obj.name,
        "type":     L.type,
        "energy":   L.energy,
        "color":    [L.color.r, L.color.g, L.color.b],
        "position": pos,
    }

    if L.type in ('SPOT', 'AREA', 'SUN'):
        d = obj.matrix_world.to_3x3() @ mathutils.Vector((0, 0, -1))
        d.normalize()
        entry["direction"] = list(d)

    if L.type == 'SPOT':
        entry["spot_size"]  = L.spot_size    # full cone angle, radians
        entry["spot_blend"] = L.spot_blend

    if L.type == 'AREA':
        entry["size"] = L.size

    lights.append(entry)

# ── Emission curves ─────────────────────────────────────────────────────
def eval_cubic_bezier(p0, hr0, hl1, p1, t):
    mt = 1.0 - t
    return mt**3 * p0 + 3*mt**2*t * hr0 + 3*mt*t**2 * hl1 + t**3 * p1

# Blackbody (Kelvin) → linear sRGB.  Glowing curves are usually coloured by a
# Blackbody node (e.g. lava/embers), whose colour lives in the node, NOT in the
# Emission Color socket default (which stays white).  Reading only the default
# would wash a deep-orange ember curve out to white, so evaluate the source node.
def blackbody_to_linear(temp):
    t = max(1000.0, min(40000.0, temp)) / 100.0
    # Tanner Helland approximation → sRGB 0..255
    r = 255.0 if t <= 66 else 329.698727446 * (t - 60) ** -0.1332047592
    if t <= 66:
        g = 99.4708025861 * math.log(t) - 161.1195681661
    else:
        g = 288.1221695283 * (t - 60) ** -0.0755148492
    if t >= 66:   b = 255.0
    elif t <= 19: b = 0.0
    else:         b = 138.5177312231 * math.log(t - 10) - 305.0447927307
    srgb = [max(0.0, min(255.0, v)) / 255.0 for v in (r, g, b)]
    # sRGB → linear (emission colour is scene-linear)
    return [(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4) for c in srgb]

def emission_color_of(socket):
    """Resolve an Emission Color socket to a linear RGB, following common nodes."""
    if socket.is_linked:
        src = socket.links[0].from_node
        if src.type == 'BLACKBODY':
            return blackbody_to_linear(src.inputs['Temperature'].default_value)
        if src.type == 'RGB':
            v = src.outputs[0].default_value
            return [v[0], v[1], v[2]]
        if src.type in ('TEX_GRADIENT', 'VALTORGB'):   # ColorRamp etc. — use last stop
            try:
                v = src.color_ramp.elements[-1].color
                return [v[0], v[1], v[2]]
            except Exception:
                pass
        print(f"  (emission colour driven by unhandled {src.type}; using socket default)")
    c = socket.default_value
    return [c[0], c[1], c[2]]

emission_curves = []
for obj in bpy.data.objects:
    if obj.type != 'CURVE':
        continue
    if not renders(obj):
        continue

    # Find emission material
    em_color    = None
    em_strength = None
    for mat in obj.data.materials:
        if not mat or not mat.use_nodes:
            continue
        for node in mat.node_tree.nodes:
            if node.type == 'EMISSION':
                em_color    = emission_color_of(node.inputs['Color'])
                em_strength = node.inputs['Strength'].default_value
                break
        if em_color:
            break

    if em_color is None:
        continue   # not an emission object

    mw_c = obj.matrix_world
    sampled = []

    for spline in obj.data.splines:
        if spline.type != 'BEZIER' or len(spline.bezier_points) < 2:
            continue
        bpts = spline.bezier_points
        n_segs = len(bpts) - 1
        sps = max(4, CURVE_SAMPLES // n_segs)   # samples per segment

        for i in range(n_segs):
            p0  = mathutils.Vector(bpts[i].co)
            hr0 = mathutils.Vector(bpts[i].handle_right)
            hl1 = mathutils.Vector(bpts[i + 1].handle_left)
            p1  = mathutils.Vector(bpts[i + 1].co)
            for j in range(sps):
                t  = j / sps
                lp = eval_cubic_bezier(p0, hr0, hl1, p1, t)
                wp = mw_c @ lp
                sampled.append([wp.x, wp.y, wp.z])

        # Final point
        last = mw_c @ mathutils.Vector(bpts[-1].co)
        sampled.append([last.x, last.y, last.z])

    if sampled:
        emission_curves.append({
            "name":     obj.name,
            "color":    em_color,
            "strength": em_strength,
            "points":   sampled,
        })

# ── Render colour management ────────────────────────────────────────────
# Capture the scene Exposure (EV) so bake_lighting.py can apply the same
# multiplier: K = 2 ** render_exposure_ev.
# Everything else (view transform, look, gamma) is handled by ACES, which
# approximates Blender Filmic Medium Contrast within ~5% across the full range.
vs = bpy.context.scene.view_settings
render_exposure_ev = float(vs.exposure)
print(f"Render exposure: {render_exposure_ev:+.2f} EV  →  K = {2**render_exposure_ev:.3f}×")

# ── Assemble & write ────────────────────────────────────────────────────
data = {
    "pc_transform":       pc_transform,
    "world_ambient":      world_ambient,
    "render_exposure_ev": render_exposure_ev,
    "lights":             lights,
    "emission_curves":    emission_curves,
}

if not OUTPUT_PATH:
    blend_path = bpy.data.filepath
    out_dir = os.path.dirname(blend_path) if blend_path else os.path.expanduser("~")
    out_path = os.path.join(out_dir, "scene_lights.json")
else:
    out_path = OUTPUT_PATH

with open(out_path, 'w', encoding='utf-8') as f:
    json.dump(data, f, indent=2)

print(f"\nWritten: {out_path}")
print(f"  Lights          : {len(lights)}")
print(f"  Emission curves : {len(emission_curves)}")
print(f"  World ambient   : {world_ambient}")
print(f"  Render exposure : {render_exposure_ev:+.2f} EV  (K = {2**render_exposure_ev:.3f}×)")
