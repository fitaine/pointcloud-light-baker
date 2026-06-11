"""
gs_capture.py — GS training data capture
Injected into Blender at runtime by launcher.bat via --python flag.
The .blend file is NEVER modified or saved.

Output:
  <output_dir>/images/cam_scene.webp     frame 0 — scene camera from .blend
                                        open this within ~20s to verify GN/lighting
  <output_dir>/images/cam_EL_NNN.jpg   109 orbital frames
  <output_dir>/transforms.json          Nerfstudio / instant-ngp format

Usage (via launcher.bat — do not run directly):
  blender --background scene.blend --python gs_capture.py -- <output_dir> <scene_name>
"""

import bpy
import math
import json
import sys
import os
from mathutils import Vector

# ── Parse arguments ───────────────────────────────────────────────────────────
argv        = sys.argv
sep         = argv.index("--") + 1
output_dir  = argv[sep]
scene_name  = argv[sep + 1]
images_dir  = os.path.join(output_dir, "images")
os.makedirs(images_dir, exist_ok=True)

print(f"\n[GS Capture] Scene    : {scene_name}")
print(f"[GS Capture] Output   : {output_dir}")

# ── Constants — tune these without touching the .blend ────────────────────────
# Square frames: an orbit has no preferred orientation, so portrait scenes
# (Dibona) and landscape scenes (Chamechaude) are covered identically.
# Resolution > samples for reprojection: each pixel colors a few points, and
# multi-view averaging acts as a second denoiser on top of Blender's.
RENDER_WIDTH     = 4096
RENDER_HEIGHT    = 4096
RENDER_SAMPLES   = 32
RENDER_FORMAT    = "WEBP"       # WEBP (~30% smaller than JPEG), JPEG, or PNG (lossless)
RENDER_QUALITY   = 90           # WEBP/JPEG quality (90 is visually lossless)

ELEVATIONS       = [8, 20, 45, 70]  # degrees above horizon — low ring catches cliff faces
STEPS_PER_RING   = 36            # cameras per ring (every 10°)
TARGET_NAME      = "GS_TARGET"  # exact name of the orbit-centre Empty in .blend

# Orbit camera intrinsics are FIXED — never copied from the scene camera.
# The artistic camera's lens is irrelevant to the orbit's job (coverage):
# Dibona's 142 mm telephoto framed ~730 m of a 3400 m scene and most of the
# terrain was never seen by any orbit frame. The orbit radius is derived
# from this FOV so the scene's bounding sphere always fits the frame.
ORBIT_LENS       = 35.0          # mm
ORBIT_SENSOR     = 36.0          # mm (square sensor for square frames)
ORBIT_MARGIN     = 1.1           # bounding sphere fills 1/1.1 of the frame

# ── Render settings ───────────────────────────────────────────────────────────
scene = bpy.context.scene
scene.render.engine                     = 'CYCLES'
scene.render.resolution_x               = RENDER_WIDTH
scene.render.resolution_y               = RENDER_HEIGHT
scene.render.resolution_percentage      = 100
scene.render.image_settings.file_format = RENDER_FORMAT
scene.render.image_settings.quality     = RENDER_QUALITY
scene.cycles.samples                    = RENDER_SAMPLES
scene.cycles.use_denoising              = True
# Color management left as-is: Filmic/AgX + artist exposure baked into renders.

# ── Force full depsgraph evaluation ──────────────────────────────────────────
# CRITICAL: in --background mode, Geometry Nodes do NOT evaluate until
# frame_set() forces a full depsgraph update.  Without this, obj.bound_box
# returns zeros for GN meshes → diagonal = 0 → orbit_radius = 0
# → camera placed inside the terrain → black frames.
print("[GS Capture] Forcing depsgraph evaluation (GN bounding boxes) …")
bpy.context.scene.frame_set(bpy.context.scene.frame_current)

# ── Export cloud DEM for automatic frame alignment ───────────────────────────
# The point cloud in the .blend may be recentered / offset relative to the raw
# LiDAR tiles (lidar_pipeline recenters PLYs; artists move objects). The render
# cameras live in the .blend's world frame, so reprojection scripts need the
# transform between that frame and the raw-tile (Lambert-93) frame.
# Rather than trusting any per-scene constant, we export a coarse elevation
# grid (max-Z per 10 m cell) of the loaded cloud, in the SAME world frame as
# the cameras. reproject_copc.py builds the same grid from the raw tiles and
# cross-correlates — alignment becomes automatic and verifiable per scene.
import numpy as np
DEM_CELL = 10.0
_pc_pts = []
for _o in bpy.context.scene.objects:
    if _o.type == 'MESH' and len(_o.data.polygons) == 0 and len(_o.data.vertices) > 0:
        _n = len(_o.data.vertices)
        _co = np.empty(_n * 3, dtype=np.float32)
        _o.data.vertices.foreach_get('co', _co)
        _co = _co.reshape(-1, 3)
        _mw = np.array(_o.matrix_world)
        if not np.allclose(_mw, np.eye(4), atol=1e-6):
            _co = _co @ _mw[:3, :3].T + _mw[:3, 3]   # apply object transform
        _pc_pts.append(_co)
        print(f"[GS Capture] cloud DEM: '{_o.name}' {_n:,} verts")
if _pc_pts:
    _co = np.concatenate(_pc_pts)
    _mn = _co[:, :2].min(0)
    _ij = ((_co[:, :2] - _mn) / DEM_CELL).astype(np.int32)
    _W, _H = int(_ij[:, 0].max()) + 1, int(_ij[:, 1].max()) + 1
    _dem = np.full(_W * _H, -1e9, dtype=np.float32)
    np.maximum.at(_dem, _ij[:, 0] * _H + _ij[:, 1], _co[:, 2])
    np.save(os.path.join(output_dir, "cloud_dem.npy"), _dem.reshape(_W, _H))
    with open(os.path.join(output_dir, "cloud_dem_meta.json"), "w") as _f:
        json.dump({"min_xy": _mn.tolist(), "cell": DEM_CELL,
                   "shape": [_W, _H]}, _f)
    print(f"[GS Capture] cloud DEM {_W}x{_H} @ {DEM_CELL}m → cloud_dem.npy")
    del _pc_pts, _co
else:
    print("[GS Capture] WARNING: no point-cloud object found — no cloud_dem.npy "
          "(reprojection will need a manual origin)")

# ── Resolve orbit target ─────────────────────────────────────────────────────
# Preferred : Empty named exactly GS_TARGET in the .blend.
# Fallback  : bounding-box centre of renderable meshes.
# NOT falling back to arbitrary empties — they are often light/camera aim helpers.
target_obj = bpy.data.objects.get(TARGET_NAME)
if target_obj is not None:
    target = target_obj.location.copy()
    print(f"[GS Capture] GS_TARGET : {target.x:.1f}, {target.y:.1f}, {target.z:.1f}")
else:
    print(f"\n[GS Capture] WARNING: No object named '{TARGET_NAME}' found.")
    print(f"[GS Capture] Computing orbit centre from mesh bounding box …")
    INF = float('inf')
    _mn = Vector(( INF,  INF,  INF))
    _mx = Vector((-INF, -INF, -INF))
    _found = False
    for _o in bpy.context.scene.objects:
        if _o.type != 'MESH' or _o.hide_render:
            continue
        for _c in _o.bound_box:
            _w = _o.matrix_world @ Vector(_c)
            _mn.x = min(_mn.x, _w.x);  _mx.x = max(_mx.x, _w.x)
            _mn.y = min(_mn.y, _w.y);  _mx.y = max(_mx.y, _w.y)
            _mn.z = min(_mn.z, _w.z);  _mx.z = max(_mx.z, _w.z)
            _found = True
    if _found:
        target = (_mn + _mx) / 2.0
        print(f"[GS Capture] Orbit centre (mesh bbox): "
              f"{target.x:.1f}, {target.y:.1f}, {target.z:.1f}")
        print(f"[GS Capture] Tip: add an Empty at this position, name it {TARGET_NAME},")
        print(f"[GS Capture]      to control the orbit centre precisely.\n")
    else:
        print(f"[GS Capture] ERROR: No renderable meshes and no '{TARGET_NAME}' empty.")
        print(f"[GS Capture] Add an Empty (Shift+A → Empty → Plain Axes) at the scene centre,")
        print(f"[GS Capture] name it {TARGET_NAME}, and re-run.")
        print(f"[GS Capture] The .blend file has not been modified.\n")
        sys.exit(1)

# ── Scene bounding box → orbit radius ────────────────────────────────────────
def get_scene_bounds():
    """Return (min_co, max_co) world-space bbox of all renderable MESH objects."""
    INF = float('inf')
    mn = Vector(( INF,  INF,  INF))
    mx = Vector((-INF, -INF, -INF))
    found = False
    for obj in bpy.context.scene.objects:
        if obj.type != 'MESH' or obj.hide_render:
            continue
        for corner in obj.bound_box:
            wco = obj.matrix_world @ Vector(corner)
            mn.x = min(mn.x, wco.x);  mx.x = max(mx.x, wco.x)
            mn.y = min(mn.y, wco.y);  mx.y = max(mx.y, wco.y)
            mn.z = min(mn.z, wco.z);  mx.z = max(mx.z, wco.z)
            found = True
    if not found:
        print("[GS Capture] WARNING: no renderable MESH objects — using target ± 100m")
        mn = target - Vector((100, 100, 100))
        mx = target + Vector((100, 100, 100))
    return mn, mx

min_co, max_co = get_scene_bounds()
diagonal     = (max_co - min_co).length
# Radius FROM the FOV: place the camera exactly far enough that the scene's
# bounding sphere (radius diagonal/2) fits inside the field of view, with
# ORBIT_MARGIN slack. No magic per-scene factor — universal by construction.
_half_fov    = math.atan(ORBIT_SENSOR / (2.0 * ORBIT_LENS))
orbit_radius = (diagonal / 2.0) / math.sin(_half_fov) * ORBIT_MARGIN

print(f"[GS Capture] Bounds   : ({min_co.x:.0f},{min_co.y:.0f},{min_co.z:.0f})"
      f" → ({max_co.x:.0f},{max_co.y:.0f},{max_co.z:.0f})")
print(f"[GS Capture] Diagonal : {diagonal:.1f}  →  orbit radius {orbit_radius:.1f}")

# Write key values to a file — Blender's \r progress lines overwrite print() in the log
_dbg = os.path.join(output_dir, "debug.txt")
with open(_dbg, "w") as _f:
    _f.write(f"target       : {target.x:.2f}, {target.y:.2f}, {target.z:.2f}\n")
    _f.write(f"min_co       : {min_co.x:.2f}, {min_co.y:.2f}, {min_co.z:.2f}\n")
    _f.write(f"max_co       : {max_co.x:.2f}, {max_co.y:.2f}, {max_co.z:.2f}\n")
    _f.write(f"diagonal     : {diagonal:.2f}\n")
    _f.write(f"orbit_radius : {orbit_radius:.2f}\n")

# ── Frame 0: render from the .blend's own scene camera ───────────────────────
# This is the FIRST file written (~20 s).  Open it immediately to verify that
# Geometry Nodes, lighting, and materials are working correctly.
# If cam_scene.webp is black → the problem is in the scene setup, not the orbit.
# If cam_scene.webp looks good → the orbital frames will too.
# The frame is also included in transforms.json as a real training viewpoint.
frames_data = []
_scene_cam  = scene.camera

if _scene_cam is not None:
    _val_path = os.path.join(images_dir, "cam_scene.webp")
    if os.path.exists(_val_path):
        print(f"\n[GS Capture] Frame 0 — cam_scene.webp exists, skipping (resume)")
    else:
        print(f"\n[GS Capture] Frame 0 — scene camera '{_scene_cam.name}' …")
        bpy.context.scene.frame_set(bpy.context.scene.frame_current)
        scene.render.filepath = _val_path
        bpy.ops.render.render(write_still=True)
        print(f"[GS Capture] ✓ cam_scene.webp  ← open this to verify rendering\n")
    # NOT added to transforms.json: this frame uses the scene camera's own
    # lens (can be a 142 mm telephoto), which doesn't match the shared orbit
    # intrinsics. It exists purely as a visual verification image.
else:
    print("[GS Capture] No scene camera in .blend — skipping validation frame.\n")

# ── Build orbital camera list ─────────────────────────────────────────────────
# cam_top (straight down, elevation 89.9°) is first so you immediately see
# whether the orbit centre and radius are correct.
camera_specs = [(89.9, 0.0, "cam_top")]
for elev in ELEVATIONS:
    for i in range(STEPS_PER_RING):
        az = (360.0 / STEPS_PER_RING) * i
        camera_specs.append((elev, az, f"cam_{elev:02d}_{i:03d}"))

total = len(camera_specs)
print(f"[GS Capture] {total} orbital cameras  ({len(ELEVATIONS)} rings × {STEPS_PER_RING} + 1 top)")

# ── Create temporary orbital camera ──────────────────────────────────────────
# Copy scene camera focal length / sensor so all frames share the same intrinsics
# and camera_angle_x is consistent across the full transforms.json.
# clip_start / clip_end are set explicitly — defaults (0.1 m / 100 m) would make
# the terrain invisible from orbit (radius typically > 1 000 m).
cam_data = bpy.data.cameras.new("GS_Cam")
cam_data.lens         = ORBIT_LENS
cam_data.sensor_width = ORBIT_SENSOR
cam_data.sensor_fit   = 'HORIZONTAL'
print(f"[GS Capture] GS_Cam: lens={ORBIT_LENS:.1f}mm  sensor={ORBIT_SENSOR:.1f}mm "
      f"(fixed — scene camera intrinsics are never copied)")
cam_data.clip_start = 0.001        # effectively zero
cam_data.clip_end   = 1_000_000    # 1 000 km — effectively infinity
print(f"[GS Capture] GS_Cam: clip {cam_data.clip_start} … {cam_data.clip_end} m")
with open(_dbg, "a") as _f:
    _f.write(f"GS_Cam clip  : {cam_data.clip_start} … {cam_data.clip_end}\n")
    if _scene_cam and _scene_cam.data:
        _f.write(f"scene_cam clip: {_scene_cam.data.clip_start} … {_scene_cam.data.clip_end}\n")
        _f.write(f"scene_cam lens: {_scene_cam.data.lens} mm\n")
cam_obj = bpy.data.objects.new("GS_Cam", cam_data)
bpy.context.scene.collection.objects.link(cam_obj)
scene.camera = cam_obj

# ── Orbital render loop ───────────────────────────────────────────────────────
for idx, (elev, azimuth, name) in enumerate(camera_specs):
    elev_rad = math.radians(elev)
    az_rad   = math.radians(azimuth)

    # Spherical → Cartesian, centred on orbit target
    x = target.x + orbit_radius * math.cos(elev_rad) * math.cos(az_rad)
    y = target.y + orbit_radius * math.cos(elev_rad) * math.sin(az_rad)
    z = target.z + orbit_radius * math.sin(elev_rad)

    cam_obj.location = Vector((x, y, z))

    # Point -Z toward target, Y up
    direction              = target - cam_obj.location
    rot_quat               = direction.to_track_quat('-Z', 'Y')
    cam_obj.rotation_euler = rot_quat.to_euler()

    # frame_set() forces a full depsgraph update including Geometry Nodes.
    bpy.context.scene.frame_set(bpy.context.scene.frame_current)

    # Camera transforms are deterministic (computed from constants), so a
    # frame already on disk can be skipped on relaunch — render resume.
    filepath = os.path.join(images_dir, f"{name}.webp")
    if os.path.exists(filepath):
        print(f"[GS Capture] {idx+1:3d}/{total}  {name}  exists, skipping (resume)")
    else:
        scene.render.filepath = filepath
        bpy.ops.render.render(write_still=True)
        print(f"[GS Capture] {idx+1:3d}/{total}  {name}  ({x:.0f}, {y:.0f}, {z:.0f})")

    # Camera-to-world matrix — Blender convention (X-right, Y-up, -Z fwd)
    # matches NeRF/Nerfstudio "blender" convention directly.
    c2w = [list(row) for row in cam_obj.matrix_world]
    frames_data.append({
        "file_path":        f"images/{name}.webp",
        "transform_matrix": c2w,
    })

# ── Save FOV before removing camera ──────────────────────────────────────────
# Must be before bpy.data.cameras.remove() — reading cam_data after removal crashes.
camera_angle_x = 2.0 * math.atan(cam_data.sensor_width / (2.0 * cam_data.lens))

# ── Cleanup ───────────────────────────────────────────────────────────────────
bpy.data.objects.remove(cam_obj, do_unlink=True)
bpy.data.cameras.remove(cam_data)
print("[GS Capture] Temporary camera removed — .blend unmodified")

# ── Export transforms.json ────────────────────────────────────────────────────
# camera_angle_x : horizontal FOV in radians (same for all frames — intrinsics copied)
# transform_matrix: 4×4 c2w, Blender convention
#
# If Nerfstudio shows upside-down / mirrored results, uncomment to flip Y+Z:
#   for f in frames_data:
#       m = f["transform_matrix"]
#       for row in m: row[1] *= -1; row[2] *= -1

# fl_x / fl_y in pixels — required by Nerfstudio dataparser
# (camera_angle_x kept for compatibility with instant-ngp / other tools)
fl_x = 0.5 * RENDER_WIDTH / math.tan(0.5 * camera_angle_x)

transforms = {
    "camera_angle_x": camera_angle_x,
    "fl_x":   fl_x,
    "fl_y":   fl_x,          # square pixels
    "cx":     RENDER_WIDTH  / 2.0,
    "cy":     RENDER_HEIGHT / 2.0,
    "w":      RENDER_WIDTH,
    "h":      RENDER_HEIGHT,
    "camera_model": "OPENCV",
    "frames": frames_data,
}
transforms_path = os.path.join(output_dir, "transforms.json")
with open(transforms_path, "w") as f:
    json.dump(transforms, f, indent=2)

# ── Summary ───────────────────────────────────────────────────────────────────
total_frames = len(frames_data)
print(f"\n[GS Capture] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
print(f"[GS Capture]  {total_frames} frames → {images_dir}")
print(f"[GS Capture]  transforms.json → {transforms_path}")
print(f"[GS Capture]  camera_angle_x = {camera_angle_x:.4f} rad "
      f"({math.degrees(camera_angle_x):.1f}°)")
print(f"[GS Capture] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
print(f"[GS Capture]  Train with Nerfstudio:")
print(f"    ns-train splatfacto --data \"{output_dir}\"")
print(f"[GS Capture] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
print(f"[GS Capture]  .blend file was NOT saved or modified.\n")
