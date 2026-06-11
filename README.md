# pointcloud-light-baker

A pipeline for displaying LiDAR point clouds with Blender artistic lighting in a web viewer, as a 3D companion to 2D photographic renders.

The goal: open a `.blend` file, run two scripts, get a lit point cloud that matches the Blender Cycles render — for every scene, with no per-scene tuning.

---

## How it works

IGN LiDAR HD tiles contain geometry only (XYZ + intensity, no colour). A separate IGN orthophoto (BDORTHO, fetched at 0.20 m/px via WMS) is used to colorize the points via PDAL — this is handled by `lidar_pipeline.py` as a prerequisite step. Blender then provides dramatic artistic lighting: area lights, spotlights, moonlight, glowing emission curves traced along the terrain. Instead of re-implementing Blender's physics in Python (which drifts from the real render scene by scene), this pipeline uses the actual Cycles renders themselves:

1. **Orbit renders** — `gs_capture.py` renders 146 frames from cameras orbiting the scene at four elevation rings (8°, 20°, 45°, 70°) + overhead, at 4K. The `.blend` file is never modified.
2. **Reprojection** — `reproject_lighting.py` / `reproject_copc.py` projects each point of the cloud into every render it is visible from, resolves occlusion with a per-camera depth buffer, and averages the Cycles pixel colors. The point's color is literally a sample of the render.
3. **Web display** — the lit cloud is converted to COPC format for streaming display in a Potree viewer (LOD, 60M+ pts), or loaded directly in the Three.js prototype viewer (< 15M pts).

Because the colors come from real Cycles renders, **any light type, material, or color management setting works automatically** — there are no physics constants to calibrate, no per-scene tuning variables.

---

## Quick start — one drop

Drag a `.blend` onto **`RUN PIPELINE.bat`**. It runs the whole chain and prints progress in the terminal:

1. **Render** — 146 orbit frames + cameras + cloud DEM (Blender, background)
2. **Reproject** — auto frame alignment, tile selection by blend footprint, color the raw IGN tiles, satellite fallback
3. **Relight** — albedo × light separation (texture from the 0.20 m ortho, lighting from the renders); the ortho resolution is verified and re-fetched at IGN native automatically
4. **Merge** — untwine → `<scene>-detail.copc.laz` (add `--both` for the plain reprojection too)
5. **Register** — scene added to the Potree viewer menu, cache version bumped

**Every stage is resumable**: close the window or kill the process at any point, drop the same `.blend` again, and it continues — finished frames and tiles are skipped, half-written files are never trusted (temp-write + rename).

Scene folder conventions (same as `lidar_pipeline.py`):

```
<blend_dir>/LIDAR/LIDAR Bases IGN/     raw IGN .copc.laz tiles
<blend_dir>/LIDAR/output/*_raster.tif  BDORTHO ortho (albedo fallback)
```

The sections below document each stage individually.

---

## Pipeline

```
.blend  (read-only — never modified or saved)
    │
    └─ gs_capture.py  →  images/ (146 × 4K WebP)  +  transforms.json
                                    │
                     reproject_lighting.py          reproject_copc.py
                     (web PLY, < 15M pts)           (raw IGN COPC tiles, full HD)
                                    │                       │
                             lit web PLY              lit LAZ tiles
                                    │                       │
                             prototype/              untwine → COPC
                             Three.js viewer         potree/ Potree viewer
                             (preview, fast)         (full quality, LOD)
```

### Requirements

```
pip install numpy scipy pillow laspy lazrs
```

PDAL (via QGIS) for COPC conversion. No Blender install needed to run the reprojection.

---

## Step 1 — Orbit renders (`gs_capture.py`)

Run via `gs-capture/launcher.bat` — drag your `.blend` onto it, or run from command line:

```bat
blender --background scene.blend --python gs_capture.py -- <output_dir> <scene_name>
```

Renders 146 frames (square 4K, 128 samples, WebP) and writes `transforms.json` with exact camera matrices plus `cloud_dem.npy` for automatic frame alignment. Timing: ~1–2 min/frame → ~3–4 h total. Per-frame resume: already-rendered frames are skipped on relaunch.

**Tunable constants** (top of `gs_capture.py`, no .blend modification needed):

| Constant | Default | Notes |
|---|---|---|
| `RENDER_WIDTH/HEIGHT` | 4096 × 4096 | square — orbits have no preferred orientation |
| `RENDER_SAMPLES` | 128 | renders are fast without denoising; multi-view averaging adds more |
| `RENDER_FORMAT` | WEBP | smaller than JPEG, visually lossless |
| `ELEVATIONS` | [8, 20, 45, 70] | rings in degrees — 8° catches low cliff faces |
| `STEPS_PER_RING` | 36 | cameras per ring (every 10°) |
| `ORBIT_LENS / ORBIT_SENSOR` | 35 / 36 mm | fixed — never copied from the scene camera; radius = (diag/2)/sin(FOV/2) × 1.1 |
| `TARGET_NAME` | `OrbitTarget` | Empty controlling orbit centre (legacy `GS_TARGET` accepted) |

The compositor is bypassed at render time (glare/denoise/grade nodes stay untouched for the 2D renders); colour management still applies.

---

## Step 2a — Reproject onto web PLY (`reproject_lighting.py`)

For preview and Three.js viewer (< 15M pts):

```bash
python reproject_lighting.py <input.ply> <capture_dir> <output-lit.ply>
```

- `<input.ply>` — point cloud in Blender world frame (stride-export from Blender via MCP, or a processed web PLY)
- `<capture_dir>` — folder containing `transforms.json` + `images/` from gs_capture.py

Timing: ~2 min for 7M pts × 146 cameras. Coverage: ~97%.

---

## Step 2b — Reproject onto raw IGN COPC tiles (`reproject_copc.py`)

Full quality path — colors the original unprocessed IGN HD tiles (30–60M pts per tile) directly:

```bash
python reproject_copc.py tiles <capture_dir> <tiles_dir> <out_dir>
```

- `tiles` — build per-camera depth buffers from the raw tiles themselves (recommended; a PLY path is also accepted but must be in the Blender world frame)
- `<tiles_dir>` — folder of raw IGN `.copc.laz` tiles

**Frame alignment is automatic.** The cloud loaded in the .blend is generally *not* in the `lambert − origin` frame (it may be recentered, cropped, or a different processing run). `gs_capture.py` exports `cloud_dem.npy` — a 10 m elevation grid of the loaded cloud, in the same world frame as the render cameras. `reproject_copc.py` builds the same grid from the raw tiles and cross-correlates to recover the exact offset (verified at 0.7 m residual on Chamechaude). If the best fit is worse than 2 m the script **aborts** rather than producing a silently misaligned cloud. `--origin X,Y,Z` remains as a manual override.

Phase A builds depth buffers (one per camera) — ~9 min for 59M pts × 146 cameras.  
Phase B colors each tile — ~15 min per tile.

---

## Step 3 — Convert to COPC for Potree

```bat
untwine -i <lit_tiles_dir> -o potree\pointclouds\<name>.copc.laz
```

untwine (bundled with QGIS) processes tile-by-tile on disk (~200 MB RAM at any scale). PDAL `writers.copc` loads everything in RAM — fine to ~50M pts, fails at 433M. `run_pipeline.py` does this automatically with a merge manifest (a changed tile set always triggers a remerge).

---

## Step 4 — View

**Three.js prototype** (< 15M pts, instant):
```bash
cd prototype && python -m http.server 8080
# http://localhost:8080/?scene=<scene-id>
```

**Potree** (60M+ pts, LOD streaming):
```bash
cd potree && python server.py 8081   # range-request server required
# http://localhost:8081/?scene=<scene-id>
```

---

## Coordinate alignment

The reprojection requires the point cloud and the Blender renders to share the same coordinate frame. **Never assume the mapping — measure it.** The Chamechaude full-res run initially failed exactly this way: the .blend used a 2×2 km recentered cloud offset (865.5, 951.9, 282.6) m from the assumed `lambert − origin` frame, so the renders were painted ~1.3 km off-terrain.

The pipeline now solves this automatically by DEM cross-correlation (see Step 2b): the capture exports an elevation grid of the cloud the cameras actually saw, reprojection matches it against the raw tiles, and a >2 m residual aborts the run. Translation-only by design — a rotated or scaled cloud fails the residual check loudly instead of producing garbage.

A non-identity `matrix_world` on the PC object (e.g. Aiguille Dibona, offset −1478, −1041, −744) is applied when exporting the DEM, so it's covered by the same mechanism.

---

## Scenes validated

| Scene | Web PLY pts | Full HD pts | Notes |
|---|---|---|---|
| Chamechaude | 7.25M | 59.5M | Night scene, moonlight + spotlight. Reprojection matches render. |
| Aiguille Dibona (Ce que je cache) | 6.3M | — | 6 lights + blue emission curve. Bake path. |
| Alpe d'Huez | 9.7M | — | Moon + emission curve (lava path). Bake path. |

---

## Fallback — Python physics bake (`bake_lighting.py`)

The original bake script is kept as a fallback for quick previews or scenes where orbit renders aren't available yet. It re-implements Blender's light physics (cos θ shading, terrain shadowing, emission curves with segment-distance model) with calibrated constants (K_GLOBAL=157, K_SUN=0.105, K_CURVE=0.19). It matches well but requires per-scene validation; the reprojection path is universal.

```bash
python bake_lighting.py <input.ply> <scene_lights.json> <output-lit.ply>
```

---

## Roadmap

### Pipeline
- **Orbit camera rework** — square renders (~3072×3072, orbits have no preferred orientation — fixes vertical scenes like Dibona), fixed orbit lens decoupled from the scene camera, and orbit radius computed from the FOV (`radius = (diag/2)/sin(FOV/2) × 1.1`) so the scene bounding sphere always fits the frame. Today the orbit copies the scene camera's lens and uses a magic 0.85 × diagonal radius, which crops tiles out of frame on long-lens scenes.
- **Drag-and-drop launcher** — drop a `.blend`, the full pipeline runs automatically
- **Emission mesh objects** — reproject light from mesh faces with Emission material (currently only bezier curves handled by bake fallback)
- **COPC raw tile integration** — full pipeline validated end-to-end (in progress)

### Pipeline (continued)
- **Windowed relight for huge orthos** — `albedo_relight.py` loads the full raster (×2) in RAM; fine to ~15000² px, but a 10 km scene at 0.20 m/px (50000²) would need ~60 GB. Fix when needed: process the ortho in crops per LiDAR tile. PDAL colorization is unaffected (GDAL streams blocks).

### Display
- **Default view = 2D render framing** — open the viewer on the exact framing of the hero render: initialize the Potree camera from the scene camera exported in `transforms.json` (`cam_scene` position, orientation and FOV), converted to the cloud's frame with the auto-align offset, instead of the generic `fitToScreen`.
- **Circular tile clipping** — for scenes centered on a summit or island, clip the point cloud to a circle rather than a rectangle. Logic: read the `OrbitTarget` empty position, find the closest border of the combined tile bounding box, use that distance as radius, delete all points whose XY distance from the target exceeds it. Opt-in only — not suitable for scenes where the point of interest is near a tile edge.
- **Volumetric display** — some scenes use Blender volumes to shape spotlight beams or add fog. These are invisible to the reprojection (no geometry to project onto). Plan: export the volume as OpenVDB from Blender, convert to NanoVDB / 3D texture, composite as a ray-march shader in Three.js over the Potree point cloud. First test scene: Aiguille Dibona (Ce que je cache) which has spotlight beam volumes.
