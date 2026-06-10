# pointcloud-light-baker

A pipeline for baking Blender artistic lighting into LiDAR point clouds and displaying them in a web viewer alongside 2D renders.

The goal: a 3D companion to each image in a deep-zoom photography/LiDAR website. Drop a `.blend` file, get a lit, web-ready point cloud that visually matches the Blender Cycles render.

---

## What it does

LiDAR scans from IGN HD contain 30–60 million points with satellite RGB colour. Blender provides dramatic lighting: area lights, spotlights, suns, and glowing emission curves traced along the terrain. This pipeline:

1. **Extracts** all lights, emission curves, and scene transforms from the open `.blend` (read-only, never modifies the file)
2. **Bakes** irradiance onto each point using physically-based formulas that match Blender Cycles output — including surface normals, terrain self-shadowing, and Lambertian shading
3. **Outputs** a lit PLY file ready for web display

---

## Current state

- Satellite RGB + dramatic Blender lighting: ✅
- POINT / SPOT / AREA / SUN light types: ✅
- Emission bezier curves (line lights): ✅
- Terrain self-shadowing (heightfield ray-march): ✅
- Surface normal estimation (k-NN PCA): ✅
- Cross-scene calibration (K_GLOBAL / K_SUN / K_CURVE): ✅
- Per-scene exposure override via `scene_lights.json`: ✅
- Emission mesh objects: 🔜
- Full-resolution display via raw COPC tiles: 🔜
- Drag-and-drop `.blend` launcher: 🔜

---

## Pipeline

```
.blend (Blender, read-only)
    │
    ├─ extract_lights.py  →  scene_lights.json   (run in Blender Script Editor)
    │
    └─ bake_lighting.py <input.ply> <scene_lights.json> <output.ply>
                                                  (pure Python, no Blender needed)
                                │
                                ▼
                        lit point cloud PLY
                                │
               ┌────────────────┴────────────────┐
               ▼                                 ▼
        prototype/                           potree/
        Three.js viewer                  Potree COPC viewer
        (< 15M pts)                      (60M+ pts, LOD)
```

### Requirements

```
pip install numpy scipy
```

PDAL (via QGIS) for COPC conversion. No Blender install needed to run the bake.

### Quick start

```bash
# Step 1 — in Blender Script Editor (read-only)
# Open extract_lights.py → Run Script
# Writes scene_lights.json next to the .blend

# Step 2 — bake
python bake_lighting.py input.ply scene_lights.json output-lit.ply

# Step 3 — view (Three.js, < 15M pts)
cd prototype && python -m http.server 8080
# open http://localhost:8080/?scene=<scene-id>

# Step 3 — view (Potree, 60M+ pts with LOD)
# Convert first:
convert_to_copc.bat output-lit.ply scene-id
# Then:
cd potree && python server.py 8081
# open http://localhost:8081/?scene=scene-id
```

---

## Per-scene tuning (no code changes)

After running `extract_lights.py`, the generated `scene_lights.json` accepts optional overrides:

```json
{
  "exposure": 1.2,         // global brightness multiplier (all lights)
  "curve_exposure": 0.6,   // emission curves only
  "lights": [
    {
      "name": "key light",
      "energy_scale": 0.5  // trim this specific light
    }
  ]
}
```

All keys default to `1.0` if absent.

---

## Point cloud coordinate alignment

The bake script aligns the PLY to Blender world space using the PC object's `matrix_world` (exported in `scene_lights.json`). The safest input is a PLY exported directly from Blender's loaded scene — this guarantees the coordinate frames match. Alternatively, any PLY processed with the same `--origin` parameter as the Blender-loaded cloud will align correctly.

---

## Roadmap

- **Drag-and-drop launcher** — drop a `.blend`, the full pipeline runs automatically (export web PLY from Blender, extract lights, bake, convert to COPC)
- **Orbit target empty** — place an empty named `VIEWER_TARGET` in the `.blend` to set the camera orbit point; fallback to terrain bounding-box centre
- **Emission meshes** — bake light from mesh objects with Emission material (currently only bezier curves are supported)
- **COPC raw tile integration** — use the original full-resolution IGN COPC tiles (usually in a `Lidar/` folder next to the `.blend`) as the display cloud, merging lighting from the baked low-res PLY. This gives maximum point density in the viewer without processing overhead. Auto-download from IGN as fallback if tiles are not found locally.

---

## Scenes validated

| Scene | Points (web) | Lights | Notes |
|---|---|---|---|
| Aiguille Dibona | 6.3M | 4× red AREA/SPOT + 2× white POINT | Ce que je cache series |
| Chamechaude | 7.25M | moon (SUN) + spotlight | Night scene |
| Alpe d'Huez | 9.7M | moon (SUN) + SPOT + emission curve | Lava-path bezier |
