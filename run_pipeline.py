"""
run_pipeline.py — one-drop pipeline: .blend → lit COPC in the Potree viewer.

Called by "RUN PIPELINE.bat" (drag a .blend onto it). Every stage is
resumable: close the window or kill the process at any point, relaunch with
the same .blend, and it continues where it stopped.

  Stage 1  Render    orbit frames + transforms.json + cloud_dem.npy (Blender,
                     background). Per-frame skip — re-renders only missing
                     frames.
  Stage 2  Reproject color raw IGN tiles from the renders (auto frame
                     alignment, albedo fallback). Per-tile skip.
  Stage 3  Merge     untwine → single <scene>.copc.laz (skipped if up to date)
  Stage 4  Register  add scene to potree/index.html SCENES + bump cache version

Scene folder conventions (same as lidar_pipeline.py):
  <blend_dir>/LIDAR/LIDAR Bases IGN/   raw IGN .copc.laz tiles
  <blend_dir>/LIDAR/output/*_raster.tif BDORTHO ortho (albedo fallback)

Usage:
  python run_pipeline.py <scene.blend>
"""

import glob
import json
import os
import re
import subprocess
import sys

HERE        = os.path.dirname(os.path.abspath(__file__))
BLENDER     = r"C:\Program Files\Blender Foundation\Blender 5.0\blender.exe"
QGIS_BIN    = r"C:\Program Files\QGIS 3.40.5\bin"
UNTWINE     = r"C:\Program Files\QGIS 3.40.5\apps\qgis-ltr\untwine.exe"
CAPTURE_PY  = os.path.join(HERE, "gs-capture", "gs_capture.py")
REPROJECT   = os.path.join(HERE, "reproject_copc.py")
POTREE_HTML = os.path.join(HERE, "potree", "index.html")
N_FRAMES    = 146   # 4 rings x 36 + top + scene cam — keep in sync w/ gs_capture


def banner(stage, msg):
    print(f"\n{'━'*64}\n  STAGE {stage} — {msg}\n{'━'*64}", flush=True)


def die(msg):
    print(f"\nERROR: {msg}", flush=True)
    sys.exit(1)


def main():
    if len(sys.argv) < 2 or not sys.argv[1].lower().endswith(".blend"):
        die("usage: python run_pipeline.py <scene.blend>")
    blend = os.path.abspath(sys.argv[1])
    if not os.path.exists(blend):
        die(f"not found: {blend}")

    blend_dir  = os.path.dirname(blend)
    scene_name = os.path.splitext(os.path.basename(blend))[0]
    slug       = re.sub(r"[^a-z0-9]+", "-", scene_name.lower()).strip("-")
    capture    = os.path.join(HERE, "gs-capture", "output", scene_name)
    tiles_dir  = os.path.join(blend_dir, "LIDAR", "LIDAR Bases IGN")
    lit_dir    = os.path.join(blend_dir, "LIDAR", "output-lit-tiles")
    copc_out   = os.path.join(HERE, "potree", "pointclouds", f"{slug}.copc.laz")

    rasters = glob.glob(os.path.join(blend_dir, "LIDAR", "output", "*_raster.tif"))
    raster  = max(rasters, key=os.path.getsize) if rasters else None

    print(f"  blend   : {blend}")
    print(f"  tiles   : {tiles_dir}")
    print(f"  raster  : {raster or '(none — unseen points will be black!)'}")
    print(f"  output  : {copc_out}")

    if not os.path.isdir(tiles_dir):
        die(f"no tiles folder: {tiles_dir}\n"
            "Expected raw IGN .copc.laz tiles in <blend_dir>/LIDAR/LIDAR Bases IGN/")

    # ── Stage 1 — render orbit ───────────────────────────────────────────────
    tj = os.path.join(capture, "transforms.json")
    dem = os.path.join(capture, "cloud_dem.npy")
    have = len(glob.glob(os.path.join(capture, "images", "*.webp")))
    if os.path.exists(tj) and os.path.exists(dem) and have >= N_FRAMES:
        banner(1, f"render — complete ({have} frames), skipping")
    else:
        banner(1, f"render — {have}/{N_FRAMES} frames present, launching Blender")
        os.makedirs(os.path.join(capture, "images"), exist_ok=True)
        r = subprocess.run([BLENDER, "--background", blend,
                            "--python", CAPTURE_PY, "--", capture, scene_name])
        if r.returncode != 0:
            die("Blender render failed — relaunch to resume from the last frame")
        if not (os.path.exists(tj) and os.path.exists(dem)):
            die("render finished but transforms.json / cloud_dem.npy missing")

    # ── Stage 2 — reproject ──────────────────────────────────────────────────
    n_tiles = len([f for f in os.listdir(tiles_dir)
                   if f.lower().endswith((".laz", ".las"))])
    n_lit   = len(glob.glob(os.path.join(lit_dir, "*_lit.laz")))
    banner(2, f"reproject — {n_lit}/{n_tiles} tiles done")
    cmd = [sys.executable, REPROJECT, "tiles", capture, tiles_dir, lit_dir]
    if raster:
        cmd += ["--raster", raster]
    r = subprocess.run(cmd)
    if r.returncode != 0:
        die("reprojection failed — relaunch to resume from the last tile")

    # ── Stage 3 — merge to COPC ──────────────────────────────────────────────
    lit_files = sorted(glob.glob(os.path.join(lit_dir, "*_lit.laz")))
    if not lit_files:
        die("no *_lit.laz produced")
    # Manifest = exact set of merged inputs + mtimes. An mtime-only check
    # misses a changed tile SET (e.g. tiles removed after a footprint fix).
    manifest_path = copc_out + ".manifest.json"
    manifest = {os.path.basename(f): os.path.getmtime(f) for f in lit_files}
    old = None
    if os.path.exists(manifest_path):
        with open(manifest_path) as f:
            old = json.load(f)
    if os.path.exists(copc_out) and old == manifest:
        banner(3, "merge — COPC up to date, skipping")
        rebuilt = False
    else:
        banner(3, f"merge — untwine {len(lit_files)} tiles → {os.path.basename(copc_out)}")
        env = dict(os.environ, PATH=QGIS_BIN + os.pathsep + os.environ["PATH"])
        tmp = copc_out[:-len(".copc.laz")] + "_tmp.copc.laz"
        r = subprocess.run([UNTWINE, "-i", lit_dir, "-o", tmp], env=env)
        if r.returncode != 0 or not os.path.exists(tmp):
            if os.path.exists(tmp):
                os.remove(tmp)
            die("untwine failed")
        os.replace(tmp, copc_out)
        with open(manifest_path, "w") as f:
            json.dump(manifest, f, indent=1)
        rebuilt = True

    # ── Stage 4 — register in viewer ─────────────────────────────────────────
    banner(4, "register in Potree viewer")
    with open(POTREE_HTML, encoding="utf-8") as f:
        html = f.read()
    changed = False
    if f'"{slug}"' not in html:
        html = html.replace("const SCENES = {",
                            f'const SCENES = {{\n  "{slug}": "{scene_name}",', 1)
        changed = True
        print(f"  added scene '{slug}' to SCENES")
    if rebuilt:
        m = re.search(r'const CLOUD_VERSION = "(\d+)"', html)
        if m:
            html = html.replace(m.group(0),
                                f'const CLOUD_VERSION = "{int(m.group(1)) + 1}"')
            changed = True
            print(f"  cache version bumped to {int(m.group(1)) + 1}")
    if changed:
        with open(POTREE_HTML, "w", encoding="utf-8") as f:
            f.write(html)
    else:
        print("  already registered, nothing to do")

    print(f"\n{'━'*64}")
    print(f"  DONE — start the viewer (potree/START VIEWER.bat) and open:")
    print(f"  http://localhost:8081/?scene={slug}")
    print(f"{'━'*64}\n")


if __name__ == "__main__":
    main()
