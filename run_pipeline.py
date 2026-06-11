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
  Stage 2b Relight   albedo x light separation (albedo_relight.py) — texture
                     from the ortho, lighting from the renders. Per-tile skip.
  Stage 3  Merge     untwine → <scene>.copc.laz + <scene>-detail.copc.laz
                     (manifest-based skip)
  Stage 4  Register  add scenes to potree/index.html SCENES + bump cache version

Scene folder conventions (same as lidar_pipeline.py):
  <blend_dir>/LIDAR/LIDAR Bases IGN/   raw IGN .copc.laz tiles
  <blend_dir>/LIDAR/output/*_raster.tif BDORTHO ortho (albedo fallback)

Usage:
  python run_pipeline.py <scene.blend> [--both]

  --both  also merge/register the plain reprojection cloud (diagnostic:
          shows exactly what the Cycles renders saw, no ortho texture).
          By default only <scene>-detail is published when a raster exists.
"""

import glob
import json
import os
import re
import subprocess
import sys

HERE        = os.path.dirname(os.path.abspath(__file__))
LIDAR_PROJ  = r"C:\Users\Tiphaine\Pictures\3D\LIDAR PROJECT"   # lidar_pipeline.py
NATIVE_RES  = 0.20   # IGN BDORTHO native m/px — rasters coarser than this are re-fetched
BLENDER     = r"C:\Program Files\Blender Foundation\Blender 5.0\blender.exe"
QGIS_BIN    = r"C:\Program Files\QGIS 3.40.5\bin"
UNTWINE     = r"C:\Program Files\QGIS 3.40.5\apps\qgis-ltr\untwine.exe"
CAPTURE_PY  = os.path.join(HERE, "gs-capture", "gs_capture.py")
REPROJECT   = os.path.join(HERE, "reproject_copc.py")
RELIGHT     = os.path.join(HERE, "albedo_relight.py")
POTREE_HTML = os.path.join(HERE, "potree", "index.html")
N_FRAMES    = 146   # 4 rings x 36 + top + scene cam — keep in sync w/ gs_capture


def raster_res(path):
    """Pixel size in metres from the GeoTIFF tags (no GDAL needed)."""
    from PIL import Image
    Image.MAX_IMAGE_PIXELS = None   # native orthos exceed the bomb guard
    try:
        return float(Image.open(path).tag_v2[33550][0])
    except Exception:
        return None


def ensure_native_raster(lit_dir, blend_dir, slug, current):
    """Never trust the raster in the folder — measure it. If it's missing or
    coarser than NATIVE_RES, fetch the IGN BDORTHO at native resolution over
    the exact extent of the lit tiles. Returns the raster path to use."""
    res = raster_res(current) if current else None
    if res is not None and res <= NATIVE_RES + 0.01:
        print(f"  raster ok: {os.path.basename(current)} @ {res:g} m/px")
        return current
    if res is not None:
        print(f"  raster {os.path.basename(current)} is {res:g} m/px — "
              f"fetching native {NATIVE_RES} m/px from IGN")
    else:
        print(f"  no raster — fetching native {NATIVE_RES} m/px from IGN")

    import laspy
    import math
    x0 = y0 = float("inf")
    x1 = y1 = float("-inf")
    for f in glob.glob(os.path.join(lit_dir, "*_lit.laz")):
        h = laspy.open(f).header
        x0, x1 = min(x0, h.x_min), max(x1, h.x_max)
        y0, y1 = min(y0, h.y_min), max(y1, h.y_max)
    x0, y0 = math.floor(x0), math.floor(y0)
    x1, y1 = math.ceil(x1), math.ceil(y1)

    out = os.path.join(blend_dir, "LIDAR", "output",
                       f"{slug}-ortho-020_raster.tif")
    sys.path.insert(0, LIDAR_PROJ)
    from lidar_pipeline import fetch_raster_tiled
    try:
        fetch_raster_tiled(x0, y0, x1, y1, NATIVE_RES,
                           "ORTHOIMAGERY.ORTHOPHOTOS", out)
    except Exception as exc:
        if current:
            print(f"  WMS fetch failed ({exc}) — falling back to {current}")
            return current
        die(f"ortho fetch failed and no fallback raster: {exc}")
    return out


def banner(stage, msg):
    print(f"\n{'━'*64}\n  STAGE {stage} — {msg}\n{'━'*64}", flush=True)


def die(msg):
    print(f"\nERROR: {msg}", flush=True)
    sys.exit(1)


def main():
    if len(sys.argv) < 2 or not sys.argv[1].lower().endswith(".blend"):
        die("usage: python run_pipeline.py <scene.blend> [--both]")
    both = "--both" in sys.argv[2:]
    blend = os.path.abspath(sys.argv[1])
    if not os.path.exists(blend):
        die(f"not found: {blend}")

    blend_dir  = os.path.dirname(blend)
    scene_name = os.path.splitext(os.path.basename(blend))[0]
    slug       = re.sub(r"[^a-z0-9]+", "-", scene_name.lower()).strip("-")
    capture    = os.path.join(HERE, "gs-capture", "output", scene_name)
    tiles_dir  = os.path.join(blend_dir, "LIDAR", "LIDAR Bases IGN")
    lit_dir    = os.path.join(blend_dir, "LIDAR", "output-lit-tiles")
    lit2_dir   = os.path.join(blend_dir, "LIDAR", "output-lit2-tiles")
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

    # ── Stage 2b — albedo x light relight ────────────────────────────────────
    # Default: publish only the detail variant (texture from ortho, lighting
    # from renders). The plain reprojection is a diagnostic — what Cycles
    # actually saw — published only with --both.
    variants = []
    banner("2b", "relight — albedo x light separation")
    raster = ensure_native_raster(lit_dir, blend_dir, slug, raster)
    if raster:
        r = subprocess.run([sys.executable, RELIGHT, lit_dir, raster, lit2_dir,
                            "--capture", capture])
        if r.returncode != 0:
            die("relight failed")
        detail_out = os.path.join(HERE, "potree", "pointclouds",
                                  f"{slug}-detail.copc.laz")
        variants.append((lit2_dir, detail_out, f"{slug}-detail",
                         f"{scene_name} · Detail"))
        if both:
            variants.append((lit_dir, copc_out, slug, scene_name))
    else:
        print("  no raster available — publishing plain reprojection only")
        variants.append((lit_dir, copc_out, slug, scene_name))

    # ── Stage 3 — merge to COPC (both variants) ──────────────────────────────
    rebuilt = False
    for src_dir, out_path, _, _ in variants:
        lit_files = sorted(glob.glob(os.path.join(src_dir, "*_lit.laz")))
        if not lit_files:
            die(f"no *_lit.laz in {src_dir}")
        # Manifest = exact set of merged inputs + mtimes. An mtime-only check
        # misses a changed tile SET (e.g. tiles removed after a footprint fix).
        manifest_path = out_path + ".manifest.json"
        manifest = {os.path.basename(f): os.path.getmtime(f) for f in lit_files}
        old = None
        if os.path.exists(manifest_path):
            with open(manifest_path) as f:
                old = json.load(f)
        if os.path.exists(out_path) and old == manifest:
            banner(3, f"merge — {os.path.basename(out_path)} up to date, skipping")
            continue
        banner(3, f"merge — untwine {len(lit_files)} tiles → {os.path.basename(out_path)}")
        env = dict(os.environ, PATH=QGIS_BIN + os.pathsep + os.environ["PATH"])
        tmp = out_path[:-len(".copc.laz")] + "_tmp.copc.laz"
        r = subprocess.run([UNTWINE, "-i", src_dir, "-o", tmp], env=env)
        if r.returncode != 0 or not os.path.exists(tmp):
            if os.path.exists(tmp):
                os.remove(tmp)
            die("untwine failed")
        os.replace(tmp, out_path)
        with open(manifest_path, "w") as f:
            json.dump(manifest, f, indent=1)
        rebuilt = True

    # ── Stage 4 — register in viewer ─────────────────────────────────────────
    banner(4, "register in Potree viewer")
    with open(POTREE_HTML, encoding="utf-8") as f:
        html = f.read()
    changed = False
    for _, _, scene_id, label in variants:
        if f'"{scene_id}"' not in html:
            html = html.replace("const SCENES = {",
                                f'const SCENES = {{\n  "{scene_id}": "{label}",', 1)
            changed = True
            print(f"  added scene '{scene_id}' to SCENES")
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
    print(f"  http://localhost:8081/?scene={variants[0][2]}")
    print(f"{'━'*64}\n")


if __name__ == "__main__":
    main()
