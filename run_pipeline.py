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
  python run_pipeline.py <scene.blend> [--both] [--test]

  --both  also merge/register the plain reprojection cloud (diagnostic:
          shows exactly what the Cycles renders saw, no ortho texture).
          By default only <scene>-detail is published when a raster exists.
  --test  fast low-quality preset for pipeline iteration: 26 orbit frames
          at 720/16spp (vs 146 at 4k/128spp), 4 m/px ortho, points
          decimated 1/10. All outputs are suffixed "-test" (capture dir,
          lit tiles, COPC, scene entry) so they never collide with a
          production run of the same .blend.
  --full  skip the interactive test-mode question (force production quality).
          With neither flag, an interactive terminal asks "Test mode? [y/N]";
          non-interactive runs default to full quality.
"""

import glob
import json
import os
import re
import subprocess
import sys
import time

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
N_FRAMES_TEST = 26  # 2 rings x 12 + top + scene cam — gs_capture --test preset
TEST_RES    = 4.0   # test-mode ortho m/px — low-def sat RGB (native is 0.20)
TEST_DECIMATE = 10  # test-mode point decimation — keep 1 point in N


def raster_res(path):
    """Pixel size in metres from the GeoTIFF tags (no GDAL needed)."""
    from PIL import Image
    Image.MAX_IMAGE_PIXELS = None   # native orthos exceed the bomb guard
    try:
        return float(Image.open(path).tag_v2[33550][0])
    except Exception:
        return None


def ensure_native_raster(lit_dir, blend_dir, slug, current, want_res=NATIVE_RES):
    """Never trust the raster in the folder — measure it. If it's missing or
    coarser than want_res, fetch the IGN BDORTHO at that resolution over
    the exact extent of the lit tiles. Returns the raster path to use."""
    res = raster_res(current) if current else None
    if res is not None and res <= want_res + 0.01:
        print(f"  raster ok: {os.path.basename(current)} @ {res:g} m/px")
        return current
    if res is not None:
        print(f"  raster {os.path.basename(current)} is {res:g} m/px — "
              f"fetching {want_res} m/px from IGN")
    else:
        print(f"  no raster — fetching {want_res} m/px from IGN")

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
                       f"{slug}-ortho-{int(round(want_res * 100)):03d}_raster.tif")
    sys.path.insert(0, LIDAR_PROJ)
    from lidar_pipeline import fetch_raster_tiled
    try:
        fetch_raster_tiled(x0, y0, x1, y1, want_res,
                           "ORTHOIMAGERY.ORTHOPHOTOS", out)
    except Exception as exc:
        if current:
            print(f"  WMS fetch failed ({exc}) — falling back to {current}")
            return current
        die(f"ortho fetch failed and no fallback raster: {exc}")
    return out


WFS_DALLES = ("https://data.geopf.fr/wfs/ows?service=WFS&version=2.0.0"
              "&request=GetFeature&typeNames=IGNF_NUAGES-DE-POINTS-LIDAR-HD:dalle"
              "&outputFormat=application/json&count=200&srsName=EPSG:2154"
              "&bbox={x0},{y0},{x1},{y1},EPSG:2154")


def ensure_tiles(tiles_dir, blend_dir):
    """If the tiles folder is empty, download the IGN LiDAR HD tiles covering
    the scene. The Lambert bbox comes from the scene's intermediate LAZ in
    LIDAR/output/ (the lidar_pipeline merge, still in Lambert-93)."""
    if any(f.lower().endswith((".laz", ".las")) for f in os.listdir(tiles_dir)):
        return
    print("  tiles folder is empty — fetching IGN LiDAR HD tiles")
    srcs = glob.glob(os.path.join(blend_dir, "LIDAR", "output", "*.laz"))
    if not srcs:
        die(f"tiles folder is EMPTY: {tiles_dir}\n"
            "and no LIDAR/output/*.laz to derive the scene bbox from.\n"
            "Download the raw IGN LiDAR HD .copc.laz tiles manually.")
    import laspy
    import urllib.request
    h = laspy.open(max(srcs, key=os.path.getsize)).header
    # shrink by 1 m so tiles that only touch the bbox edge are not pulled in
    url = WFS_DALLES.format(x0=h.x_min + 1, y0=h.y_min + 1,
                            x1=h.x_max - 1, y1=h.y_max - 1)
    print(f"  scene bbox {h.x_min:.0f},{h.y_min:.0f} .. {h.x_max:.0f},{h.y_max:.0f}")
    with urllib.request.urlopen(url, timeout=60) as r:
        feats = json.load(r).get("features", [])
    # the WFS bbox filter is loose (returns edge-neighbours) — keep only
    # tiles whose geometry truly overlaps the scene bbox (1 m inset, so a
    # centimetre overhang doesn't pull in a whole neighbouring tile)
    def overlaps(f):
        rings = f["geometry"]["coordinates"]
        pts = rings[0] if f["geometry"]["type"] == "Polygon" else rings[0][0]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return (min(xs) < h.x_max - 1 and max(xs) > h.x_min + 1 and
                min(ys) < h.y_max - 1 and max(ys) > h.y_min + 1)
    feats = [f for f in feats if overlaps(f)]
    if not feats:
        die("IGN WFS returned no LiDAR HD tiles for the scene bbox")
    print(f"  {len(feats)} tiles to download")
    for i, f in enumerate(feats):
        p = f["properties"]
        name = p.get("name_download") or p["name"] + ".copc.laz"
        out = os.path.join(tiles_dir, name)
        if os.path.exists(out):
            print(f"  [{i+1}/{len(feats)}] {name} — already here")
            continue
        tmp = out + ".part"
        try:
            # the geoplateforme download server 403s Python's default UA
            req = urllib.request.Request(
                p["url"], headers={"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)"})
            with urllib.request.urlopen(req, timeout=120) as r, open(tmp, "wb") as fo:
                total = int(r.headers.get("Content-Length") or 0)
                done = 0
                t0 = last = time.time()
                while True:
                    chunk = r.read(1 << 20)
                    if not chunk:
                        break
                    fo.write(chunk)
                    done += len(chunk)
                    now = time.time()
                    if now - last >= 0.25 or done == total:
                        last = now
                        speed = done / max(now - t0, 1e-3)
                        if total:
                            frac = done / total
                            bar = "█" * int(28 * frac) + "░" * (28 - int(28 * frac))
                            eta = (total - done) / max(speed, 1)
                            line = (f"  [{i+1}/{len(feats)}] {name}  {bar} "
                                    f"{frac*100:3.0f}%  {done/1e6:.0f}/{total/1e6:.0f} MB"
                                    f"  {speed/1e6:.1f} MB/s  ETA {eta:4.0f}s")
                        else:
                            line = (f"  [{i+1}/{len(feats)}] {name}  "
                                    f"{done/1e6:.0f} MB  {speed/1e6:.1f} MB/s")
                        print("\r" + line, end="", flush=True)
            print()
            os.replace(tmp, out)
        except Exception as exc:
            print()
            if os.path.exists(tmp):
                os.remove(tmp)
            die(f"tile download failed ({exc}) — relaunch to resume")


def compute_scene_view(capture):
    """Per-scene default view = the .blend scene camera's framing, converted
    to cloud (Lambert-93) coordinates with the measured alignment offset.
    Returns {position, target, fov} or None if the capture predates the
    scene_camera/alignment exports."""
    import math
    tj_path = os.path.join(capture, "transforms.json")
    al_path = os.path.join(capture, "alignment.json")
    if not (os.path.exists(tj_path) and os.path.exists(al_path)):
        return None
    with open(tj_path) as f:
        sc = json.load(f).get("scene_camera")
    if not sc:
        return None
    with open(al_path) as f:
        off = json.load(f)["blend_to_lambert"]
    M = sc["transform_matrix"]
    pos = [M[0][3] + off[0], M[1][3] + off[1], M[2][3] + off[2]]
    fwd = [-M[0][2], -M[1][2], -M[2][2]]          # camera -Z in world
    # aim point: along the view axis, at the distance of the cloud centre
    dist = 1500.0
    meta_path = os.path.join(capture, "cloud_dem_meta.json")
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
        cx = meta["min_xy"][0] + meta["shape"][0] * meta["cell"] / 2 + off[0]
        cy = meta["min_xy"][1] + meta["shape"][1] * meta["cell"] / 2 + off[1]
        d = (cx - pos[0]) * fwd[0] + (cy - pos[1]) * fwd[1]
        dist = max(200.0, d)
    target = [pos[i] + fwd[i] * dist for i in range(3)]
    # vertical FOV (three.js convention) from lens/sensor/render aspect
    sw, lens = sc["sensor_width"], sc["lens"]
    rx, ry = sc["res_x"], sc["res_y"]
    if sc.get("sensor_fit") == "VERTICAL" or (sc.get("sensor_fit") == "AUTO" and ry > rx):
        vfov = 2 * math.atan(sw / (2 * lens))
    else:
        vfov = 2 * math.atan((sw * ry / rx) / (2 * lens))
    return {"position": [round(v, 2) for v in pos],
            "target": [round(v, 2) for v in target],
            "fov": round(math.degrees(vfov), 2)}


def banner(stage, msg):
    print(f"\n{'━'*64}\n  STAGE {stage} — {msg}\n{'━'*64}", flush=True)


def die(msg):
    print(f"\nERROR: {msg}", flush=True)
    sys.exit(1)


def main():
    if len(sys.argv) < 2 or not sys.argv[1].lower().endswith(".blend"):
        die("usage: python run_pipeline.py <scene.blend> [--both] [--test]")
    both = "--both" in sys.argv[2:]
    test = "--test" in sys.argv[2:]
    # No flag? Ask. (--test or --full skip the question; non-interactive runs
    # — schedulers, scripts — default to full quality without blocking.)
    if not test and "--full" not in sys.argv[2:] and sys.stdin.isatty():
        try:
            test = input("  Test mode — fast low-quality preset? [y/N] "
                         ).strip().lower() in ("y", "yes", "o", "oui")
        except EOFError:
            pass
    blend = os.path.abspath(sys.argv[1])
    if not os.path.exists(blend):
        die(f"not found: {blend}")

    blend_dir  = os.path.dirname(blend)
    scene_name = os.path.splitext(os.path.basename(blend))[0]
    # TEST preset: everything low-quality and namespaced "-test" so a test run
    # never touches the production capture, lit tiles, COPCs or scene entries.
    if test:
        print("\n  ── TEST PRESET — fast low-quality run, outputs suffixed -test ──")
        scene_name += "-test"
    n_frames   = N_FRAMES_TEST if test else N_FRAMES
    slug       = re.sub(r"[^a-z0-9]+", "-", scene_name.lower()).strip("-")
    capture    = os.path.join(HERE, "gs-capture", "output", scene_name)
    tiles_dir  = os.path.join(blend_dir, "LIDAR", "LIDAR Bases IGN")
    suffix     = "-test" if test else ""
    lit_dir    = os.path.join(blend_dir, "LIDAR", f"output-lit-tiles{suffix}")
    lit2_dir   = os.path.join(blend_dir, "LIDAR", f"output-lit2-tiles{suffix}")
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
    ensure_tiles(tiles_dir, blend_dir)

    # ── Stage 1 — render orbit ───────────────────────────────────────────────
    tj = os.path.join(capture, "transforms.json")
    dem = os.path.join(capture, "cloud_dem.npy")
    have = len(glob.glob(os.path.join(capture, "images", "*.webp")))
    if os.path.exists(tj) and os.path.exists(dem) and have >= n_frames:
        banner(1, f"render — complete ({have} frames), skipping")
    else:
        banner(1, f"render — {have}/{n_frames} frames present, launching Blender")
        os.makedirs(os.path.join(capture, "images"), exist_ok=True)
        r = subprocess.run([BLENDER, "--background", blend,
                            "--python", CAPTURE_PY, "--", capture, scene_name]
                           + (["--test"] if test else []))
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
    if test:
        # decimated points + relaxed alignment guard: forest canopy inflates
        # the residual; in test mode favour getting through the pipeline
        cmd += ["--decimate", str(TEST_DECIMATE), "--max-residual", "5"]
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
    raster = ensure_native_raster(lit_dir, blend_dir, slug, raster,
                                  TEST_RES if test else NATIVE_RES)
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
    view = compute_scene_view(capture)
    for _, _, scene_id, label in variants:
        if f'"{scene_id}"' not in html:
            entry = json.dumps({"label": label, "view": view}) if view else f'"{label}"'
            html = html.replace("const SCENES = {",
                                f'const SCENES = {{\n  "{scene_id}": {entry},', 1)
            changed = True
            print(f"  added scene '{scene_id}' to SCENES"
                  + (" (with 2D-render default view)" if view else ""))
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
