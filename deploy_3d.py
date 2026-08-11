"""
Deploy a 3D scene to OVH — cloud + menu entry, together.
--------------------------------------------------------
    python deploy_3d.py <scene-id> [--name "Short Name"] [--sub "Subtitle"]
    python deploy_3d.py --list

Why this exists: the COPC upload and the PUBLISHED menu entry are two halves
of one act, and doing them by hand meant the menu was always the half that got
forgotten. PUBLISHED is a CURATED list — SCENES holds every cloud ever built,
most of them local test scenes with no data on the server — so the menu entry
must be written exactly when the cloud actually lands on the server, never at
build time. run_pipeline.py deliberately does NOT touch PUBLISHED for that
reason: it would publish dead links for every local experiment.

Idempotent: re-running skips an already-uploaded cloud (size match) and never
duplicates a menu entry.
"""

import os
import re
import sys
import json
import subprocess

HERE        = os.path.dirname(os.path.abspath(__file__))
POTREE      = os.path.join(HERE, "potree")
CLOUDS      = os.path.join(POTREE, "pointclouds")
POTREE_HTML = os.path.join(POTREE, "index.html")

HOST        = "tiphainef@ssh.cluster114.hosting.ovh.net"
REMOTE_3D   = "~/lidar/3d"        # for ssh (tilde expanded by the shell)
REMOTE_REL  = "lidar/3d"          # for sftp (paths are home-relative already)
PUBLIC_URL  = "https://lidar.tiphainebuccino.com/3d"


def die(msg):
    sys.exit(f"ERROR: {msg}")


def run(cmd, **kw):
    return subprocess.run(cmd, **kw)


def ssh(script):
    """Run a snippet on the server, return stdout (stderr folded in)."""
    r = subprocess.run(["ssh", "-o", "ConnectTimeout=20", HOST, script],
                       capture_output=True, text=True)
    # OpenSSH prints a post-quantum advisory on this host; it is not an error.
    return r.stdout.strip()


def read_html():
    with open(POTREE_HTML, encoding="utf-8") as f:
        return f.read()


def scenes_label(html, scene_id):
    """Label registered in SCENES by run_pipeline, if any."""
    m = re.search('"' + re.escape(scene_id) + r'":\s*\{"label":\s*"([^"]*)"', html)
    if m:
        return m.group(1)
    m = re.search('"' + re.escape(scene_id) + r'":\s*"([^"]*)"', html)
    return m.group(1) if m else None


def in_published(html, scene_id):
    return f'id: "{scene_id}"' in html


def add_published(html, scene_id, name, sub):
    """Append an entry to the PUBLISHED array, just before its closing ']'."""
    m = re.search(r'const PUBLISHED = \[\n(.*?)\n\];', html, re.S)
    if not m:
        die("could not find the PUBLISHED array in index.html")
    entry = (f'  {{ id: "{scene_id}", name: {json.dumps(name, ensure_ascii=False)}, '
             f'sub: {json.dumps(sub, ensure_ascii=False)} }},')
    return html[:m.end(1)] + "\n" + entry + html[m.end(1):]


def main():
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help"):
        sys.exit(__doc__)

    html = read_html()

    if args[0] == "--list":
        local = sorted(f[:-len(".copc.laz")] for f in os.listdir(CLOUDS)
                       if f.endswith(".copc.laz"))
        # Compare SIZES, not mere existence: a half-finished scp leaves a file
        # sitting there, and "is it really deployed?" is the whole point of
        # this table.
        remote = {}
        for line in ssh(f"stat -c'%s %n' {REMOTE_3D}/pointclouds/*.copc.laz "
                        "2>/dev/null").splitlines():
            parts = line.strip().split(None, 1)
            if len(parts) == 2 and parts[0].isdigit():
                base = os.path.basename(parts[1])
                if base.endswith(".copc.laz"):
                    remote[base[:-len(".copc.laz")]] = int(parts[0])
        print(f"{'scene':60} {'server':>10}  {'menu':>6}")
        for s in local:
            sz = os.path.getsize(os.path.join(CLOUDS, s + ".copc.laz"))
            rz = remote.get(s)
            if rz is None:
                state = "NO"
            elif rz == sz:
                state = "yes"
            else:
                state = f"{100 * rz / sz:.0f}%"   # partial / stale upload
            print(f"{s[:58]:60} {state:>10}  "
                  f"{'yes' if in_published(html, s) else 'NO':>6}")
        return

    scene_id = args[0]
    name = sub = None
    for i, a in enumerate(args[1:]):
        if a == "--name" and i + 2 <= len(args) - 1:
            name = args[i + 2]
        elif a == "--sub" and i + 2 <= len(args) - 1:
            sub = args[i + 2]

    cloud = os.path.join(CLOUDS, scene_id + ".copc.laz")
    if not os.path.exists(cloud):
        die(f"no such cloud: {cloud}\n       (try --list)")
    manifest = cloud + ".manifest.json"
    size = os.path.getsize(cloud)

    label = scenes_label(html, scene_id)
    if name is None:
        name = label or scene_id
    if sub is None:
        sub = ""

    print(f"  scene : {scene_id}")
    print(f"  cloud : {size / 1024**3:.2f} GB")
    print(f"  menu  : name={name!r} sub={sub!r}")

    # ── 1. upload the cloud (skip when the server copy already matches) ──────
    remote_size = ssh(f"stat -c%s {REMOTE_3D}/pointclouds/"
                      f"'{scene_id}.copc.laz' 2>/dev/null || echo 0")
    if remote_size.isdigit() and int(remote_size) == size:
        print(f"  cloud already on server ({size:,} bytes) — skipping upload")
    else:
        have = int(remote_size) if remote_size.isdigit() else 0
        if 0 < have < size:
            print(f"  partial upload found ({100 * have / size:.0f}%) — resuming")
        else:
            print("  uploading cloud… (this is the slow part)")
        # sftp 'reput', not scp: a dropped wifi mid-transfer leaves a TRUNCATED
        # file behind, and scp restarts from byte zero every time (1.6 GB again).
        # reput continues from whatever is already there. Worse, scp has been
        # seen to exit 0 on a truncated file — which is why the md5 check below
        # is not optional.
        local_fwd = cloud.replace("\\", "/")
        batch = f'reput "{local_fwd}" "{REMOTE_REL}/pointclouds/{scene_id}.copc.laz"\nbye\n'
        r = subprocess.run(["sftp", "-o", "ConnectTimeout=20", HOST],
                           input=batch, text=True)
        if r.returncode != 0:
            die("cloud upload failed — rerun to resume from where it stopped")
        if os.path.exists(manifest):
            run(["scp", "-o", "ConnectTimeout=20", manifest,
                 f"{HOST}:{REMOTE_3D}/pointclouds/"])

        # Verify by CHECKSUM. Matching byte counts are not proof: a resumed
        # transfer onto a bad prefix would size-match and still be corrupt.
        check = ssh(f"stat -c%s {REMOTE_3D}/pointclouds/'{scene_id}.copc.laz'")
        if not check.isdigit() or int(check) != size:
            die(f"size mismatch after upload: local {size}, server {check}")
        print("  verifying checksum…")
        import hashlib
        h = hashlib.md5()
        with open(cloud, "rb") as f:
            for blk in iter(lambda: f.read(1 << 20), b""):
                h.update(blk)
        remote_md5 = ssh(f"md5sum {REMOTE_3D}/pointclouds/"
                         f"'{scene_id}.copc.laz' | cut -d' ' -f1")
        if remote_md5 != h.hexdigest():
            die(f"CHECKSUM MISMATCH — upload is corrupt\n"
                f"       local  {h.hexdigest()}\n       server {remote_md5}\n"
                f"       delete the server copy and rerun")
        print(f"  uploaded and verified ({size:,} bytes, md5 {remote_md5[:12]}…)")

    # ── 2. menu entry — the half that always got forgotten ───────────────────
    if in_published(html, scene_id):
        print("  menu entry already present")
    else:
        html = add_published(html, scene_id, name, sub)
        with open(POTREE_HTML, "w", encoding="utf-8") as f:
            f.write(html)
        print(f"  added to PUBLISHED menu: {name} · {sub}")

    # ── 3. push the viewer page (carries menu + CLOUD_VERSION) ───────────────
    r = run(["scp", "-o", "ConnectTimeout=20", POTREE_HTML,
             f"{HOST}:{REMOTE_3D}/index.html"])
    if r.returncode != 0:
        die("index.html upload failed")
    print("  index.html deployed")

    print(f"\n  LIVE: {PUBLIC_URL}/?scene={scene_id}")


if __name__ == "__main__":
    main()
