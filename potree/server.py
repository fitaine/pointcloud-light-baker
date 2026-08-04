#!/usr/bin/env python3
"""
Unified dev server: gallery at / and Potree viewer at /3d/

Range request support is required for COPC .laz files (the COPC JS library
uses Range: bytes=X-Y to read chunks without downloading the whole file;
Python's built-in http.server ignores Range headers and breaks it).

Routing:
  /            →  ../test/          (2D gallery)
  /3d/...      →  ./                (Potree viewer, range-capable)
  anything else → 404

Usage:  python server.py [port]   (default 8081)
"""

import os, sys, mimetypes, json
from urllib.parse import unquote
from http.server import HTTPServer, SimpleHTTPRequestHandler

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
GALLERY_DIR  = os.path.normpath(os.path.join(SCRIPT_DIR, '..', '..', 'test'))
POTREE_DIR   = SCRIPT_DIR
CLOUDS_DIR   = os.path.join(SCRIPT_DIR, 'pointclouds')


class RangeHTTPRequestHandler(SimpleHTTPRequestHandler):

    server_version = "RangeHTTP/1.0"

    def _resolve_path(self):
        """Return (fs_path, url_tail) with the /3d/ prefix stripped."""
        url = unquote(self.path.split('?')[0])
        if url.startswith('/3d/') or url == '/3d':
            tail = url[4:] or 'index.html'
            return os.path.join(POTREE_DIR, tail.lstrip('/')), tail
        else:
            tail = url.lstrip('/') or 'index.html'
            return os.path.join(GALLERY_DIR, tail), tail

    def _api_scenes(self):
        """Return JSON list of all .copc.laz scene IDs in pointclouds/."""
        scenes = []
        if os.path.isdir(CLOUDS_DIR):
            for f in sorted(os.listdir(CLOUDS_DIR)):
                if f.endswith('.copc.laz'):
                    scenes.append(f[:-9])  # strip .copc.laz
        body = json.dumps(scenes).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self._add_cors()
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = unquote(self.path.split('?')[0])
        if url == '/api/scenes':
            self._api_scenes()
            return
        fs_path, _ = self._resolve_path()
        if os.path.isdir(fs_path):
            fs_path = os.path.join(fs_path, 'index.html')
        self._serve_file(fs_path, send_body=True)

    def do_HEAD(self):
        fs_path, _ = self._resolve_path()
        if os.path.isdir(fs_path):
            fs_path = os.path.join(fs_path, 'index.html')
        self._serve_file(fs_path, send_body=False)

    # ──────────────────────────────────────────────────────────────────────────
    def _serve_file(self, path, send_body=True):
        if not os.path.isfile(path):
            self.send_error(404, "File not found")
            return

        file_size = os.path.getsize(path)
        ctype = mimetypes.guess_type(path)[0] or "application/octet-stream"
        range_header = self.headers.get("Range")

        if not range_header:
            # Plain full-file response
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(file_size))
            self.send_header("Accept-Ranges", "bytes")
            self._add_cors()
            self.end_headers()
            if send_body:
                self._stream_file(path, 0, file_size)
            return

        # ── Parse Range: bytes=start-end ──────────────────────────────────────
        try:
            spec = range_header.strip().lower()
            if not spec.startswith("bytes="):
                raise ValueError("only bytes range supported")
            spec = spec[6:]  # strip "bytes="

            # Handle multi-range (just serve the first range; COPC only sends one)
            if "," in spec:
                spec = spec.split(",")[0].strip()

            if "-" not in spec:
                raise ValueError("invalid range spec")

            raw_start, raw_end = spec.split("-", 1)
            if not raw_start and not raw_end:
                raise ValueError("empty range")
            elif not raw_start:
                # suffix-range: bytes=-N  →  last N bytes
                n = int(raw_end)
                start = max(0, file_size - n)
                end   = file_size - 1
            elif not raw_end:
                # open-end: bytes=start-
                start = int(raw_start)
                end   = file_size - 1
            else:
                start = int(raw_start)
                end   = int(raw_end)

            # Clamp
            start = max(0, min(start, file_size - 1))
            end   = max(start, min(end, file_size - 1))
            length = end - start + 1

        except Exception as exc:
            self.send_error(400, "Bad Range header: " + str(exc))
            return

        self.send_response(206)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Range", f"bytes {start}-{end}/{file_size}")
        self.send_header("Content-Length", str(length))
        self.send_header("Accept-Ranges", "bytes")
        self._add_cors()
        self.end_headers()

        if send_body:
            self._stream_file(path, start, length)

    # ──────────────────────────────────────────────────────────────────────────
    def _stream_file(self, path, offset, length):
        try:
            with open(path, "rb") as f:
                f.seek(offset)
                remaining = length
                while remaining > 0:
                    chunk = f.read(min(65536, remaining))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)
        except (ConnectionAbortedError, BrokenPipeError):
            pass  # client disconnected — normal for streaming

    def _add_cors(self):
        """COPC fetch() calls need CORS headers when Origin differs."""
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Range")
        self.send_header("Access-Control-Expose-Headers", "Content-Range, Accept-Ranges")

    def do_OPTIONS(self):
        self.send_response(204)
        self._add_cors()
        self.end_headers()

    def log_message(self, fmt, *args):
        # Filter out noisy tile-load requests; keep errors
        msg = fmt % args
        if "206" in msg or "404" in msg or "500" in msg:
            super().log_message(fmt, *args)
        elif not any(ext in msg for ext in [".laz", ".json", ".js", ".css", ".woff"]):
            super().log_message(fmt, *args)


# ────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8081

    print()
    print("─" * 62)
    print("  LiDAR unified dev server")
    print(f"  Gallery  →  http://localhost:{port}/")
    print(f"  Potree   →  http://localhost:{port}/3d/")
    print(f"  Potree   →  http://localhost:{port}/3d/?scene=chamechaude-lit")
    print("─" * 62)
    print()

    server = HTTPServer(("", port), RangeHTTPRequestHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
