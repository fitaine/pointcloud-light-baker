#!/usr/bin/env python3
"""
Range-capable HTTP server for the Potree COPC viewer.

Python's built-in http.server ignores Range headers, which breaks the COPC
JavaScript library — it uses byte-range requests (fetch with Range: bytes=X-Y)
to read EVLRs and point data chunks from .copc.laz files without downloading
the full file. Without range support, Copc.create() throws and the viewer
gets stuck on the loading screen indefinitely.

Usage:  python server.py [port]   (default port 8081)
"""

import os, sys, mimetypes
from http.server import HTTPServer, SimpleHTTPRequestHandler


class RangeHTTPRequestHandler(SimpleHTTPRequestHandler):
    """SimpleHTTPRequestHandler extended with HTTP/1.1 Range request support."""

    server_version = "RangeHTTP/1.0"

    # ──────────────────────────────────────────────────────────────────────────
    def do_GET(self):
        path = self.translate_path(self.path.split('?')[0])
        if os.path.isdir(path):
            return SimpleHTTPRequestHandler.do_GET(self)
        self._serve_file(path, send_body=True)

    def do_HEAD(self):
        path = self.translate_path(self.path.split('?')[0])
        if os.path.isdir(path):
            return SimpleHTTPRequestHandler.do_HEAD(self)
        self._serve_file(path, send_body=False)

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

    # cd to the directory containing this script (potree/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    print()
    print("─" * 62)
    print("  LiDAR Point Cloud Viewer — Potree (range-capable server)")
    print(f"  http://localhost:{port}/")
    print(f"  http://localhost:{port}/?scene=chamechaude-lit")
    print(f"  http://localhost:{port}/?scene=grande-motte-lit")
    print(f"  http://localhost:{port}/?test=1   ← Potree sample COPC")
    print("─" * 62)
    print()

    server = HTTPServer(("", port), RangeHTTPRequestHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
