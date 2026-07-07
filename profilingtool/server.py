#!/usr/bin/env python3
"""Web dashboard for live ROS 2 node resource profiling in Docker."""

from __future__ import annotations

import argparse
import json
import mimetypes
from datetime import datetime, timezone
from functools import partial
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from collector import _use_local_collection, collect_snapshot


ROOT = Path(__file__).resolve().parent
STATIC_DIR = ROOT / "static"


class ProfilerHandler(BaseHTTPRequestHandler):
    container_name: str = "carkit"

    def log_message(self, format: str, *args) -> None:
        if self.path.startswith("/api/"):
            return
        super().log_message(format, *args)

    def _send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path: Path) -> None:
        if not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND, "File not found")
            return

        content = path.read_bytes()
        content_type, _ = mimetypes.guess_type(str(path))
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Content-Length", str(len(content)))
        if path.suffix in {".html", ".js", ".css"}:
            self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(content)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        route = parsed.path

        if route == "/api/metrics":
            try:
                snapshot = collect_snapshot(self.container_name)
                self._send_json(snapshot.to_dict())
            except Exception as exc:
                self._send_json(
                    {
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                        "container_name": self.container_name,
                        "container_running": False,
                        "errors": [str(exc)],
                        "nodes": [],
                        "launch_processes": [],
                        "ros2_nodes": [],
                    },
                    status=HTTPStatus.OK,
                )
            return

        if route in {"/", "/index.html"}:
            self._send_file(STATIC_DIR / "index.html")
            return

        if route.startswith("/static/"):
            relative = route.removeprefix("/static/")
            target = (STATIC_DIR / relative).resolve()
            if not str(target).startswith(str(STATIC_DIR.resolve())):
                self.send_error(HTTPStatus.FORBIDDEN, "Forbidden")
                return
            self._send_file(target)
            return

        self.send_error(HTTPStatus.NOT_FOUND, "Not found")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8765, help="HTTP port (default: 8765)")
    parser.add_argument("--container", default="carkit", help="Docker container name (default: carkit)")
    args = parser.parse_args()

    handler = partial(ProfilerHandler)
    handler.container_name = args.container

    server = ThreadingHTTPServer((args.host, args.port), handler)
    mode = "local (inside container)" if _use_local_collection() else f"docker exec -> {args.container}"
    print(f"ROS 2 profiler dashboard: http://{args.host}:{args.port}")
    print(f"Collection mode: {mode}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
