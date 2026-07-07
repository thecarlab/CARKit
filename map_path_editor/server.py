#!/usr/bin/env python3
"""Dependency-free web service for aligning reusable path shapes on occupancy maps."""

from __future__ import annotations

import argparse
import ast
import json
import mimetypes
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import quote, unquote, urlparse


ROOT = Path(__file__).resolve().parent
STATIC_DIR = ROOT / "static"
DEFAULT_MAP_DIR = ROOT.parent / "map"


class MapError(ValueError):
    """Raised when a map YAML or image file cannot be loaded."""


def _parse_scalar(raw_value: str) -> object:
    value = raw_value.split("#", 1)[0].strip()
    if not value:
        return ""
    if value.startswith("[") and value.endswith("]"):
        try:
            return ast.literal_eval(value)
        except (SyntaxError, ValueError) as exc:
            raise MapError(f"Invalid list value: {value}") from exc
    if (value.startswith('"') and value.endswith('"')) or (
        value.startswith("'") and value.endswith("'")
    ):
        return value[1:-1]
    for parser in (int, float):
        try:
            return parser(value)
        except ValueError:
            continue
    return value


def parse_map_yaml(yaml_path: Path) -> dict:
    config: dict[str, object] = {}
    try:
        lines = yaml_path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise MapError(f"Could not read {yaml_path.name}: {exc}") from exc

    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        config[key.strip()] = _parse_scalar(value)

    if "image" not in config:
        raise MapError(f"{yaml_path.name} does not define an image")
    if "resolution" not in config:
        raise MapError(f"{yaml_path.name} does not define a resolution")
    if "origin" not in config:
        raise MapError(f"{yaml_path.name} does not define an origin")

    origin = config["origin"]
    if not isinstance(origin, list) or len(origin) < 2:
        raise MapError(f"{yaml_path.name} origin must contain at least x and y")

    return config


def read_pgm_header(pgm_path: Path) -> tuple[str, int, int, int]:
    try:
        data = pgm_path.read_bytes()
    except OSError as exc:
        raise MapError(f"Could not read {pgm_path.name}: {exc}") from exc

    index = 0
    tokens: list[bytes] = []

    def skip_space_and_comments() -> None:
        nonlocal index
        while index < len(data):
            if data[index] == ord("#"):
                while index < len(data) and data[index] not in (10, 13):
                    index += 1
                continue
            if chr(data[index]).isspace():
                index += 1
                continue
            break

    while len(tokens) < 4:
        skip_space_and_comments()
        if index >= len(data):
            raise MapError(f"{pgm_path.name} has an incomplete PGM header")
        start = index
        while index < len(data) and not chr(data[index]).isspace():
            index += 1
        tokens.append(data[start:index])

    try:
        magic = tokens[0].decode("ascii")
        width = int(tokens[1])
        height = int(tokens[2])
        max_value = int(tokens[3])
    except (UnicodeDecodeError, ValueError) as exc:
        raise MapError(f"{pgm_path.name} has an invalid PGM header") from exc

    if magic not in {"P2", "P5"}:
        raise MapError(f"{pgm_path.name} is {magic}, expected P2 or P5 PGM")
    if width <= 0 or height <= 0 or max_value <= 0:
        raise MapError(f"{pgm_path.name} has invalid PGM dimensions")

    return magic, width, height, max_value


def _is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def resolve_yaml(map_dir: Path, map_id: str) -> Path:
    yaml_path = (map_dir / map_id).resolve()
    allowed_suffixes = {".yaml", ".yml"}
    if yaml_path.suffix.lower() not in allowed_suffixes:
        raise MapError("Map id must reference a YAML file")
    if not _is_relative_to(yaml_path, map_dir.resolve()):
        raise MapError("Map id is outside the configured map directory")
    if not yaml_path.is_file():
        raise MapError(f"Map {map_id} was not found")
    return yaml_path


def resolve_image(map_dir: Path, yaml_path: Path, config: dict) -> Path:
    image_name = str(config["image"])
    image_path = (yaml_path.parent / image_name).resolve()
    if not _is_relative_to(image_path, map_dir.resolve()):
        raise MapError(f"{yaml_path.name} image is outside the configured map directory")
    if not image_path.is_file():
        raise MapError(f"{yaml_path.name} references missing image {image_name}")
    return image_path


def map_metadata(map_dir: Path, yaml_path: Path) -> dict:
    config = parse_map_yaml(yaml_path)
    image_path = resolve_image(map_dir, yaml_path, config)
    magic, width, height, max_value = read_pgm_header(image_path)
    origin = list(config["origin"])

    return {
        "id": yaml_path.name,
        "yamlFile": yaml_path.name,
        "imageFile": str(config["image"]),
        "imageUrl": f"/api/maps/{quote(yaml_path.name)}/image",
        "format": magic,
        "width": width,
        "height": height,
        "maxValue": max_value,
        "resolution": float(config["resolution"]),
        "origin": [float(origin[0]), float(origin[1]), float(origin[2] if len(origin) > 2 else 0.0)],
        "mode": config.get("mode", "trinary"),
        "negate": int(config.get("negate", 0)),
        "occupiedThresh": float(config.get("occupied_thresh", 0.65)),
        "freeThresh": float(config.get("free_thresh", 0.25)),
    }


def discover_maps(map_dir: Path) -> dict:
    maps: list[dict] = []
    errors: list[str] = []
    for yaml_path in sorted([*map_dir.glob("*.yaml"), *map_dir.glob("*.yml")]):
        try:
            maps.append(map_metadata(map_dir, yaml_path))
        except MapError as exc:
            errors.append(str(exc))
    return {"maps": maps, "errors": errors}


class MapPathEditorHandler(BaseHTTPRequestHandler):
    map_dir: Path = DEFAULT_MAP_DIR.resolve()

    def log_message(self, format: str, *args) -> None:
        if self.path.startswith("/api/"):
            return
        super().log_message(format, *args)

    def _send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path: Path, content_type: str | None = None) -> None:
        if not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND, "File not found")
            return
        content = path.read_bytes()
        guessed_type, _ = mimetypes.guess_type(str(path))
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type or guessed_type or "application/octet-stream")
        self.send_header("Content-Length", str(len(content)))
        if path.suffix in {".html", ".js", ".css"}:
            self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(content)

    def _send_map_metadata(self, map_id: str) -> None:
        try:
            yaml_path = resolve_yaml(self.map_dir, map_id)
            self._send_json(map_metadata(self.map_dir, yaml_path))
        except MapError as exc:
            self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)

    def _send_map_image(self, map_id: str) -> None:
        try:
            yaml_path = resolve_yaml(self.map_dir, map_id)
            config = parse_map_yaml(yaml_path)
            image_path = resolve_image(self.map_dir, yaml_path, config)
            self._send_file(image_path, "image/x-portable-graymap")
        except MapError as exc:
            self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        route = parsed.path

        if route == "/api/maps":
            self._send_json(discover_maps(self.map_dir))
            return

        if route.startswith("/api/maps/"):
            tail = route.removeprefix("/api/maps/")
            if tail.endswith("/image"):
                self._send_map_image(unquote(tail.removesuffix("/image")))
                return
            self._send_map_metadata(unquote(tail))
            return

        if route in {"/", "/index.html"}:
            self._send_file(STATIC_DIR / "index.html")
            return

        if route.startswith("/static/"):
            relative = route.removeprefix("/static/")
            target = (STATIC_DIR / relative).resolve()
            if not _is_relative_to(target, STATIC_DIR.resolve()):
                self.send_error(HTTPStatus.FORBIDDEN, "Forbidden")
                return
            self._send_file(target)
            return

        self.send_error(HTTPStatus.NOT_FOUND, "Not found")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8010, help="HTTP port (default: 8010)")
    parser.add_argument(
        "--map-dir",
        default=str(DEFAULT_MAP_DIR),
        help=f"Directory containing Nav2 YAML/PGM maps (default: {DEFAULT_MAP_DIR})",
    )
    args = parser.parse_args()

    map_dir = Path(args.map_dir).expanduser().resolve()
    if not map_dir.is_dir():
        raise SystemExit(f"Map directory does not exist: {map_dir}")

    handler = type(
        "ConfiguredMapPathEditorHandler",
        (MapPathEditorHandler,),
        {"map_dir": map_dir},
    )

    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"Map path editor: http://{args.host}:{args.port}")
    print(f"Map directory: {map_dir}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
