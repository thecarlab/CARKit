#!/usr/bin/env python3

import argparse
import hashlib
import json
import platform
import shutil
from datetime import datetime, timezone
from pathlib import Path

import tensorrt
import torch
import ultralytics
from ultralytics import YOLO


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export a batch-one FP16 TensorRT engine on the target Jetson."
    )
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument(
        "--output-dir",
        default=Path(
            "/workspaces/CARKit/carkit/perception/carkit_perception/models"
        ),
        type=Path,
    )
    parser.add_argument("--name", default="yolo11n_fp16.engine")
    parser.add_argument("--image-size", default=640, type=int)
    args = parser.parse_args()

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required to export the FP16 TensorRT engine")
    if not args.source.is_file():
        raise FileNotFoundError(args.source)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    exported = Path(
        YOLO(str(args.source)).export(
            format="engine",
            half=True,
            imgsz=args.image_size,
            batch=1,
            dynamic=False,
            device=0,
            simplify=False,
        )
    )
    engine_path = args.output_dir / args.name
    if exported.resolve() != engine_path.resolve():
        shutil.move(str(exported), engine_path)

    metadata = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_model": str(args.source.resolve()),
        "source_model_sha256": sha256(args.source),
        "engine_sha256": sha256(engine_path),
        "precision": "FP16",
        "batch": 1,
        "image_size": args.image_size,
        "dynamic": False,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "versions": {
            "cuda": torch.version.cuda,
            "pytorch": torch.__version__,
            "tensorrt": tensorrt.__version__,
            "ultralytics": ultralytics.__version__,
        },
    }
    engine_path.with_suffix(".json").write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )
    print(engine_path)


if __name__ == "__main__":
    main()
