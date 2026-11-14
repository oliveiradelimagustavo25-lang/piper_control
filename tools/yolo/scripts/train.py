#!/usr/bin/env python3
"""Fine-tune YOLO26n-seg on the generated MuJoCo dataset."""
import argparse
import json
import os
from datetime import datetime
from pathlib import Path
import re

import yaml

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=ROOT / "tools/yolo/data/round_02/dataset.yaml")
    parser.add_argument("--weights", type=Path, default=ROOT / "tools/yolo/models/yolo26n-seg.pt")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="auto", help="auto, cpu, or a CUDA device index")
    parser.add_argument("--project", type=Path, default=ROOT / "tools/yolo/runs")
    parser.add_argument("--name", default="")
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    data_file, weights = args.data.resolve(), args.weights.resolve()
    if not data_file.is_file() or not weights.is_file():
        raise SystemExit(f"dataset or pretrained weights missing: {data_file}, {weights}")
    data = yaml.safe_load(data_file.read_text())
    classes = yaml.safe_load((ROOT / "src/my_robot_perception/config/object_classes.yaml").read_text())["classes"]
    expected = [entry["name"] for entry in sorted(classes, key=lambda item: item["id"])]
    if data.get("names") != expected:
        raise SystemExit(f"dataset class names do not match object_classes.yaml: {data.get('names')} != {expected}")

    # Keep Ultralytics caches, including its AMP-check weights, inside this tool.
    config_dir = ROOT / "tools/yolo/.deps/config"
    os.environ["YOLO_CONFIG_DIR"] = str(config_dir)
    settings_file = config_dir / "Ultralytics/settings.json"
    settings_file.parent.mkdir(parents=True, exist_ok=True)
    settings = json.loads(settings_file.read_text()) if settings_file.exists() else {}
    settings["weights_dir"] = str(ROOT / "tools/yolo/models")
    settings_file.write_text(json.dumps(settings, indent=2))

    import torch
    from ultralytics import YOLO

    device = args.device
    if device == "auto":
        device = "0" if torch.cuda.is_available() else "cpu"
    if args.name:
        run_name = args.name
    else:
        rounds = [int(match.group(1)) for path in args.project.glob("round_*")
                  if (match := re.match(r"round_(\d+)_", path.name))]
        if (args.project / "first_round").exists():
            rounds.append(1)
        run_name = f"round_{max(rounds, default=0) + 1:02d}_{datetime.now().astimezone():%Y%m%d_%H%M%S}"
    model = YOLO(str(weights))
    if model.task != "segment":
        raise SystemExit(f"expected segmentation weights, got task={model.task}")
    run_dir = args.project.resolve() / run_name
    if run_dir.exists():
        raise SystemExit(f"training run already exists: {run_dir}")
    model.train(
        data=str(data_file), epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
        device=device, workers=0 if device == "cpu" else 4, project=str(args.project),
        name=run_name, exist_ok=False, seed=args.seed, deterministic=True,
        patience=max(5, args.epochs // 3), cache=False, plots=True,
        degrees=180, translate=0.08, scale=0.35, fliplr=0.5, flipud=0.0,
        hsv_h=0.01, hsv_s=0.45, hsv_v=0.30, mosaic=0.5, close_mosaic=5,
        overlap_mask=True, mask_ratio=4,
    )
    print(f"best checkpoint: {run_dir / 'weights/best.pt'}")
    print(f"device: {device}; Ultralytics validation metrics and plots are in {run_dir}")


if __name__ == "__main__":
    main()
