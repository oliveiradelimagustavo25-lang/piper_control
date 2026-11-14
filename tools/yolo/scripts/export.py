#!/usr/bin/env python3
"""Export local segmentation weights to the C++ SDK contract and record provenance."""

import argparse
from datetime import datetime
import hashlib
from pathlib import Path
import re
import shutil

import yaml


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('weights', type=Path, help='Existing custom segmentation .pt file')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--output-dir', type=Path, default=root / 'models/artifacts')
    parser.add_argument('--manifest', type=Path, default=root / 'models/manifest.yaml')
    parser.add_argument('--name', help='Optional ONNX filename; defaults to training run, epochs, and time')
    parser.add_argument('--raw', action='store_true', help='Export one-to-many outputs with external NMS')
    args = parser.parse_args()
    if not args.weights.is_file() or args.imgsz < 32 or args.imgsz % 32:
        parser.error('Provide existing weights and a positive imgsz divisible by 32')
    import ultralytics
    model = ultralytics.YOLO(str(args.weights.resolve()))
    if model.task != 'segment':
        parser.error('The model must be an instance segmentation model')
    run_dir = args.weights.resolve().parent.parent
    run_args = run_dir / 'args.yaml'
    run_info = yaml.safe_load(run_args.read_text()) if run_args.is_file() else {}
    run_info = run_info or {}
    match = re.match(r'round_(\d+)_(\d{8}_\d{6})$', run_dir.name)
    if match:
        round_number, trained_at = int(match.group(1)), match.group(2)
    elif run_dir.name == 'first_round':
        round_number = 1
        trained_at = datetime.fromtimestamp(run_dir.stat().st_mtime).astimezone().strftime('%Y%m%d_%H%M%S')
    else:
        round_number = 0
        trained_at = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S')
    epochs = int(run_info.get('epochs', 0))
    filename = args.name or f'round_{round_number:02d}_e{epochs:03d}_{trained_at}.onnx'
    if not filename.endswith('.onnx') or Path(filename).name != filename:
        parser.error('--name must be a plain .onnx filename')
    export_args = {
        'format': 'onnx', 'imgsz': args.imgsz, 'batch': 1, 'dynamic': False,
        'half': False, 'simplify': False, 'opset': 18, 'device': 'cpu', 'nms': False,
    }
    if args.raw:
        export_args['end2end'] = False
    exported = Path(model.export(**export_args))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    destination = args.output_dir.resolve() / filename
    if destination.exists():
        parser.error(f'Will not overwrite existing model: {destination}')
    if exported.resolve() != destination:
        shutil.copy2(exported, destination)
    import onnx
    graph = onnx.load(str(destination)).graph
    if len(graph.input) != 1 or len(graph.output) != 2:
        raise RuntimeError('Unexpected segmentation export: expected one input and two outputs')
    prediction = [d.dim_value for d in graph.output[0].type.tensor_type.shape.dim]
    prototypes = [d.dim_value for d in graph.output[1].type.tensor_type.shape.dim]
    layout = 'end_to_end' if prediction[-1] == 6 + prototypes[1] else 'raw'
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest = {
        'file': str(destination.relative_to(root)) if destination.is_relative_to(root) else str(destination),
        'source_weights': args.weights.name,
        'training_round': round_number,
        'epochs': epochs,
        'trained_at': trained_at,
        'sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
        'ultralytics': ultralytics.__version__,
        'names': model.names, 'imgsz': args.imgsz, 'batch': 1, 'precision': 'FP32',
        'layout': layout, 'opset': 18,
    }
    encoded_manifest = yaml.safe_dump(manifest, sort_keys=False)
    args.manifest.with_name(destination.stem + '.yaml').write_text(encoded_manifest)
    args.manifest.write_text(encoded_manifest)
    print(destination)
    print(args.manifest.resolve())


if __name__ == '__main__':
    main()
