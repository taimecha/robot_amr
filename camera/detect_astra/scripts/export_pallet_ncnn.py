"""Export the trained weights for Pi inference without changing the source model."""
import argparse
from pathlib import Path
import shutil

from ultralytics import YOLO

PROJECT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--model", type=Path, default=PROJECT / "models/best_v3.pt")
parser.add_argument("--output-dir", type=Path, default=PROJECT / "data/runtime_models")
args = parser.parse_args()
if not args.model.is_file():
    parser.error(f"Missing model: {args.model}")
args.output_dir.mkdir(parents=True, exist_ok=True)
runtime_weights = args.output_dir / args.model.name
if runtime_weights.resolve() != args.model.resolve():
    shutil.copy2(args.model, runtime_weights)
print(YOLO(str(runtime_weights)).export(
    format="ncnn", imgsz=[480, 640], device="cpu", quantize=32))
