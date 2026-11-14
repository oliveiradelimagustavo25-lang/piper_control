#!/usr/bin/env python3
"""Render a labeled YOLO-seg dataset from the project's MuJoCo world."""
import argparse
import json
import math
import random
import sys
from collections import Counter
from pathlib import Path

import cv2
import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))

from my_robot_world.scenes.scene import build_scene

WORLD = ROOT / "src/my_robot_world"
DESCRIPTION = ROOT / "src/my_robot_description"
POSE_1 = {"joint1": 0.0, "joint2": 1.3006, "joint3": -0.6496,
          "joint4": 0.0103, "joint5": 0.7436, "joint6": 0.0}
RGB = {"red": (1, 0, 0, 1), "green": (0, 1, 0, 1),
       "blue": (0, 0, 1, 1), "yellow": (1, 1, 0, 1)}


def catalog(path):
    classes = yaml.safe_load(path.read_text())["classes"]
    classes = sorted(classes, key=lambda c: int(c["id"]))
    for index, entry in enumerate(classes):
        entry["index"] = index
        dims = entry["geometry"]["dimensions"]
        if len(dims) != 3 or min(dims) <= 0:
            raise ValueError(f"invalid dimensions for {entry['name']}")
    return classes


def add_objects(spec, classes, rng, max_per_class, cylinder_weight):
    for body in list(spec.worldbody.bodies):
        if body.name in {"red_cube", "green_cube", "blue_cube"}:
            spec.delete(body)

    # Match the visible workspace around the project's current default objects.
    placed, instances = [], []
    remaining = classes[:]
    order = []
    for _ in range(rng.randint(1, min(3, len(classes)))):
        weights = [cylinder_weight if cls["geometry"]["type"] == "cylinder" else 1.0
                   for cls in remaining]
        selected = rng.choices(remaining, weights=weights, k=1)[0]
        order.append(selected)
        remaining.remove(selected)
    for cls in order:
        count = rng.randint(1, max_per_class)
        dx, dy, height = map(float, cls["geometry"]["dimensions"])
        for number in range(count):
            for _ in range(120):
                x, y = rng.uniform(0.30, 0.34), rng.uniform(-0.08, 0.08)
                yaw = rng.uniform(-math.pi, math.pi)
                # Conservative rotated footprint; the first round avoids overlaps.
                half = (max(dx, dy) / 2 + 0.004)
                if all(abs(x - ox) >= half + ohx or abs(y - oy) >= half + ohy
                       for ox, oy, ohx, ohy in placed):
                    break
            else:
                continue
            body_name = f"class_{cls['id']}_{number + 1}"
            body = spec.worldbody.add_body(name=body_name, pos=[x, y, height / 2 + 0.0005])
            body.add_freejoint()
            geom = body.add_geom(name=f"{body_name}_geom", mass=0.001,
                                 rgba=cls.get("simulation", {}).get("rgba") or RGB.get(cls["color"], (0.7, 0.7, 0.7, 1)))
            if cls["geometry"]["type"] == "box":
                geom.type = mujoco.mjtGeom.mjGEOM_BOX
                geom.size = [dx / 2, dy / 2, height / 2]
            elif cls["geometry"]["type"] == "cylinder":
                if abs(dx - dy) > 1e-8:
                    raise ValueError(f"{cls['name']} cylinder X/Y diameters differ")
                geom.type = mujoco.mjtGeom.mjGEOM_CYLINDER
                geom.size = [dx / 2, height / 2, 0]
            else:
                raise ValueError(f"unsupported training geometry: {cls['geometry']['type']}")
            # Rotate free bodies after compile by setting their free-joint quaternion.
            placed.append((x, y, half, half))
            instances.append((cls, body_name, yaw))
    return instances


def render(classes, rng, max_per_class, cylinder_weight, empty=False):
    spec = build_scene(WORLD, DESCRIPTION)
    instances = [] if empty else add_objects(spec, classes, rng, max_per_class, cylinder_weight)
    model = spec.compile()
    data = mujoco.MjData(model)
    for joint, angle in POSE_1.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        data.qpos[model.jnt_qposadr[jid]] = angle + rng.uniform(-0.01, 0.01)
    for cls, name, yaw in instances:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{name}_free")
        qpos = model.jnt_qposadr[jid]
        data.qpos[qpos + 3:qpos + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    mujoco.mj_forward(model, data)
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_camera")
    renderer = mujoco.Renderer(model, height=600, width=800)
    renderer.disable_segmentation_rendering()
    renderer.update_scene(data, camera=camera_id)
    rgb = renderer.render().copy()
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera=camera_id)
    seg = renderer.render().copy()
    renderer.close()
    geom_ids = {}
    for geom_id in range(model.ngeom):
        body = int(model.geom_bodyid[geom_id])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
        if name and name.startswith("class_"):
            geom_ids[name] = geom_id
    return rgb, seg[..., 0], geom_ids, instances


def polygon(mask):
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 12:
        return None
    contour = cv2.approxPolyDP(contour, max(1.0, 0.002 * cv2.arcLength(contour, True)), True)
    points = contour.reshape(-1, 2).astype(np.float32)
    if len(points) < 3:
        return None
    points[:, 0] = np.clip(points[:, 0] / 800, 0, 1)
    points[:, 1] = np.clip(points[:, 1] / 600, 0, 1)
    return points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=ROOT / "src/my_robot_perception/config/object_classes.yaml")
    parser.add_argument("--output", type=Path, default=ROOT / "tools/yolo/data/round_02")
    parser.add_argument("--train", type=int, default=1200)
    parser.add_argument("--val", type=int, default=300)
    parser.add_argument("--max-per-class", type=int, default=3)
    parser.add_argument("--cylinder-weight", type=float, default=4.0,
                        help="relative sampling weight for cylinder classes")
    parser.add_argument("--empty-fraction", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    classes = catalog(args.catalog.resolve())
    names = [cls["name"] for cls in classes]
    output = args.output.resolve()
    if output.exists() and not args.force:
        raise SystemExit(f"{output} exists; use --force to replace it")
    if output.exists():
        import shutil
        shutil.rmtree(output)
    for split in ("train", "val"):
        (output / "images" / split).mkdir(parents=True, exist_ok=True)
        (output / "labels" / split).mkdir(parents=True, exist_ok=True)

    rng = random.Random(args.seed)
    plan = ["train"] * args.train + ["val"] * args.val
    rng.shuffle(plan)
    counts = {split: Counter() for split in ("train", "val")}
    samples = []
    for index, split in enumerate(plan):
        empty = rng.random() < args.empty_fraction
        rgb, seg, geom_ids, instances = render(
            classes, rng, args.max_per_class, args.cylinder_weight, empty)
        rows = []
        visible = []
        for cls, name, _ in instances:
            geom_id = geom_ids.get(name)
            if geom_id is None:
                continue
            points = polygon(seg == geom_id)
            if points is None:
                continue
            rows.append(" ".join([str(cls["index"])] + [f"{v:.6f}" for v in points.reshape(-1)]))
            counts[split][cls["name"]] += 1
            visible.append(cls["name"])
        stem = f"{split}_{index:05d}"
        cv2.imwrite(str(output / "images" / split / f"{stem}.jpg"),
                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 92])
        (output / "labels" / split / f"{stem}.txt").write_text("\n".join(rows) + ("\n" if rows else ""))
        if index < 12:
            preview = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            for row in rows:
                fields = row.split()
                points = np.array(fields[1:], dtype=np.float32).reshape(-1, 2)
                points *= [800, 600]
                cv2.polylines(preview, [points.astype(np.int32)], True, (0, 255, 255), 1)
                cv2.putText(preview, names[int(fields[0])], tuple(points[0].astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            (output / "preview").mkdir(exist_ok=True)
            cv2.imwrite(str(output / "preview" / f"{stem}.jpg"), preview)
        samples.append({"split": split, "image": f"images/{split}/{stem}.jpg", "instances": visible})
        if (index + 1) % 25 == 0 or index + 1 == len(plan):
            print(f"rendered {index + 1}/{len(plan)}", flush=True)

    (output / "dataset.yaml").write_text(yaml.safe_dump({
        "path": str(output), "train": "images/train", "val": "images/val", "names": names,
    }, sort_keys=False))
    report = {"seed": args.seed, "samples": len(plan), "camera": "MuJoCo wrist_camera at pose_1 +/- 0.01 rad",
              "cylinder_sampling_weight": args.cylinder_weight,
              "classes": [{"id": c["id"], "index": c["index"], "name": c["name"]} for c in classes],
              "instances": {split: dict(counts[split]) for split in counts}, "images": samples,
              "empty_fraction_requested": args.empty_fraction,
              "annotation_check": "per-instance MuJoCo geom segmentation; first 12 overlays in preview/"}
    (output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps({"dataset": str(output / "dataset.yaml"), "instances": report["instances"]},
                     indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
