#!/usr/bin/env python3
"""ROS-free offline inference: class-mask sampling, staged metrics and full grasps.

Attribution is a position-based proxy, not a grasp success or Piper TCP check.
Sampling is seeded per item; CUDA extensions may still be nondeterministic.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

BASE = Path(__file__).resolve().parents[1]
STAGES = ('after_nms', 'width_filtered')


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def valid_depth(depth):
    return np.isfinite(depth) & (depth > 0.05) & (depth < 3.0)


def build_cloud(depth, k):
    if depth.ndim != 2 or not all(np.isfinite(k[v]) for v in ('fx', 'fy', 'cx', 'cy')):
        raise ValueError('Expected 2D metre depth and finite intrinsics')
    if k['fx'] <= 0 or k['fy'] <= 0:
        raise ValueError('Focal lengths must be positive')
    v, u = np.indices(depth.shape)
    points = np.stack([(u-k['cx'])*depth/k['fx'], (v-k['cy'])*depth/k['fy'], depth], -1)
    return points[valid_depth(depth)].astype(np.float32)


def voxel_downsample(points, voxel_m):
    if not len(points):
        return points
    keys = np.floor(points / voxel_m).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(index)]


def sample(points, count, rng):
    if not len(points):
        return points
    return points[rng.choice(len(points), count, replace=len(points) < count)]


def item_seed(seed, item_id):
    # Independent of discovery order and --limit; the two clouds use separate RNGs.
    return [seed, int.from_bytes(hashlib.sha256(item_id.encode()).digest()[:4], 'little')]


def postprocess(group, scene, params, detector):
    """Keep each processing stage; GraspGroup.nms returns a NEW group."""
    raw = group.grasp_group_array.copy()
    blocked = (detector(scene, voxel_size=params['collision_voxel']).detect(
        group, approach_dist=params['approach_dist'], collision_thresh=params['collision_thresh'])
        if len(group) else np.zeros(0, dtype=bool))
    group = group[~blocked]
    after_collision = group.grasp_group_array.copy()
    if len(group):
        group = group.nms()
        group.sort_by_score()
    after_nms = group.grasp_group_array.copy()
    width_filtered = after_nms[(after_nms[:, 1] > 0) &
                               (after_nms[:, 1] <= params['width_max'])]
    return dict(raw=raw, after_collision=after_collision, after_nms=after_nms,
                width_filtered=width_filtered)


def evaluate_stage(array, matrix, targets, tol, top_k=1):
    positions = array[:, 13:16] @ matrix[:3, :3].T + matrix[:3, 3]
    attribution = {}
    for body in targets:
        ranks = np.flatnonzero(np.all(np.abs(positions - body['center']) <= tol, axis=1))
        attribution[body['body']] = dict(top1=bool(len(ranks) and ranks[0] == 0),
                                       top5=bool(len(ranks) and ranks[0] < 5),
                                       best_rank=int(ranks[0]) if len(ranks) else None,
                                       # 同一目标的前 K 个归属候选（array 已按 score 降序）：
                                       # top-1 常常偏心或腕部不可达，执行端需要备选。
                                       best_ranks=[int(r) for r in ranks[:max(1, top_k)]])
    rotations = matrix[:3, :3] @ array[:, 4:13].reshape(-1, 3, 3)
    candidates = [dict(rank=i, reference_position_base=positions[i].tolist(),
                       reference_rotation_base=rotations[i].tolist(),
                       approach_base=rotations[i, :, 0].tolist(),
                       width=float(row[1]), score=float(row[0]),
                       height=float(row[2]), depth=float(row[3]))
                  for i, row in enumerate(array[:10])]
    widths = array[:, 1] * 1000
    approach = np.degrees(np.arccos(np.clip(-rotations[:, 2, 0], -1, 1)))
    # 抓取坐标系：+z=接近轴（R 第 0 列）、+x=开合轴（第 1 列）、+y=第 2 列（右手系保持）。
    proposals = []
    for body in targets:
        for rank in attribution[body['body']]['best_ranks']:
            grasp_frame = np.stack([rotations[rank][:, 1], rotations[rank][:, 2],
                                    rotations[rank][:, 0]], axis=1)
            proposals.append(dict(body=body['body'], rank=int(rank),
                                  position=positions[rank].tolist(),
                                  quaternion=Rotation.from_matrix(grasp_frame).as_quat().tolist(),
                                  width=float(array[rank, 1]), score=float(array[rank, 0])))
    return dict(attribution=attribution, candidates_top=candidates, proposals=proposals,
                width_mm=dict(min=float(widths.min()) if len(array) else None,
                              med=float(np.median(widths)) if len(array) else None,
                              max=float(widths.max()) if len(array) else None),
                depth_bins={f'{d:.2f}': int(np.isclose(array[:, 3], d, atol=1e-6, rtol=0).sum())
                            for d in (.01, .02, .03, .04)},
                approach_deg_med=float(np.median(approach)) if len(array) else None)


def load_model(weights, seed):
    # Heavy dependencies stay out of reporting and CPU regression checks.
    import torch
    for directory in ('models', 'utils', 'pointnet2', 'knn'):
        sys.path.insert(0, str(BASE/'third_party/graspnet-baseline'/directory))
    from graspnet import GraspNet, pred_decode
    from graspnetAPI import GraspGroup
    from collision_detector import ModelFreeCollisionDetector

    torch.manual_seed(seed)
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA unavailable; check the selected environment and GPU access')
    torch.cuda.manual_seed_all(seed)
    net = GraspNet(0, 300, 12, 4, .05, -.02, [.01, .02, .03, .04], is_training=False)
    checkpoint = torch.load(weights, map_location='cpu', weights_only=True)
    net.load_state_dict(checkpoint['model_state_dict'])
    net.cuda().eval()
    return torch, net, pred_decode, GraspGroup, ModelFreeCollisionDetector, checkpoint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--proposals', default='',
                        help='行式候选输出（ITEM/BEST，供 ROS 节点零依赖解析）；'
                             '姿态取 after_nms 阶段，参考点语义同 meta 说明')
    parser.add_argument('--weights', default=str(BASE/'models/artifacts/checkpoint-rs.tar'))
    parser.add_argument('--top-k', type=int, default=1, dest='top_k',
                       help='每个目标输出前 K 个归属候选（BEST 行）；ROS 节点用它做候选内选择')
    args = parser.parse_args()
    if not Path(args.weights).is_file():
        parser.error('Weights missing; see tools/graspnet/models/README.md or pass --weights')
    manifest = json.loads(Path(args.input).read_text())
    params, items = manifest['params'], manifest['items']
    if params['sample_mode'] not in ('scene_random', 'masked_random', 'masked_voxel'):
        parser.error('Unknown sample_mode')
    if (params['n_points'] < 1 or params['voxel_m'] <= 0 or params['collision_voxel'] <= 0 or
            params['width_max'] <= 0 or params.get('collision_points_max', 80000) < 1):
        parser.error('Point counts, voxel sizes and width limit must be positive')
    torch, net, decode, group_type, detector, checkpoint = load_model(args.weights, params['seed'])
    output = Path(args.out).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    arrays_dir = output.parent/'candidates'
    arrays_dir.mkdir(exist_ok=True)
    meta = dict(schema_version=2, evaluation_scope='class_mask',
                reference_frame_semantics='model grasp reference, NOT calibrated Piper TCP',
                weights=str(Path(args.weights).resolve()), weights_sha256=sha256(args.weights),
                epoch=int(checkpoint.get('epoch', -1)), torch=torch.__version__,
                numpy=np.__version__, params=params, n_items=len(items),
                warmup_forwards=2, stages=list(STAGES))
    results, warmed = [], False

    def save():
        output.write_text(json.dumps(dict(meta=meta, results=results), indent=1, allow_nan=False))

    save()
    for index, item in enumerate(items):
        identity = {k: item.get(k, '') for k in ('id', 'root', 'run', 'scenario')}
        identity['target_bodies'] = [t['body'] for t in item['targets']]
        seed = item_seed(params['seed'], item['id'])
        rng = np.random.default_rng(seed)
        rng_cloud = np.random.default_rng([*seed, 1])
        t0 = time.perf_counter()
        try:
            if 'depth_raw' in item:
                shape = item['shape']
                depth = np.fromfile(item['depth_raw'], dtype='<f4').reshape(
                    shape[0], shape[1]).astype(np.float32)
            else:
                depth = np.load(item['depth'], allow_pickle=False).astype(np.float32)
            labels = np.array(Image.open(item['labels']))
            if depth.shape != labels.shape:
                raise ValueError('Depth and class label shapes differ')
            matrix = np.asarray(item['M'], dtype=float)
            if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
                raise ValueError('Expected finite 4x4 base-from-camera matrix')
            full = build_cloud(depth, item['K'])
            masked = full[(labels == item['class'])[valid_depth(depth)]]
            points = (full if params['sample_mode'] == 'scene_random' else
                      voxel_downsample(masked, params['voxel_m'])
                      if params['sample_mode'] == 'masked_voxel' else masked)
            points = sample(points, params['n_points'], rng)
            if not len(points):
                raise ValueError('Empty input cloud')
        except (OSError, ValueError, KeyError) as error:
            results.append(dict(identity, error=str(error)))
            save()
            continue
        cloud_ms = (time.perf_counter()-t0)*1000
        pc = torch.from_numpy(points[None]).cuda()
        if not warmed:
            with torch.no_grad():
                for _ in range(2):
                    net({'point_clouds': pc})
            torch.cuda.synchronize()
            warmed = True
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            prediction = net({'point_clouds': pc})
        torch.cuda.synchronize()
        forward_ms = (time.perf_counter()-t0)*1000
        t0 = time.perf_counter()
        group = group_type(decode(prediction)[0].cpu().numpy())
        if params['collision_cloud'] == 'masked':
            scene = masked
        else:
            count = min(params.get('collision_points_max', 80000), len(full))
            scene = full[rng_cloud.choice(len(full), count, replace=False)]
        arrays = postprocess(group, scene, params, detector)
        stages = {stage: evaluate_stage(arrays[stage], matrix, item['targets'],
                                       params.get('attrib_tol_m', .02), args.top_k)
                  for stage in STAGES}
        post_ms = (time.perf_counter()-t0)*1000
        array_path = arrays_dir/f'{index:05d}.npz'
        # Preserve every candidate at native precision, full rotations and source transform.
        np.savez_compressed(array_path, **arrays, base_from_camera=matrix)
        results.append(dict(identity, sampling_seed=seed,
                            counts=dict(raw=len(arrays['raw']),
                                        collision_blocked=len(arrays['raw'])-len(arrays['after_collision']),
                                        after_collision=len(arrays['after_collision']),
                                        after_nms=len(arrays['after_nms']),
                                        width_pass=len(arrays['width_filtered'])),
                            stages=stages, candidate_arrays=str(array_path.relative_to(output.parent)),
                            timings_ms=dict(cloud=cloud_ms, forward=forward_ms, post=post_ms)))
        save()
        print(f'{index+1}/{len(items)} {item["id"]}: '
              f'NMS={len(arrays["after_nms"])} width={len(arrays["width_filtered"])}', flush=True)
    if args.proposals:
        lines = []
        for index, item in enumerate(items):
            result = results[index]
            if 'error' in result:
                continue
            stamp = float(item.get('stamp', 0.0))
            lines.append(f'ITEM {index} {stamp:.6f} {item["class"]}')
            for entry in result['stages']['after_nms']['proposals']:
                p, q = entry['position'], entry['quaternion']
                lines.append('BEST {} {} {} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} '
                             '{:.6f} {:.6f} {:.4f} {:.3f}'.format(
                                 index, entry['body'], entry['rank'],
                                 p[0], p[1], p[2], q[0], q[1], q[2], q[3],
                                 entry['width'], entry['score']))
        Path(args.proposals).write_text('\n'.join(lines) + '\n')
        best = sum(1 for line in lines if line.startswith('BEST '))
        print(f'wrote {args.proposals}: {best} proposals', flush=True)
    print(f'wrote {output}: {len(results)} items', flush=True)


if __name__ == '__main__':
    main()
