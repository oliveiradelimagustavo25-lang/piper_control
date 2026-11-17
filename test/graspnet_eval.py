#!/usr/bin/env python3
"""GraspNet 离线评测驱动（系统 python 运行，负责 YAML→JSON 与汇总）。

流程：发现 log/scenarios 下全部快照 → 按 (输入模式) 生成 manifest → 调用
tools/graspnet/scripts/predict.py（conda graspnet 环境，子进程）→ 聚合为
summary.yaml / summary.md。输出默认写入 log/graspnet/<时间>/。

指标口径：top1/top5 是"目标归属"（三轴 ±tol 距真值中心），不代表抓取可执行；
分别报告 NMS 后与宽度过滤后的类别级结果；错误项计入总数。采样种子可复现，
不保证所有 CUDA 扩展逐位确定。
"""
import argparse
import datetime
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml
import cv2

ROOT = Path(__file__).resolve().parents[1]
PREDICT = ROOT / 'tools/graspnet/scripts/predict.py'
DEFAULT_PY = os.environ.get('GRASPNET_PYTHON')
STAGES = ('after_nms', 'width_filtered')
RUN_RE = re.compile(r'^(?P<scenario>.*?)_r\d+(?P<suffix>.*)$')


def visible_pixels(snap, cls):
    """目标类别在深度有效区内的像素数；不足说明该帧目标不可见（如 home 位帧）。"""
    depth = np.load(str(snap / 'depth.npy')).astype(np.float32)
    labels = cv2.imread(str(snap / 'masks.png'), cv2.IMREAD_GRAYSCALE)
    if labels is None:
        return 0
    valid = (depth > 0.05) & (depth < 3.0)
    return int(((labels == cls) & valid).sum())


def discover(roots, min_pixels):
    items = []
    skipped = [0]
    for root in sorted(Path(p) for p in roots):
        for batch in sorted(p for p in root.iterdir() if p.is_dir()):
            for run_dir in sorted(p for p in batch.iterdir() if (p / 'snapshots').is_dir()):
                match = RUN_RE.match(run_dir.name)
                scenario = match.group('scenario') if match else run_dir.name
                suffix = match.group('suffix') if match else ''
                for snap in sorted((run_dir / 'snapshots').glob('[0-9][0-9][0-9]')):
                    objects = yaml.safe_load((snap / 'objects.yaml').read_text()) or {}
                    truth = objects.get('truth') or {}
                    if not truth:
                        continue
                    info = yaml.safe_load((snap / 'camera_info.yaml').read_text())
                    trans = yaml.safe_load((snap / 'transforms.yaml').read_text())
                    k = info['K']
                    by_class = {}
                    for body, item in truth.items():
                        cls = int(body.split('_')[1])
                        by_class.setdefault(cls, []).append(
                            {'body': body, 'center': item['p']})
                    for cls, targets in sorted(by_class.items()):
                        pixels = visible_pixels(snap, cls)
                        if pixels < min_pixels:
                            skipped[0] += 1
                            continue
                        items.append({
                            'id': f'{batch.name}/{run_dir.name}/{snap.name}/class{cls}',
                            'root': batch.name, 'run': run_dir.name,
                            'scenario': scenario, 'suffix': suffix,
                            'depth': str((snap / 'depth.npy').resolve()),
                            'labels': str((snap / 'masks.png').resolve()),
                            'class': cls,
                            'K': {'fx': float(k[0]), 'fy': float(k[4]),
                                  'cx': float(k[2]), 'cy': float(k[5])},
                            'M': trans['base_from_camera_optical']['matrix'],
                            'targets': targets, 'mask_pixels': pixels,
                        })
    return items, skipped[0]


def run_mode(items, mode, args, out_dir):
    params = {
        'sample_mode': mode, 'n_points': args.n_points, 'seed': args.seed,
        'voxel_m': args.voxel_m, 'collision_cloud': args.collision_cloud,
        'collision_points_max': args.collision_points_max,
        'collision_voxel': 0.01, 'approach_dist': 0.05,
        'collision_thresh': 0.01,
        'width_max': args.width_max, 'attrib_tol_m': args.attrib_tol,
    }
    manifest_dir = out_dir / mode
    manifest_dir.mkdir(parents=True, exist_ok=True)
    manifest = manifest_dir / 'manifest.json'
    manifest.write_text(json.dumps({'params': params, 'items': items}, indent=1))
    results_path = manifest_dir / 'results.json'
    print(f'[{mode}] {len(items)} items -> {results_path}', flush=True)
    env = dict(os.environ, MPLCONFIGDIR=str(manifest_dir / '.mplcache'))
    env.setdefault('MPLBACKEND', 'Agg')
    python = [args.python] if args.python else ['conda', 'run', '--no-capture-output', '-n', 'graspnet', 'python']
    with (manifest_dir / 'predict.log').open('w') as log:
        proc = subprocess.run([*python, str(PREDICT), '--input', str(manifest),
                               '--out', str(results_path),
                               '--weights', str(args.weights)],
                              cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                              env=env, timeout=args.timeout)
    if proc.returncode or not results_path.is_file():
        sys.exit(f'{mode}: predict.py failed (exit {proc.returncode}), '
                 f'see {manifest_dir}/predict.log')
    return json.loads(results_path.read_text())


def aggregate(results, mode):
    if results.get('meta', {}).get('schema_version') != 2:
        raise ValueError('Expected schema_version=2; rerun inference instead of mixing old metrics')
    groups = {}
    for item in results['results']:
        key = (item.get('root', ''), item.get('scenario', ''))
        groups.setdefault(key, []).append(item)
    rows = []

    def mean(values, digits=3):
        return round(float(np.mean(values)), digits) if values else None

    for (root_name, scenario), entries in sorted(groups.items()):
        valid = [e for e in entries if 'error' not in e]
        for stage in STAGES:
            evaluated = [e['stages'][stage] for e in valid]
            top1 = sum(any(v['top1'] for v in e['attribution'].values()) for e in evaluated)
            covered = sum(sum(v['top5'] for v in e['attribution'].values()) /
                          max(1, len(e['attribution'])) for e in evaluated)
            widths = [e['width_mm']['med'] for e in evaluated if e['width_mm']['med'] is not None]
            count_key = 'after_nms' if stage == 'after_nms' else 'width_pass'
            rows.append(dict(root=root_name, scenario=scenario, stage=stage,
                             n_items=len(entries), n_valid=len(valid),
                             n_errors=len(entries)-len(valid),
                             n_no_candidates=sum(e['counts'][count_key] == 0 for e in valid),
                             top1_rate=round(top1/len(entries), 3),
                             top5_coverage=round(covered/len(entries), 3),
                             width_med_mm=round(float(np.median(widths)), 1) if widths else None,
                             blocked_rate=mean([e['counts']['collision_blocked'] /
                                                max(1, e['counts']['raw']) for e in valid]),
                             cloud_ms=mean([e['timings_ms']['cloud'] for e in valid], 1),
                             forward_ms=mean([e['timings_ms']['forward'] for e in valid], 1),
                             post_ms=mean([e['timings_ms']['post'] for e in valid], 1)))
    return rows


def write_summary(out_dir, tables, attrib_tol_mm, skipped=0):
    lines = ['# GraspNet 离线对照评测（schema v2）', '',
             f'类别级评估：top1/top5 为模型参考点距目标中心 ±{attrib_tol_mm:.0f} mm/轴的空间归属代理指标，'
             '不代表实例关联、Piper TCP 或抓取可执行性。', '',
             '分别报告 after_nms（宽度过滤前）与 width_filtered（过滤后）。'
             'top1 分母为全部样本；top5 为每样本目标覆盖比例的均值；错误项按 0 计入。', '',
             '错误数与无候选数分列；无候选数只统计未报错样本。宽度列为每样本宽度中位数的中位数。'
             '时间为有效项均值，不含模型加载、预热和候选文件写入。', '',
             f'发现阶段跳过 {skipped} 个类别掩码有效深度像素不足的快照-类别组合；'
             '这些项不参与上述条件指标，不能据此推断完整系统识别成功率。', '']
    for mode, rows in tables:
        lines.extend([f'## 输入模式：{mode}', '',
                      '| 根目录 | 场景 | 阶段 | 总数 | 有效 | 错误 | 无候选 | top1归属 | top5覆盖 | 宽度中位(mm) | 碰撞淘汰 | 点云(ms) | 前向(ms) | 后处理(ms) |',
                      '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |'])
        for row in rows:
            lines.append('| {root} | {scenario} | {stage} | {n_items} | {n_valid} | {n_errors} | '
                         '{n_no_candidates} | {top1_rate} | {top5_coverage} | {width_med_mm} | '
                         '{blocked_rate} | {cloud_ms} | {forward_ms} | {post_ms} |'.format(**row))
        lines.append('')
    (out_dir / 'summary.md').write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--roots', nargs='+', type=Path,
                        default=[ROOT / 'log/scenarios'])
    parser.add_argument('--inputs', default='masked_random,masked_voxel,scene_random')
    parser.add_argument('--python', default=DEFAULT_PY,
                        help='模型 Python 路径；默认 GRASPNET_PYTHON，未设置则 conda run -n graspnet')
    parser.add_argument('--timeout', type=float, default=1800, help='单模式子进程超时秒数')
    parser.add_argument('--weights', default=ROOT / 'tools/graspnet/models/artifacts/checkpoint-rs.tar')
    parser.add_argument('--out', type=Path, default=None)
    parser.add_argument('--limit', type=int, default=0, help='限制快照数（冒烟用）')
    parser.add_argument('--seed', type=int, default=20261117)
    parser.add_argument('--n-points', type=int, default=20000)
    parser.add_argument('--voxel-m', type=float, default=0.001)
    parser.add_argument('--collision-cloud', default='scene', choices=['scene', 'masked'])
    parser.add_argument('--collision-points-max', type=int, default=80000)
    parser.add_argument('--width-max', type=float, default=0.07, help='夹爪硬限位，仅过滤用')
    parser.add_argument('--attrib-tol', type=float, default=0.02)
    parser.add_argument('--min-pixels', type=int, default=50,
                        help='目标类别掩码∩有效深度少于该像素数的帧跳过（目标不可见）')
    args = parser.parse_args()

    modes = [m.strip() for m in args.inputs.split(',') if m.strip()]
    if not modes or any(m not in ('scene_random', 'masked_random', 'masked_voxel') for m in modes):
        parser.error('Unknown or empty --inputs')
    if (args.n_points < 1 or args.voxel_m <= 0 or args.width_max <= 0 or
            args.collision_points_max < 1 or args.timeout <= 0 or args.attrib_tol <= 0 or
            args.min_pixels < 0 or args.limit < 0):
        parser.error('Invalid point count, threshold, limit or timeout')
    args.weights = Path(args.weights).resolve()
    if not args.weights.is_file():
        parser.error('Weights missing: see tools/graspnet/models/README.md or pass --weights')
    if not args.python and not shutil.which('conda'):
        parser.error('Set --python /path/to/graspnet/python or provide conda on PATH')
    args.roots = [p.resolve() for p in args.roots]
    if any(not p.is_dir() for p in args.roots):
        parser.error('Snapshot root missing; provide --roots pointing to collected scenarios')
    items, skipped = discover(args.roots, args.min_pixels)
    if args.limit:
        seen, kept = set(), []
        for item in items:
            key = (item['root'], item['run'], item['depth'])
            if key in seen:
                continue
            seen.add(key)
            kept.append([i for i in items if (i['root'], i['run'], i['depth']) == key])
            if len(seen) >= args.limit:
                break
        items = [i for group in kept for i in group]
    if not items:
        sys.exit('no snapshots discovered')
    stamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    out_dir = (args.out or (ROOT / 'log/graspnet' / stamp)).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f'{len(items)} items（另跳过 {skipped} 个无可见目标的快照-类别组合）-> {out_dir}', flush=True)

    tables = []
    for mode in modes:
        results = run_mode(items, mode, args, out_dir)
        rows = aggregate(results, mode)
        tables.append((mode, rows))

    write_summary(out_dir, tables, args.attrib_tol * 1000, skipped)
    summary = {mode: [{k: v for k, v in row.items() if not k.startswith('_')} for row in rows]
               for mode, rows in tables}
    (out_dir / 'summary.yaml').write_text(yaml.safe_dump(summary, sort_keys=False,
                                                         allow_unicode=True))
    print((out_dir / 'summary.md').read_text())
    print(f'输出目录：{out_dir}')


if __name__ == '__main__':
    main()
