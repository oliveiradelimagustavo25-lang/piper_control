"""按共同类别表生成 MuJoCo 场景；缺失/不支持的类别只跳过自身。"""
from collections import Counter
import json
import math
from pathlib import Path
import warnings
import xml.etree.ElementTree as ET

import yaml


def class_id(value):
    """YAML 整数键和字符串键使用同一个 ID；0 保留给背景。"""
    if isinstance(value, bool) or not str(value).isdigit() or not 1 <= int(value) <= 255:
        raise ValueError(f'Invalid class ID: {value!r}')
    return int(value)


def catalog_file(perception_config, override=''):
    if override:
        return Path(override).expanduser().resolve(strict=True)
    config = Path(perception_config).expanduser().resolve(strict=True)
    data = yaml.safe_load(config.read_text())
    value = data.get('/**', {}).get('ros__parameters', {}).get('class_config', 'object_classes.yaml')
    return (config.parent / value).resolve(strict=True)


def load_catalog(path):
    entries = yaml.safe_load(Path(path).read_text())['classes']
    if not isinstance(entries, list):
        raise ValueError('class_config requires a classes list')
    ids, names = Counter(), Counter()
    for entry in entries:
        try:
            ids[class_id(entry['id'])] += 1
            if isinstance(entry.get('name'), str):
                names[entry['name']] += 1
        except (KeyError, TypeError, ValueError):
            pass
    result = {}
    for entry in entries:
        try:
            cid = class_id(entry['id'])
            name = entry['name']
            geometry = entry['geometry']
            dimensions = geometry['dimensions']
            if ids[cid] != 1 or not isinstance(name, str) or not name or names[name] != 1:
                raise ValueError('duplicate/invalid class ID or name')
            if not isinstance(geometry['type'], str) or not geometry['type']:
                raise ValueError('missing geometry type')
            if (not isinstance(dimensions, list) or len(dimensions) != 3 or
                    any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v <= 0 for v in dimensions)):
                raise ValueError('dimensions must be three positive finite lengths')
            result[cid] = entry
        except (KeyError, TypeError, ValueError) as error:
            warnings.warn(f'Skipping invalid class {entry!r}: {error}', stacklevel=2)
    return result


def generate(scenes, output, world_dir, class_config):
    """None 使用类别表默认位置；显式映射仅生成列出的类别。返回场景路径。"""
    catalog = load_catalog(class_config)
    if scenes is None:
        scenes = {cid: [c['simulation']['default_position']] for cid, c in catalog.items()
                  if 'default_position' in c.get('simulation', {})}
    if not isinstance(scenes, dict):
        raise ValueError('scenes must map class IDs to lists of [x, y, z]')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = {'requested': {}, 'generated': {}, 'skipped': [], 'bodies': {}}
    objects = ET.Element('mujocoinclude')
    normalized = []
    for key, positions in scenes.items():
        try:
            normalized.append((class_id(key), positions))
        except ValueError as error:
            report['skipped'].append({'class_id': str(key), 'reason': str(error)})
    counts = Counter(cid for cid, _ in normalized)
    colors = {'red': [1, 0, 0, 1], 'green': [0, 1, 0, 1], 'blue': [0, 0, 1, 1]}
    for cid, positions in normalized:
        report['requested'][str(cid)] = report['requested'].get(str(cid), 0) + (len(positions) if isinstance(positions, list) else 0)
        try:
            if counts[cid] != 1:
                raise ValueError('duplicate normalized class ID')
            if cid not in catalog:
                raise ValueError('class is not defined in object_classes')
            if not isinstance(positions, list):
                raise ValueError('expected a list of [x, y, z]')
            entry = catalog[cid]
            geometry = entry['geometry']
            half = [float(v) / 2 for v in geometry['dimensions']]
            shape = geometry['type']
            if shape not in ('box', 'cylinder', 'sphere'):
                raise ValueError(f'unsupported MuJoCo geometry: {shape}')
            if shape in ('cylinder', 'sphere') and not math.isclose(half[0], half[1]):
                raise ValueError('round geometry requires equal X/Y diameters')
            if shape == 'sphere' and not math.isclose(half[0], half[2]):
                raise ValueError('sphere requires equal diameters')
            size = {'box': half, 'cylinder': [half[0], half[2]], 'sphere': [half[0]]}[shape]
            simulation = entry.get('simulation', {})
            mass = float(simulation.get('mass', 0.001))
            rgba = simulation.get('rgba', colors.get(entry.get('color'), [0.7, 0.7, 0.7, 1]))
            if mass <= 0 or not math.isfinite(mass) or len(rgba) != 4 or not all(0 <= float(v) <= 1 for v in rgba):
                raise ValueError('invalid simulation mass/rgba')
            report['generated'][str(cid)] = 0
            for index, position in enumerate(positions, 1):
                if (not isinstance(position, list) or len(position) != 3 or
                        any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in position)):
                    report['skipped'].append({'class_id': cid, 'instance': index, 'reason': 'invalid position'})
                    continue
                name = f'class_{cid}_{index}'
                body = ET.SubElement(objects, 'body', name=name, pos=' '.join(map(str, position)))
                ET.SubElement(body, 'freejoint')
                ET.SubElement(body, 'geom', type=shape, size=' '.join(map(str, size)),
                              mass=str(mass), rgba=' '.join(map(str, rgba)))
                report['generated'][str(cid)] += 1
                report['bodies'][name] = cid
        except (KeyError, TypeError, ValueError) as error:
            report['skipped'].append({'class_id': cid, 'reason': str(error)})
    for skipped in report['skipped']:
        warnings.warn(f'Scene skipped: {skipped}', stacklevel=2)
    (output / 'scene_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"Scene requested={report['requested']}, generated={report['generated']}, skipped={len(report['skipped'])}", flush=True)
    ET.indent(objects)
    ET.ElementTree(objects).write(output / 'objects.xml', encoding='unicode')
    template = Path(world_dir).resolve() / 'basic/preview.xml'
    scene = ET.parse(template)
    for include in scene.iter('include'):
        filename = include.get('file')
        if filename != 'robot_description.xml':
            include.set('file', 'objects.xml' if filename == '../objects/cubes.xml'
                        else str((template.parent / filename).resolve()))
    for texture in scene.iter('texture'):
        if texture.get('file'):
            texture.set('file', str((template.parent / texture.get('file')).resolve()))
    ET.indent(scene)
    result = output / 'scene.xml'
    scene.write(result, encoding='unicode')
    return result
