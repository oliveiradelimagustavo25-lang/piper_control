"""将总配置的 scenes 颜色坐标列表转换为启动前 MJCF；由启动链自动调用。"""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def generate(scenes, output, world_dir):
    """未设置的颜色沿用原 cubes.xml；显式 [] 清空。返回完整场景路径。"""
    scenes = {} if scenes is None else scenes
    if not isinstance(scenes, dict):
        raise ValueError('scenes must map colors to lists of [x, y, z]')
    world = Path(world_dir).resolve()
    template = world / 'basic/preview.xml'
    defaults = ET.parse(world / 'objects/cubes.xml').getroot()
    bodies = {body.get('name').removesuffix('_cube'): body for body in defaults.findall('body')}
    unknown = set(scenes) - set(bodies)
    if unknown:
        raise ValueError(f'Unknown scenes colors: {sorted(unknown)}')
    objects = ET.Element('mujocoinclude')
    for color, original in bodies.items():
        positions = scenes.get(color)
        if positions is None:
            positions = [[float(x) for x in original.get('pos').split()]]
        if not isinstance(positions, list):
            raise ValueError(f'scenes.{color} requires a list of [x, y, z]')
        for i, position in enumerate(positions):
            if not isinstance(position, list) or len(position) != 3:
                raise ValueError(f'scenes.{color}[{i}] requires [x, y, z]')
            position = [float(x) for x in position]
            if not all(math.isfinite(x) for x in position):
                raise ValueError(f'Non-finite position in scenes.{color}[{i}]')
            body = copy.deepcopy(original)
            body.set('name', original.get('name') + (f'_{i + 1}' if i else ''))
            body.set('pos', ' '.join(map(str, position)))
            objects.append(body)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    ET.indent(objects)
    ET.ElementTree(objects).write(output / 'objects.xml', encoding='unicode')
    scene = ET.parse(template)
    for include in scene.iter('include'):
        filename = include.get('file')
        if filename == 'robot_description.xml':
            continue
        include.set('file', 'objects.xml' if filename == '../objects/cubes.xml'
                    else str((template.parent / filename).resolve()))
    for texture in scene.iter('texture'):
        if texture.get('file'):
            texture.set('file', str((template.parent / texture.get('file')).resolve()))
    ET.indent(scene)
    result = output / 'scene.xml'
    scene.write(result, encoding='unicode')
    return result
