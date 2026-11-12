"""放置规则校验、YAML 优先级合并与 dry-run 描述；不进行物体坐标猜测。"""
import math

import yaml

COLORS = ('red', 'green', 'blue')
KEYS = ('place_position_modes', 'place_positions')


def parameters(config):
    result = {}
    if 'place_position_modes' in config:
        modes = config['place_position_modes']
        if not isinstance(modes, list) or len(modes) != 3 or any(m not in ('relative', 'absolute') for m in modes):
            raise ValueError('place_position_modes requires three X/Y/Z entries: relative or absolute')
        result['place_position_modes'] = modes
    if 'place_positions' in config:
        positions = config['place_positions']
        if not isinstance(positions, dict) or set(positions) != set(COLORS):
            raise ValueError('place_positions requires red, green and blue arrays; null/missing colors are not allowed')
        for color in COLORS:
            values = positions[color]
            if (not isinstance(values, list) or len(values) != 3 or
                    any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in values)):
                raise ValueError(f'place_positions.{color} requires three finite center coordinates/offsets (not null)')
            result['place_positions.' + color] = [float(v) for v in values]
    return result


def resolve(planner, modes_argument='', positions_argument=''):
    """节点 YAML < 总配置（由 launch 默认参数传入）< 显式 launch 参数。"""
    result = dict(planner)
    overrides = {}
    for key, value in zip(KEYS, (modes_argument, positions_argument)):
        if value:
            overrides[key] = yaml.safe_load(value)
    # 支持节点 YAML 的标准嵌套形式，也支持点分参数名。先合并高优先级值再校验。
    has_nested = 'place_positions' in result
    nested = result.pop('place_positions', None)
    if has_nested and 'place_positions' not in overrides:
        if not isinstance(nested, dict):
            raise ValueError('Invalid planner place_positions mapping')
        for color, value in nested.items():
            result.setdefault('place_positions.' + color, value)
    result.update(parameters(overrides))
    effective = {}
    if 'place_position_modes' in result:
        effective['place_position_modes'] = result['place_position_modes']
    if any('place_positions.' + color in result for color in COLORS):
        effective['place_positions'] = {color: result.get('place_positions.' + color) for color in COLORS}
    result.update(parameters(effective))
    if 'place_positions' not in effective and any(m == 'relative' for m in result.get('place_position_modes', [])):
        raise ValueError('Relative modes require explicit place_positions; legacy simulation points are not offsets')
    return result


def describe(planner, frame):
    if not any('place_positions.' + c in planner for c in COLORS):
        return [f'Placement: legacy region/slots in {frame}; no place_positions override']
    modes = planner.get('place_position_modes', ['absolute'] * 3)
    lines = [f'Placement frame: {frame}; X/Y/Z modes: {modes}']
    for color in COLORS:
        values = planner['place_positions.' + color]
        axes = [f'initial_{axis} {value:+g}' if mode == 'relative' else f'{value:g}'
                for axis, value, mode in zip('xyz', values, modes)]
        lines.append(f'{color} object center = [{", ".join(axes)}]')
    if 'relative' in modes:
        lines.append('Relative targets need live observations; Planner prints frozen absolute targets before dispatch.')
    return lines
