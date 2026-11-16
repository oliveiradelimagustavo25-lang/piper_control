"""逐类别放置规则与 YAML 优先级；显式缺失规则不会回退旧坐标。"""
import math
import warnings

import yaml

KEYS = ('place_position_modes', 'place_positions', 'place_regions')


def class_id(key, table):
    """类别键必须唯一映射到 1..255；同一 ID 的重复写法（1 与 '1'）一律拒绝。"""
    cid = str(key)
    if (isinstance(key, bool) or not cid.isdigit() or not 1 <= int(cid) <= 255 or
            sum(str(k).isdigit() and int(str(k)) == int(cid) for k in table) != 1):
        return None
    return int(cid)


def parameters(config):
    result = {}
    if 'place_position_modes' in config:
        modes = config['place_position_modes']
        if not isinstance(modes, list) or len(modes) != 3 or any(m not in ('relative', 'absolute') for m in modes):
            raise ValueError('place_position_modes requires three X/Y/Z entries: relative or absolute')
        result['place_position_modes'] = modes
    if 'place_positions' not in config and 'place_regions' not in config:
        return result
    result['use_position_rules'] = True
    point_ids = set()
    if 'place_positions' in config:
        positions = config['place_positions']
        if not isinstance(positions, dict):
            warnings.warn('Ignoring invalid place_positions; no legacy fallback', stacklevel=2)
        else:
            for key, values in positions.items():
                cid = class_id(key, positions)
                if cid is None:
                    warnings.warn(f'Ignoring invalid/duplicate placement class {key!r}', stacklevel=2)
                    continue
                if (not isinstance(values, list) or len(values) != 3 or
                        any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in values)):
                    warnings.warn(f'Ignoring placement class {cid}: expected three finite coordinates', stacklevel=2)
                    continue
                result['place_positions.' + str(cid)] = [float(v) for v in values]
                point_ids.add(cid)
    if 'place_regions' in config:
        regions = config['place_regions']
        if not isinstance(regions, dict):
            warnings.warn('Ignoring invalid place_regions; no legacy fallback', stacklevel=2)
        else:
            for key, values in regions.items():
                cid = class_id(key, regions)
                if cid is None:
                    warnings.warn(f'Ignoring invalid/duplicate placement region {key!r}', stacklevel=2)
                    continue
                if (not isinstance(values, list) or len(values) != 5 or
                        any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in values)):
                    warnings.warn(f'Ignoring placement region {cid}: expected five finite values', stacklevel=2)
                    continue
                xmin, ymin, xmax, ymax, z = (float(v) for v in values)
                if xmin > xmax or ymin > ymax:
                    warnings.warn(f'Ignoring placement region {cid}: xmin/ymin must not exceed xmax/ymax', stacklevel=2)
                    continue
                if cid in point_ids:
                    raise ValueError(f'Placement class {cid} defines both place_positions and place_regions')
                result['place_regions.' + str(cid)] = [xmin, ymin, xmax, ymax, z]
    return result


def resolve(planner, modes_argument='', positions_argument='', regions_argument=''):
    result = dict(planner)
    has_positions = 'place_positions' in planner
    has_regions = 'place_regions' in planner
    positions = result.pop('place_positions', None)
    regions = result.pop('place_regions', None)
    present = (has_positions or has_regions or planner.get('use_position_rules', False) or
               any(k.startswith(('place_positions.', 'place_regions.')) for k in result))
    if not has_positions and present:
        collected = {k.split('.', 1)[1]: v for k, v in result.items() if k.startswith('place_positions.')}
        if collected:
            positions = collected
            has_positions = True
    if not has_regions and present:
        collected = {k.split('.', 1)[1]: v for k, v in result.items() if k.startswith('place_regions.')}
        if collected:
            regions = collected
            has_regions = True
    if positions_argument or regions_argument:
        # 更高层提供任一显式规则时，整体替换低层两类规则；
        # 同层同时给出点与区域留给 parameters() 报配置错误。
        positions = yaml.safe_load(positions_argument) if positions_argument else {}
        regions = yaml.safe_load(regions_argument) if regions_argument else {}
        has_positions = bool(positions_argument)
        has_regions = bool(regions_argument)
        present = True
    if present:
        result = {k: v for k, v in result.items()
                  if not k.startswith(('place_positions.', 'place_regions.'))}
        config = {}
        if has_positions:
            config['place_positions'] = positions
        if has_regions:
            config['place_regions'] = regions
        result.update(parameters(config))
    if modes_argument:
        result['place_position_modes'] = yaml.safe_load(modes_argument)
    result.update(parameters({k: result[k] for k in ('place_position_modes',) if k in result}))
    if not present and any(m == 'relative' for m in result.get('place_position_modes', [])):
        raise ValueError('Relative modes require explicit class placement rules')
    return result


def describe(planner, frame, catalog=None):
    if not planner.get('use_position_rules'):
        return [f'Placement: configured class regions/slots in {frame}']
    modes = planner.get('place_position_modes', ['absolute'] * 3)
    lines = [f'Placement frame: {frame}; X/Y/Z modes: {modes}']
    for key, values in planner.items():
        if not key.startswith('place_positions.'):
            continue
        cid = int(key.split('.')[1])
        if catalog is not None and cid not in catalog:
            lines.append(f'Ignored placement class {cid}: undefined in object_classes')
            continue
        axes = [f'initial_{axis} {value:+g}' if mode == 'relative' else f'{value:g}'
                for axis, value, mode in zip('xyz', values, modes)]
        lines.append(f'class {cid} object center = [{", ".join(axes)}]')
    for key, values in planner.items():
        if not key.startswith('place_regions.'):
            continue
        cid = int(key.split('.')[1])
        if catalog is not None and cid not in catalog:
            lines.append(f'Ignored placement class {cid}: undefined in object_classes')
            continue
        xmin, ymin, xmax, ymax, z = values
        lines.append(f'class {cid} region = x[{xmin:g}, {xmax:g}] y[{ymin:g}, {ymax:g}] z={z:g} '
                     '- absolute, grid step from planner YAML')
    lines.append('Classes without an applicable placement rule are not picked; no legacy fallback.')
    if any(key.startswith('place_regions.') for key in planner):
        lines.append('Region rules are always absolute; X/Y/Z modes only apply to class points.')
    if 'relative' in modes:
        lines.append('Relative targets need live observations; Planner prints frozen absolute targets before dispatch.')
    return lines
