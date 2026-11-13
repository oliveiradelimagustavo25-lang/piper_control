"""逐类别放置规则与 YAML 优先级；显式缺失规则不会回退旧坐标。"""
import math
import warnings

import yaml

KEYS = ('place_position_modes', 'place_positions')


def parameters(config):
    result = {}
    if 'place_position_modes' in config:
        modes = config['place_position_modes']
        if not isinstance(modes, list) or len(modes) != 3 or any(m not in ('relative', 'absolute') for m in modes):
            raise ValueError('place_position_modes requires three X/Y/Z entries: relative or absolute')
        result['place_position_modes'] = modes
    if 'place_positions' not in config:
        return result
    result['use_position_rules'] = True
    positions = config['place_positions']
    if not isinstance(positions, dict):
        warnings.warn('Ignoring invalid place_positions; no legacy fallback', stacklevel=2)
        return result
    for key, values in positions.items():
        cid = str(key)
        if (isinstance(key, bool) or not cid.isdigit() or not 1 <= int(cid) <= 255 or
                sum(str(k).isdigit() and int(str(k)) == int(cid) for k in positions) != 1):
            warnings.warn(f'Ignoring invalid/duplicate placement class {key!r}', stacklevel=2)
            continue
        if (not isinstance(values, list) or len(values) != 3 or
                any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in values)):
            warnings.warn(f'Ignoring placement class {cid}: expected three finite coordinates', stacklevel=2)
            continue
        result['place_positions.' + str(int(cid))] = [float(v) for v in values]
    return result


def resolve(planner, modes_argument='', positions_argument=''):
    result = dict(planner)
    positions = result.pop('place_positions', None)
    present = 'place_positions' in planner or planner.get('use_position_rules', False) or any(k.startswith('place_positions.') for k in result)
    if 'place_positions' not in planner and present:
        positions = {k.split('.', 1)[1]: v for k, v in result.items() if k.startswith('place_positions.')}
    if positions_argument:
        positions = yaml.safe_load(positions_argument)
        present = True
    if present:
        result = {k: v for k, v in result.items() if not k.startswith('place_positions.')}
        result.update(parameters({'place_positions': positions}))
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
    lines.append('Classes without an applicable placement rule are not picked; no legacy fallback.')
    if 'relative' in modes:
        lines.append('Relative targets need live observations; Planner prints frozen absolute targets before dispatch.')
    return lines
