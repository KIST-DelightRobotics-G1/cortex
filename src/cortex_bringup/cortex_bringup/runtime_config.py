"""Resolve deployment YAML and optional launch overrides without ROS imports."""
from pathlib import Path

import yaml


def detector_overrides(params_file, profile='', model='', device=''):
    """Validate the effective detector model; CLI values override YAML values."""
    detector = {}
    for source in (params_file, profile):
        if not source:
            continue
        path = Path(source).expanduser()
        if not path.is_file():
            raise ValueError(f'parameter file does not exist: {path}')
        spec = yaml.safe_load(path.read_text()) or {}
        detector.update(spec.get('detector_node', {}).get('ros__parameters', {}))
    if model:
        detector['model'] = model
    if device:
        detector['device'] = device
    result = {}
    if detector.get('backend') not in ('yolo', 'always'):
        raise ValueError('YAML must explicitly select detector_node.backend: yolo or always')
    if detector['backend'] == 'yolo':
        weight = Path(str(detector.get('model', ''))).expanduser()
        if not weight.is_file():
            raise ValueError(
                f'YOLO model not found: {weight}. Set detector_node.model in the YAML '
                'or pass model:=/absolute/path/model.pt; Docker mounts weights at /models/cortex.')
        result['model'] = str(weight.resolve())
    if device:
        result['device'] = str(device)
    return result


LOG_LEVELS = ('debug', 'info', 'warn', 'error', 'fatal')


def node_logging(logging_file, name):
    """logging.yaml -> (ROS log level, console topics, recorded topics) for one node.
    Topics come back comma-joined, the form of the log_events / record_events parameters."""
    spec = {}
    if logging_file:
        path = Path(logging_file).expanduser()
        if not path.is_file():
            raise ValueError(f'logging file does not exist: {path}')
        spec = yaml.safe_load(path.read_text()) or {}
    node = (spec.get('nodes') or {}).get(name) or {}
    level = str(node.get('level', spec.get('level', 'info'))).lower()
    if level not in LOG_LEVELS:
        raise ValueError(f'{name}: log level must be one of {", ".join(LOG_LEVELS)}, got {level!r}')
    def joined(value):
        return ','.join(str(t) for t in ([value] if isinstance(value, str) else value or []))
    return level, joined(node.get('events')), joined(node.get('record', spec.get('record')))
