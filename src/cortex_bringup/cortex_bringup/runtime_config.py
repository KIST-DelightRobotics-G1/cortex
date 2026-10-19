"""Resolve deployment YAML and optional launch overrides without ROS imports."""
import math
import os
from pathlib import Path

import yaml


def node_parameters(params_file, profile, name):
    """Merge one node's ROS parameters from the site YAML and optional overlay."""
    parameters = {}
    for source in (params_file, profile):
        if not source:
            continue
        path = Path(source).expanduser()
        if not path.is_file():
            raise ValueError(f'parameter file does not exist: {path}')
        spec = yaml.safe_load(path.read_text()) or {}
        parameters.update(spec.get(name, {}).get('ros__parameters', {}))
    return parameters


def detector_overrides(params_file, profile='', model='', device='', *, name='detector_node'):
    """Validate the effective detector model; CLI values override YAML values."""
    detector = node_parameters(params_file, profile, name)
    if model:
        detector['model'] = model
    if device:
        detector['device'] = device
    result = {}
    if detector.get('backend') not in ('yolo', 'always'):
        raise ValueError(f'YAML must explicitly select {name}.backend: yolo or always')
    if detector['backend'] == 'yolo':
        weight = Path(str(detector.get('model', ''))).expanduser()
        if not weight.is_file():
            hint = ' or pass model:=/absolute/path/model.pt' if name == 'detector_node' else ''
            raise ValueError(
                f'YOLO model not found: {weight}. Set {name}.model in the YAML{hint}; '
                'Docker mounts weights at /models/cortex.')
        result['model'] = str(weight.resolve())
    if device:
        result['device'] = str(device)
    return result


def wrist_detector_parameters(params_file, profile='', device=''):
    """Opt-in second camera. Refuse a partial/misrouted holding configuration."""
    orchestrator = node_parameters(params_file, profile, 'orchestrator_node')
    enabled = orchestrator.get('holding_precheck_enabled', False)
    if not isinstance(enabled, bool):
        raise ValueError('holding_precheck_enabled must be a boolean')
    if not enabled:
        return None
    if orchestrator.get('planner_mode', 'llm') != 'llm':
        raise ValueError('holding precheck is supported in planner_mode: llm')
    for key in ('precheck_timeout_s', 'holding_max_age_s'):
        value = orchestrator.get(key, 0.)
        if not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'holding precheck requires positive finite {key}')
    head = node_parameters(params_file, profile, 'detector_node')
    wrist = node_parameters(params_file, profile, 'wrist_detector_node')
    for key in ('camera_topic', 'service', 'detections_topic'):
        value = wrist.get(key, '')
        if not isinstance(value, str) or not value.startswith('/') or value == head.get(key):
            raise ValueError(f'wrist_detector_node.{key} must be a separate absolute ROS name')
    if wrist['service'] != orchestrator.get('holding_detector_service'):
        raise ValueError('holding_detector_service must match wrist_detector_node.service')
    if (wrist.get('backend') != 'yolo' or wrist.get('allow_unstamped') is not False or
            wrist.get('require_latest_hit') is not True or
            wrist.get('target_keys') != ['held_cucumber'] or wrist.get('target_classes') != ['cucumber']):
        raise ValueError('wrist check requires stamped YOLO cucumber evidence and latest-hit policy')
    roi, overlap = wrist.get('check_roi', []), wrist.get('roi_min_overlap', 0.)
    if (not isinstance(roi, list) or len(roi) != 4 or
            not all(isinstance(v, (float, int)) and math.isfinite(v) and 0 <= v <= 1 for v in roi) or
            roi[0] >= roi[2] or roi[1] >= roi[3] or
            not isinstance(overlap, (float, int)) or not math.isfinite(overlap) or not 0 <= overlap <= 1):
        raise ValueError('invalid wrist check_roi or roi_min_overlap')
    wrist.update(detector_overrides(params_file, profile, device=device, name='wrist_detector_node'))
    return wrist


LOG_LEVELS = ('debug', 'info', 'warn', 'error', 'fatal')


def default_logging_file(share):
    """$CORTEX_LOGGING_FILE when set (docker/run.sh points it at the host's config, so an edit
    applies on the next launch without an image rebuild), else the installed logging.yaml."""
    return os.environ.get('CORTEX_LOGGING_FILE') or str(Path(share)/'config'/'logging.yaml')


def _logging_spec(logging_file):
    if not logging_file:
        return {}
    path = Path(logging_file).expanduser()
    if not path.is_file():
        raise ValueError(f'logging file does not exist: {path}')
    return yaml.safe_load(path.read_text()) or {}


def node_logging(logging_file, name):
    """logging.yaml -> (ROS log level, console topics, recorded topics) for one node.
    Topics come back comma-joined, the form of the log_events / record_events parameters."""
    spec = _logging_spec(logging_file)
    node = (spec.get('nodes') or {}).get(name) or {}
    level = str(node.get('level', spec.get('level', 'info'))).lower()
    if level not in LOG_LEVELS:
        raise ValueError(f'{name}: log level must be one of {", ".join(LOG_LEVELS)}, got {level!r}')
    def joined(value):
        return ','.join(str(t) for t in ([value] if isinstance(value, str) else value or []))
    return level, joined(node.get('events')), joined(node.get('record', spec.get('record')))


SESSION_LOG_KEYS = {'dir': str, 'keep_days': int, 'pre_roll_s': float, 'post_roll_s': float,
                    'end_timeout_s': float}


def session_log(logging_file):
    """logging.yaml session_log -> session_recorder_node parameters, or None when it is off."""
    section = _logging_spec(logging_file).get('session_log') or {}
    if not section.get('enabled', False):
        return None
    unknown = set(section) - set(SESSION_LOG_KEYS) - {'enabled'}
    if unknown:
        raise ValueError(f'session_log: unknown key(s) {sorted(unknown)}; '
                         f'known: enabled, {", ".join(SESSION_LOG_KEYS)}')
    return {key: SESSION_LOG_KEYS[key](value) for key, value in section.items() if key != 'enabled'}
