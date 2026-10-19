"""Deployment precedence and safe missing-model behavior, without ROS."""
from pathlib import Path
import pytest
import yaml
from cortex_bringup.runtime_config import detector_overrides

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'src/cortex_bringup/config/cortex_params.yaml'


def write_config(path, **values):
    path.write_text(yaml.safe_dump({'detector_node': {'ros__parameters': values}}))
    return path


def test_standard_defaults_match_reviewed_v3_policy(tmp_path):
    base = yaml.safe_load(BASE.read_text())
    v3 = yaml.safe_load((BASE.parent/'fridge_detector_v3.yaml').read_text())
    for node, spec in v3.items():
        for key, value in spec['ros__parameters'].items():
            assert base[node]['ros__parameters'][key] == value, (node, key)
    with pytest.raises(ValueError, match='YOLO model not found'):
        detector_overrides(BASE, model=str(tmp_path/'absent.pt'))


def test_yaml_model_and_device_are_used_without_cli_override(tmp_path):
    weight = tmp_path/'model with spaces.pt'
    weight.touch()
    cfg = write_config(tmp_path/'site.yaml', backend='yolo', model=str(weight), device='cpu')
    assert detector_overrides(cfg) == {'model': str(weight)}


def test_custom_yaml_overlay_and_cli_have_correct_precedence(tmp_path):
    first, second, third = [tmp_path/n for n in ('base.pt', 'overlay.pt', 'cli.pt')]
    for p in (first, second, third):
        p.touch()
    cfg = write_config(tmp_path/'site.yaml', backend='yolo', model=str(first), device='cpu')
    overlay = write_config(tmp_path/'arbitrary-name.yaml', model=str(second))
    assert detector_overrides(cfg, overlay)['model'] == str(second)
    assert detector_overrides(cfg, overlay, str(third), '0') == {
        'model': str(third), 'device': '0'}


def test_missing_yaml_is_not_silently_ignored(tmp_path):
    with pytest.raises(ValueError, match='parameter file'):
        detector_overrides(BASE, tmp_path/'absent.yaml')


def test_explicit_stub_needs_no_weight(tmp_path):
    cfg = write_config(tmp_path/'demo.yaml', backend='always', model='/missing.pt')
    assert detector_overrides(cfg) == {}



def test_incomplete_site_yaml_cannot_fall_back_to_implicit_stub(tmp_path):
    cfg = write_config(tmp_path/'incomplete.yaml', device='cpu')
    with pytest.raises(ValueError, match='explicitly select'):
        detector_overrides(cfg)


def test_node_logging_reads_level_and_events(tmp_path):
    from cortex_bringup.runtime_config import node_logging
    spec = tmp_path/'logging.yaml'
    spec.write_text('level: warn\nrecord: all\n'
                    'nodes:\n  stt_node: {level: debug, events: [transcript, session]}\n'
                    '  llm_node: {events: request, record: [request, timing]}\n')
    assert node_logging(str(spec), 'stt_node') == ('debug', 'transcript,session', 'all')
    assert node_logging(str(spec), 'llm_node') == ('warn', 'request', 'request,timing')
    assert node_logging(str(spec), 'tts_node') == ('warn', '', 'all')
    assert node_logging('', 'tts_node') == ('info', '', '')
    spec.write_text('level: loud\n')
    with pytest.raises(ValueError):
        node_logging(str(spec), 'stt_node')
    with pytest.raises(ValueError):
        node_logging(str(tmp_path/'missing.yaml'), 'stt_node')


def test_shipped_logging_yaml_defaults():
    from cortex_bringup.runtime_config import node_logging
    shipped = Path(__file__).resolve().parents[1]/'src/cortex_bringup/config/logging.yaml'
    assert node_logging(str(shipped), 'orchestrator_node') == ('info', 'session,plan,precheck,subtask', 'all')
    assert node_logging(str(shipped), 'detector_node') == ('info', 'config,check', 'all')
    assert node_logging(str(shipped), 'stt_node') == ('info', 'transcript', 'all')
    assert node_logging(str(shipped), 'llm_node') == ('info', '', 'all')


def test_session_log_section(tmp_path):
    from cortex_bringup.runtime_config import session_log
    shipped = ROOT/'src/cortex_bringup/config/logging.yaml'
    assert session_log(str(shipped)) == {'dir': '~/cortex_logs', 'keep_days': 14, 'pre_roll_s': 5.0,
                                         'post_roll_s': 2.0, 'end_timeout_s': 120.0}
    spec = tmp_path/'logging.yaml'
    spec.write_text('session_log: {enabled: false, dir: /x}\n')
    assert session_log(str(spec)) is None and session_log('') is None
    spec.write_text('session_log: {enabled: true, keep: 3}\n')
    with pytest.raises(ValueError):
        session_log(str(spec))


def test_default_logging_file_prefers_the_mounted_host_copy(monkeypatch, tmp_path):
    from cortex_bringup.runtime_config import default_logging_file
    monkeypatch.delenv('CORTEX_LOGGING_FILE', raising=False)
    assert default_logging_file(tmp_path) == str(tmp_path/'config'/'logging.yaml')
    monkeypatch.setenv('CORTEX_LOGGING_FILE', '/workspace/cortex/host_config/logging.yaml')
    assert default_logging_file(tmp_path) == '/workspace/cortex/host_config/logging.yaml'


def wrist_site(tmp_path):
    spec = yaml.safe_load(BASE.read_text())
    spec['orchestrator_node']['ros__parameters']['holding_precheck_enabled'] = True
    wrist = spec['wrist_detector_node']['ros__parameters']
    weight = tmp_path/'wrist.pt'
    weight.touch()
    wrist.update(camera_topic='/test/right_wrist/image', model=str(weight),
                 check_roi=[.4, .4, .9, .9], roi_min_overlap=.5)
    return spec


def test_wrist_opt_in_and_layered_configuration(tmp_path):
    from cortex_bringup.runtime_config import wrist_detector_parameters
    assert wrist_detector_parameters(BASE) is None
    spec = wrist_site(tmp_path)
    site = tmp_path/'site.yaml'
    site.write_text(yaml.safe_dump(spec))
    params = wrist_detector_parameters(site, device='cpu')
    assert params['device'] == 'cpu' and params['model'] == str(tmp_path/'wrist.pt')
    overlay = tmp_path/'disable.yaml'
    overlay.write_text('orchestrator_node:\n  ros__parameters:\n    holding_precheck_enabled: false\n')
    assert wrist_detector_parameters(site, overlay) is None


@pytest.mark.parametrize('node,key,value', [
    ('orchestrator_node', 'holding_precheck_enabled', 'true'),
    ('orchestrator_node', 'holding_detector_service', '/wrong'),
    ('orchestrator_node', 'precheck_timeout_s', 0.),
    ('orchestrator_node', 'holding_max_age_s', float('nan')),
    ('orchestrator_node', 'planner_mode', 'scenario'),
    ('wrist_detector_node', 'camera_topic', ''),
    ('wrist_detector_node', 'camera_topic', '/kist/camera/head/color/h264'),
    ('wrist_detector_node', 'service', '/cortex/detector/check'),
    ('wrist_detector_node', 'detections_topic', '/cortex/detections'),
    ('wrist_detector_node', 'backend', 'always'),
    ('wrist_detector_node', 'allow_unstamped', True),
    ('wrist_detector_node', 'require_latest_hit', False),
    ('wrist_detector_node', 'check_roi', [.8, .2, .3, .9]),
    ('wrist_detector_node', 'roi_min_overlap', -.1),
    ('wrist_detector_node', 'target_keys', ['cucumber']),
    ('wrist_detector_node', 'model', '/absent.pt'),
])
def test_incomplete_or_misrouted_wrist_config_fails_before_starting_nodes(tmp_path, node, key, value):
    from cortex_bringup.runtime_config import wrist_detector_parameters
    spec = wrist_site(tmp_path)
    spec[node]['ros__parameters'][key] = value
    site = tmp_path/'site.yaml'
    site.write_text(yaml.safe_dump(spec))
    with pytest.raises(ValueError):
        wrist_detector_parameters(site)


def test_wrist_full_view_candidate_keeps_separate_camera_thresholds(tmp_path):
    from cortex_bringup.runtime_config import node_parameters, wrist_detector_parameters
    spec = wrist_site(tmp_path)
    wrist = spec['wrist_detector_node']['ros__parameters']
    wrist.update(check_roi=[0., 0., 1., 1.], roi_min_overlap=0.)
    site = tmp_path/'site.yaml'
    site.write_text(yaml.safe_dump(spec))
    params = wrist_detector_parameters(site)
    assert params['default_min_confidence'] == .7
    assert params['infer_conf'] == .25 and params['roi_min_overlap'] == 0.
    assert node_parameters(site, '', 'detector_node')['default_min_confidence'] == .25
