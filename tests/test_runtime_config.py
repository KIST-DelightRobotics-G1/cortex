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
    assert node_logging(str(shipped), 'detector_node') == ('info', 'config,status,check', 'all')
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
