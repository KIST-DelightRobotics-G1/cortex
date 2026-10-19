"""ROS launch wiring; runs in Humble CI after colcon build."""
import runpy
from pathlib import Path

import pytest
pytest.importorskip('launch.launch_context', reason='requires ROS 2 launch')
from launch import LaunchContext
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT/'src/cortex_bringup/launch'
BASE = LAUNCH.parent/'config/cortex_params.yaml'


def context(tmp_path, only='false', llm_backend=''):
    weight = tmp_path/'weight.pt'
    weight.touch()
    ctx = LaunchContext()
    ctx.launch_configurations.update(params_file=str(BASE), detector_profile='',
                                     model=str(weight), device='cpu', detector_only=only,
                                     llm_backend=llm_backend,
                                     logging_file=str(LAUNCH.parent/'config/logging.yaml'))
    return ctx


def test_standard_launch_builds_full_graph_and_detector_only(tmp_path):
    main = runpy.run_path(str(LAUNCH/'cortex.launch.py'))
    full = main['_nodes'](context(tmp_path))
    assert len(full) == 8 and all(isinstance(n, Node) for n in full)
    assert full[-1].node_executable == 'session_recorder_node'   # logging.yaml session_log
    assert len(main['_nodes'](context(tmp_path, 'true'))) == 1


def test_logging_yaml_sets_level_and_events(tmp_path):
    main = runpy.run_path(str(LAUNCH/'cortex.launch.py'))
    spec = tmp_path/'logging.yaml'
    spec.write_text('level: warn\nnodes:\n  stt_node: {level: debug, events: [transcript]}\n')
    ctx = context(tmp_path)
    ctx.launch_configurations['logging_file'] = str(spec)
    for node in main['_nodes'](ctx):
        stt = node.node_executable == 'stt_node'
        assert node._Node__arguments == ['--ros-args', '--log-level', 'debug' if stt else 'warn']
        events, record = node._Node__parameters[-1].values()  # the last layer
        assert events.value == ('transcript' if stt else '') and record.value == ''


def test_llm_backend_override_is_accepted_and_validated(tmp_path):
    """Any of the three backends builds the full graph; anything else fails fast
    at launch time rather than silently leaving the YAML value in place."""
    main = runpy.run_path(str(LAUNCH/'cortex.launch.py'))
    for backend in ('', 'dummy', 'gemini', 'openai'):
        nodes = main['_nodes'](context(tmp_path, llm_backend=backend))
        assert len(nodes) == 8, backend
    with pytest.raises(ValueError):
        main['_nodes'](context(tmp_path, llm_backend='bogus'))


def test_legacy_launch_delegates_to_standard_launch(tmp_path):
    compat = runpy.run_path(str(LAUNCH/'cortex_yolo.launch.py'))
    ctx = context(tmp_path)
    ctx.launch_configurations['profile'] = str(BASE.parent/'fridge_detector_v3.yaml')
    include = compat['_include'](ctx)[0]
    arguments = dict(include.launch_arguments)
    assert arguments['params_file'] == str(BASE)
    assert arguments['device'] == 'cpu'
    assert arguments['detector_profile'].endswith('fridge_detector_v3.yaml')


def test_demo_launch_builds_without_model(tmp_path):
    demo = runpy.run_path(str(LAUNCH/'llm_demo.launch.py'))
    description = demo['generate_launch_description']()
    assert description.entities


def test_wrist_launch_adds_a_separate_detector_and_keeps_ego(tmp_path):
    import yaml
    main = runpy.run_path(str(LAUNCH/'cortex.launch.py'))
    ctx = context(tmp_path)
    wrist_weight = tmp_path/'wrist.pt'
    wrist_weight.touch()
    overlay = tmp_path/'wrist.yaml'
    overlay.write_text(yaml.safe_dump({
        'orchestrator_node': {'ros__parameters': {'holding_precheck_enabled': True}},
        'wrist_detector_node': {'ros__parameters': {'camera_topic': '/test/wrist/image',
            'model': str(wrist_weight), 'check_roi': [0., 0., 1., 1.], 'roi_min_overlap': 0.}}}))
    ctx.launch_configurations['detector_profile'] = str(overlay)
    nodes = main['_nodes'](ctx)
    detectors = [n for n in nodes if n.node_executable == 'detector_node']
    assert len(nodes) == 9 and len(detectors) == 2
    values = [v.value for v in detectors[1]._Node__parameters[0].values() if hasattr(v, 'value')]
    assert '/test/wrist/image' in values and str(wrist_weight) in values
    assert '/cortex/wrist_detector/check' in values and 'cpu' in values
    ctx.launch_configurations['detector_only'] = 'true'
    assert len(main['_nodes'](ctx)) == 2
