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
                                     llm_backend=llm_backend, record_session='true')
    return ctx


def test_standard_launch_builds_full_graph_and_detector_only(tmp_path):
    main = runpy.run_path(str(LAUNCH/'cortex.launch.py'))
    full = main['_nodes'](context(tmp_path))
    assert len(full) == 7 and all(isinstance(n, Node) for n in full)
    assert len(main['_nodes'](context(tmp_path, 'true'))) == 1


def test_llm_backend_override_is_accepted_and_validated(tmp_path):
    """Any of the three backends builds the full graph; anything else fails fast
    at launch time rather than silently leaving the YAML value in place."""
    main = runpy.run_path(str(LAUNCH/'cortex.launch.py'))
    for backend in ('', 'dummy', 'gemini', 'openai'):
        nodes = main['_nodes'](context(tmp_path, llm_backend=backend))
        assert len(nodes) == 7, backend
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
    assert arguments['record_session'] == 'true'
    assert arguments['detector_profile'].endswith('fridge_detector_v3.yaml')


def test_demo_launch_builds_without_model(tmp_path):
    demo = runpy.run_path(str(LAUNCH/'llm_demo.launch.py'))
    description = demo['generate_launch_description']()
    assert description.entities


def test_session_recording_wiring_and_opt_out(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from launch.actions import ExecuteProcess

    main = runpy.run_path(str(LAUNCH/'cortex.launch.py'))
    scope = main['_launch_setup'].__globals__
    monkeypatch.setitem(scope, 'launch_config', SimpleNamespace(log_dir=str(tmp_path/'session')))
    monkeypatch.setitem(scope, 'get_package_share_directory', lambda _: str(ROOT))
    node_options, process_options = [], []

    def node(**kwargs):
        node_options.append(kwargs)
        return Node(**kwargs)

    def process(**kwargs):
        process_options.append(kwargs)
        return ExecuteProcess(**kwargs)

    monkeypatch.setitem(scope, 'Node', node)
    monkeypatch.setitem(scope, 'ExecuteProcess', process)
    ctx = context(tmp_path)
    main['_launch_setup'](ctx)
    assert len(node_options) == 7 and len(process_options) == 1
    assert all(n['output'] == 'both' for n in node_options)
    assert all(n['additional_env']['ROS_LOG_DIR'] == str(tmp_path/'session') for n in node_options)
    by_name = {n['name']: n for n in node_options}
    assert by_name['detector_node']['parameters'][-1] == {'diagnostics_enabled': True}
    assert by_name['orchestrator_node']['parameters'][-1] == {'detector_diagnostics_enabled': True}
    cmd = process_options[0]['cmd']
    assert str(ROOT/'scripts/collect_yolo_diagnostics.py') in cmd
    assert cmd[cmd.index('--duration')+1] == '0'
    assert str(tmp_path/'session/diagnostics') in cmd

    node_options.clear()
    process_options.clear()
    ctx.launch_configurations['record_session'] = 'false'
    main['_launch_setup'](ctx)
    assert not process_options
    assert len(node_options) == 7 and all(n['output'] == 'both' for n in node_options)
    assert not any(isinstance(layer, dict) and
                   ('diagnostics_enabled' in layer or 'detector_diagnostics_enabled' in layer)
                   for n in node_options for layer in n['parameters'])

    ctx.launch_configurations.update(record_session='true', detector_only='true')
    main['_launch_setup'](ctx)
    cmd = process_options[0]['cmd']
    assert cmd[cmd.index('--orchestrator-node')+1] == ''
    ctx.launch_configurations['record_session'] = 'invalid'
    with pytest.raises(ValueError, match='record_session'):
        main['_launch_setup'](ctx)


def test_installed_collector_matches_source():
    from ament_index_python.packages import get_package_share_directory
    installed = Path(get_package_share_directory('cortex_bringup'))/'scripts/collect_yolo_diagnostics.py'
    assert installed.read_bytes() == (ROOT/'scripts/collect_yolo_diagnostics.py').read_bytes()


def test_standard_launch_saves_stdout_and_stderr_without_robot(tmp_path):
    """Exercise Humble Node/LaunchService I/O with a harmless executable substitute."""
    import os
    import subprocess
    import sys

    weight = tmp_path/'weight.pt'
    weight.touch()
    script = r"""
import os, runpy, sys
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node
main = runpy.run_path(sys.argv[1])
def harmless_node(**options):
    # Keep the real launch wiring; only replace the robot executable.
    options.pop('package')
    options['executable'] = sys.executable
    options['arguments'] = ['-u', '-c',
        'import os,sys; print("SESSION_STDOUT"); print("SESSION_STDERR", file=sys.stderr); '
        'print("CHILD_LOG_DIR="+os.environ["ROS_LOG_DIR"])']
    return Node(**options)
main['_nodes'].__globals__['Node'] = harmless_node
settings = dict(params_file=sys.argv[2], detector_profile='', model=sys.argv[3],
                device='cpu', detector_only='true', llm_backend='', record_session='false')
service = LaunchService()
service.include_launch_description(LaunchDescription(
    [SetLaunchConfiguration(k, v) for k, v in settings.items()] +
    [OpaqueFunction(function=main['_launch_setup'])]))
sys.exit(service.run())
"""
    result = subprocess.run(
        [sys.executable, '-c', script, str(LAUNCH/'cortex.launch.py'), str(BASE), str(weight)],
        env={**os.environ, 'ROS_LOG_DIR': str(tmp_path/'logs'), 'ROS_LOCALHOST_ONLY': '1'},
        capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout+result.stderr
    logs = list((tmp_path/'logs').glob('*/launch.log'))
    assert len(logs) == 1
    saved = logs[0].read_text()
    assert 'SESSION_STDOUT' in saved and 'SESSION_STDERR' in saved
    assert f'CHILD_LOG_DIR={logs[0].parent}' in saved
