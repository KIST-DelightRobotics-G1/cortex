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
                                     llm_backend=llm_backend)
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
    assert arguments['detector_profile'].endswith('fridge_detector_v3.yaml')


def test_demo_launch_builds_without_model(tmp_path):
    demo = runpy.run_path(str(LAUNCH/'llm_demo.launch.py'))
    description = demo['generate_launch_description']()
    assert description.entities
