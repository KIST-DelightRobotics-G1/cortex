"""Shared perception policy -> real pure executor (no ROS or motion modules)."""
from pathlib import Path

import pytest
import yaml

from cortex_cognition import executor as ex, planner
from cortex_perception.detection import DetectionRecord, PresenceWindow, target_mapping

ROOT = Path(__file__).resolve().parents[1]
CFG = planner.load_config(str(ROOT/'src/cortex_cognition/config/actions.yaml'))
PROFILE = yaml.safe_load((ROOT/'src/cortex_bringup/config/fridge_detector.yaml').read_text())


def run(window, target, action, now):
    commands = []
    ports = ex.Ports(now=lambda: now,
                     send_cmd=lambda module, step, pid: commands.append((module, step.action)),
                     send_cancel=lambda *a: None,
                     check_target=lambda t: (window.check(t, now).found, window.check(t, now).detail),
                     say=lambda *a: None, stop_speech=lambda: None, trace=lambda *a: None,
                     status=lambda *a: None)
    params = ex.Params(detector_fail_open=PROFILE['orchestrator_node']['ros__parameters']['detector_fail_open'])
    executor = ex.Executor(CFG, ports, params)
    executor.heard('test', 'test')
    executor.on_step('test', 0, 0, action, [target], '', '', '')
    return commands


@pytest.mark.parametrize('target,label,action', [('fridge_door', 'refrigerator', 'open'),
                                               ('cucumber', 'cucumber', 'pick')])
def test_real_profile_gates_dispatch_on_distinct_frame_evidence(target, label, action):
    p = PROFILE['detector_node']['ros__parameters']
    w = PresenceWindow(target_mapping(p['target_keys'], p['target_classes']), ['refrigerator', 'cucumber'],
                       p['window_s'], p['default_min_confidence'], p['min_frames'])
    assert run(w, target, action, 0) == []
    d = DetectionRecord(label, .9, .5, .5, .3, .3)
    for i in range(3):
        w.add([d], i*.125, i+1, i*.125)
    assert run(w, target, action, .25) == [('vla', action)]
    assert run(w, target, action, 1.0) == []
    w.invalidate('model_not_loaded')
    assert run(w, target, action, 1.0) == []


def test_cancel_constructor_matches_the_headerless_wire_schema():
    # Compile the actual method and run against a strict generated-message-shaped object.
    # This catches the former msg.header access without pretending to run DDS.
    import ast
    from types import SimpleNamespace
    tree = ast.parse((ROOT/'src/cortex_cognition/cortex_cognition/orchestrator_node.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'OrchestratorNode')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_send_cancel')
    class Cmd:
        __slots__ = ('plan_id', 'index', 'cancel')
        def __init__(self, plan_id, index, cancel):
            self.plan_id, self.index, self.cancel = plan_id, index, cancel
    scope = {'SubtaskCmd': Cmd, 'str': str, 'int': int}
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<actual cancel method>', 'exec'), scope)
    sent = []
    scope['_send_cancel'](SimpleNamespace(cmd_pub={'nav': SimpleNamespace(publish=sent.append)}), 'nav', 'p1', 2)
    assert (sent[0].plan_id, sent[0].index, sent[0].cancel) == ('p1', 2, True)
