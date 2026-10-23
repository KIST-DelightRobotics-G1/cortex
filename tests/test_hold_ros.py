"""Orchestrator side of holding: GripperPresence in, holding window and its stats logged."""
import json
import os
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 and colcon-built interfaces')
from cortex_msgs.msg import GripperPresence, ObjectPresence, SubtaskState
from cortex_cognition import hold, planner
from cortex_cognition.orchestrator_node import OrchestratorNode
from cortex_common.events import EventLog

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = planner.load_config(os.path.join(ROOT, 'src/cortex_cognition/config/actions.yaml'))
METHODS = ('_set_holding', '_end_hold', '_sample_hold', '_on_gripper', '_update_robot_state',
           '_send_cmd')


def make(monitor=True):
    logs, clock, sent = [], [0.], []
    n = SimpleNamespace(
        held_monitor=monitor, gripper=hold.GripperView(1.0), _hold_stats=None,
        holding='none', at='home', _sent={}, _PICKS=hold.PICKS, _PUTS=hold.PUTS,
        cfg=dict(CFG, vla_prompts={}), _now=lambda: clock[0],
        _exec=SimpleNamespace(plan_id='p1', phase='RUNNING'),
        cmd_pub={'vla': SimpleNamespace(publish=sent.append)},
        get_logger=lambda: SimpleNamespace(info=logs.append, warning=lambda *_: None))
    n.events = EventLog(n, known=('state', 'hold', 'subtask'), topics='state,hold')
    for name in METHODS:
        setattr(n, name, getattr(OrchestratorNode, name).__get__(n))
    return n, logs, clock


def gripper_msg(state, camera_ok=True):
    return GripperPresence(camera_ok=camera_ok,
                           objects=[ObjectPresence(key='cucumber', state=state, confidence=.9)])


def done(n, action, args, index):
    n._sent['vla'] = ('p1', index, action, args)
    n._update_robot_state('vla', SubtaskState(plan_id='p1', index=index, status=SubtaskState.DONE))


def events(logs, name):
    return [json.loads(line.split(' ', 1)[1]) for line in logs if line.startswith(name + ' ')]


def test_holding_window_logs_state_changes_and_stats_until_release():
    n, logs, clock = make()
    done(n, 'take_out', ['cucumber', 'fridge'], 3)
    assert n.holding == 'cucumber'
    assert events(logs, 'HOLD_MONITOR')[-1] == {'stage': 'start', 'object': 'cucumber'}
    n._on_gripper(gripper_msg(ObjectPresence.PRESENT))
    clock[0] = 6.
    n._on_gripper(gripper_msg(ObjectPresence.ABSENT))       # e.g. occluded by the hand
    clock[0] = 7.
    n._on_gripper(gripper_msg(ObjectPresence.PRESENT))
    clock[0] = 10.
    step = SimpleNamespace(action='place', args=['cucumber', 'table'], index=7, instruction='Place.')
    n._send_cmd('vla', step, 'p1')                           # letting it go: stats end here
    states = [e['state'] for e in events(logs, 'HOLD_STATE')]
    assert states == ['UNKNOWN', 'PRESENT', 'ABSENT', 'PRESENT']   # no message yet at the pick
    [stats] = events(logs, 'HOLD_STATS')
    assert stats['reason'] == 'release' and stats['object'] == 'cucumber'
    assert stats['duration_s'] == 10.0 and stats['absent'] == .1 and stats['changes'] == 3
    done(n, 'place', ['cucumber', 'table'], 7)
    assert n.holding == 'none' and len(events(logs, 'HOLD_STATS')) == 1


def test_lost_object_ends_the_window():
    n, logs, clock = make()
    done(n, 'pick', ['cucumber'], 1)
    clock[0] = 2.
    n._set_holding('none', 'lost')                           # executor's object_lost port
    assert n.holding == 'none' and events(logs, 'HOLD_STATS')[0]['reason'] == 'lost'
    assert events(logs, 'ROBOT_STATE')[-1]['holding'] == 'none'


def test_repeated_done_burst_logs_once_and_disabled_monitor_keeps_quiet():
    n, logs, _ = make(monitor=False)
    for _ in range(3):                                       # DONE arrives 3x
        done(n, 'pick', ['cucumber'], 1)
    assert n.holding == 'cucumber' and len(events(logs, 'ROBOT_STATE')) == 1
    assert events(logs, 'HOLD_MONITOR') == [] and n._hold_stats is None


def test_failed_pick_clears_holding():
    n, logs, _ = make()
    done(n, 'pick', ['cucumber'], 1)
    n._sent['vla'] = ('p1', 2, 'pick', ['cucumber'])
    n._update_robot_state('vla', SubtaskState(plan_id='p1', index=2, status=SubtaskState.FAILED))
    assert n.holding == 'none' and events(logs, 'HOLD_STATS')[0]['reason'] == 'pick_failed'
