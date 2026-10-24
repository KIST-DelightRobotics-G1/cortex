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
METHODS = ('_set_holding', '_start_hold', '_end_hold', '_sample_hold', '_on_gripper',
           '_update_robot_state', '_send_cmd')


def make(console='state,hold,hold_trace'):
    logs, clock, sent = [], [0.], []
    n = SimpleNamespace(
        gripper=hold.GripperView(1.0), _hold_stats=None,
        holding='none', at='home', _sent={}, _PICKS=hold.PICKS, _PUTS=hold.PUTS,
        cfg=dict(CFG, vla_prompts={}), _now=lambda: clock[0],
        _exec=SimpleNamespace(plan_id='p1', phase='RUNNING'),
        cmd_pub={'vla': SimpleNamespace(publish=sent.append)},
        get_logger=lambda: SimpleNamespace(info=logs.append, warning=lambda *_: None))
    n.events = EventLog(n, known=('state', 'hold', 'hold_trace', 'subtask'), topics=console)
    for name in METHODS:
        setattr(n, name, getattr(OrchestratorNode, name).__get__(n))
    return n, logs, clock


def gripper_msg(state, camera_ok=True):
    return GripperPresence(camera_ok=camera_ok,
                           objects=[ObjectPresence(key='cucumber', state=state, confidence=.9)])


def done(n, action, args, index, status=SubtaskState.DONE):
    n._sent['vla'] = ('p1', index, action, args)
    n._update_robot_state('vla', SubtaskState(plan_id='p1', index=index, status=status))


def events(logs, name):
    return [json.loads(line.split(' ', 1)[1]) for line in logs if line.startswith(name + ' ')]


def put(n, index):
    step = SimpleNamespace(action='place', args=['cucumber', 'table'], index=index, instruction='Place.')
    n._send_cmd('vla', step, 'p1')


def test_holding_window_logs_state_changes_and_stats_until_release():
    n, logs, clock = make()
    done(n, 'take_out', ['cucumber', 'fridge'], 3)
    assert n.holding == 'cucumber'
    [start] = events(logs, 'HOLD_START')
    assert (start['object'], start['state'], start['detail']) == ('cucumber', 'UNKNOWN', 'no_message')
    n._on_gripper(gripper_msg(ObjectPresence.PRESENT))
    clock[0] = 6.
    n._on_gripper(gripper_msg(ObjectPresence.ABSENT))       # e.g. occluded by the hand
    clock[0] = 7.
    n._on_gripper(gripper_msg(ObjectPresence.PRESENT))
    clock[0] = 10.
    put(n, 7)                                                # letting it go: stats end here
    states = [e['state'] for e in events(logs, 'HOLD_STATE')]
    assert states == ['PRESENT', 'ABSENT', 'PRESENT']
    [stats] = events(logs, 'HOLD_STATS')
    assert stats['reason'] == 'release' and stats['object'] == 'cucumber'
    assert stats['duration_s'] == 10.0 and stats['absent'] == .1 and stats['changes'] == 3
    assert (stats['absent_runs'], stats['absent_max_s']) == (1, 1.0)
    assert stats['unknown_why'] == ['no_message']
    done(n, 'place', ['cucumber', 'table'], 7)
    assert n.holding == 'none' and len(events(logs, 'HOLD_STATS')) == 1


def test_console_gets_window_lines_only_not_every_change():
    n, logs, clock = make(console='hold')                    # logging.yaml default
    done(n, 'pick', ['cucumber'], 1)
    for i in range(20):                                      # a flickering detector
        clock[0] = i * .1
        n._on_gripper(gripper_msg(ObjectPresence.PRESENT if i % 2 else ObjectPresence.ABSENT))
    clock[0] = 3.
    n._sample_hold()                                         # no message for 1 s: stale
    clock[0] = 3.5
    n._sample_hold()
    put(n, 2)
    names = [line.split(' ', 1)[0] for line in logs]
    assert names == ['HOLD_START', 'HOLD_UNKNOWN', 'HOLD_STATS']
    assert events(logs, 'HOLD_UNKNOWN')[0]['detail'] == 'stale'
    assert events(logs, 'HOLD_STATS')[0]['absent_runs'] == 10


def test_lost_object_ends_the_window():
    n, logs, clock = make()
    done(n, 'pick', ['cucumber'], 1)
    clock[0] = 2.
    n._set_holding('none', 'lost')                           # executor's object_lost port
    assert n.holding == 'none' and events(logs, 'HOLD_STATS')[0]['reason'] == 'lost'
    assert events(logs, 'ROBOT_STATE')[-1]['holding'] == 'none'


def test_failed_put_resumes_the_window_once_the_plan_is_over():
    n, logs, clock = make()
    done(n, 'pick', ['cucumber'], 1)
    put(n, 2)
    done(n, 'place', ['cucumber', 'table'], 2, SubtaskState.FAILED)
    assert n.holding == 'cucumber' and n._hold_stats is None
    n._sample_hold()                                         # plan still stopping
    assert n._hold_stats is None
    n._exec.phase = 'IDLE'
    n._sample_hold()
    assert events(logs, 'HOLD_START')[-1]['reason'] == 'resumed'


def test_repeated_done_burst_logs_once():
    n, logs, _ = make()
    for _ in range(3):                                       # DONE arrives 3x
        done(n, 'pick', ['cucumber'], 1)
    assert n.holding == 'cucumber' and len(events(logs, 'ROBOT_STATE')) == 1
    assert len(events(logs, 'HOLD_START')) == 1


def test_failed_pick_clears_holding():
    n, logs, _ = make()
    done(n, 'pick', ['cucumber'], 1)
    n._sent['vla'] = ('p1', 2, 'pick', ['cucumber'])
    n._update_robot_state('vla', SubtaskState(plan_id='p1', index=2, status=SubtaskState.FAILED))
    assert n.holding == 'none' and events(logs, 'HOLD_STATS')[0]['reason'] == 'pick_failed'
