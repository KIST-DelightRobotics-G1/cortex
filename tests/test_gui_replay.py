"""Where gui_bridge starts replaying to a late client (a refreshed display)."""
import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 and colcon-built interfaces')
from cortex_gui.gui_bridge_node import replay_start


def buf(*events):
    """(plan_id, kind[, 'stop']) -> the bridge's buffer rows (seq, plan_id, kind, stop, json)."""
    return [(n, e[0], e[1], len(e) > 2, f'{e[0]}:{e[1]}') for n, e in enumerate(events)]


def replayed(events):
    return [e[-1] for e in events[replay_start(events):]]


def test_small_talk_while_running_keeps_the_plan():
    events = buf(('p1', 'HEARD'), ('p1', 'PLAN_LINE'), ('p1', 'STEP_START'),
                 ('p2', 'HEARD'), ('p2', 'THINKING'), ('p2', 'REPLY'),       # "지금 뭐 해?"
                 ('p1', 'STEP_DONE'), ('p1', 'STEP_START'))
    assert replayed(events)[0] == 'p1:HEARD' and len(replayed(events)) == 8


def test_new_plan_after_preemption_starts_at_its_own_utterance():
    events = buf(('p1', 'HEARD'), ('p1', 'PLAN_LINE'), ('p1', 'STEP_START'),
                 ('p2', 'HEARD'), ('p1', 'CANCEL'), ('p2', 'PLAN_LINE'), ('p2', 'STEP_START'))
    assert replayed(events) == ['p2:HEARD', 'p1:CANCEL', 'p2:PLAN_LINE', 'p2:STEP_START']


def test_stop_word_is_not_where_the_plan_began():
    events = buf(('p1', 'HEARD'), ('p1', 'PLAN_LINE'), ('p1', 'HEARD', 'stop'), ('p1', 'CANCEL'))
    assert replayed(events)[0] == 'p1:HEARD' and len(replayed(events)) == 4


def test_without_any_plan_the_newest_utterance():
    events = buf(('p1', 'HEARD'), ('p1', 'REPLY'), ('p2', 'HEARD'), ('p2', 'NOTE'))
    assert replayed(events) == ['p2:HEARD', 'p2:NOTE']
    assert replay_start([]) == 0
