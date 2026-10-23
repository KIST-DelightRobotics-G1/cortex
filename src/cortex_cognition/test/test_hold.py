# -*- coding: utf-8 -*-
"""Holding: the picked object must stay in the gripper until it is put down (hold.py)."""
import pytest

from cortex_cognition import executor as ex
from cortex_cognition import hold
from test_executor import Harness, run_step

# carrying the cucumber to the table, as the plan streams after the pick
CARRY = [('move_to', ['table']), ('place', ['cucumber', 'table'])]


class Hand:
    """What the orchestrator would answer: the held object and the gripper camera state."""

    def __init__(self, h, held='cucumber', state=hold.PRESENT, detail='0.90'):
        self.held, self.state, self.detail, self.lost = held, state, detail, []
        h.x.p.held_object = lambda: self.held
        h.x.p.gripper = lambda key: (self.state, self.detail) if key == 'cucumber' \
            else (hold.UNSUPPORTED, 'not_judged')
        h.x.p.object_lost = self._lost

    def _lost(self, key):
        self.lost.append(key)
        self.held = 'none'


def harness(**params):
    p = dict(held_monitor=True, precheck_timeout_s=1.0, precheck_retry_s=.1,
             detector_fail_open=False)
    p.update(params)
    return Harness(params=ex.Params(**p))


def events(h, name):
    return [f for e, f in h.events if e == name]


# --- gate: a carrying step starts only with the object in the gripper ---------------------

def test_carrying_step_starts_when_the_gripper_sees_the_object():
    h = harness()
    Hand(h)
    h.x.heard('p1', '테이블에 놓아줘')
    h.plan('p1', CARRY)
    assert h.cmds == [('nav', 0, 'move_to', 'p1')]
    assert (ex.T_GROUND, 0, '오이 들고 있음') in h.traces


def test_carrying_step_waits_then_fails_when_the_object_is_not_in_the_gripper():
    h = harness()
    Hand(h, state=hold.ABSENT)
    h.x.heard('p1', 'x')
    h.plan('p1', CARRY)
    assert h.cmds == [] and h.x._st == 'PRECHECK'
    h.advance(1.1)
    assert h.cmds == [] and h.x.phase == 'IDLE'
    assert h.says[-1] == '오이를 들고 있지 않아 작업을 중단합니다.'
    assert any('not_holding' in d for _, _, _, d in h.statuses)


def test_object_seen_again_within_the_precheck_window_starts_the_step():
    h = harness()
    hand = Hand(h, state=hold.ABSENT)
    h.x.heard('p1', 'x')
    h.plan('p1', CARRY)
    h.advance(.3)
    hand.state = hold.PRESENT
    h.advance(.2)
    assert h.cmds == [('nav', 0, 'move_to', 'p1')]


def test_gripper_condition_adds_to_the_existing_precheck():
    seen = set()
    h = Harness(lambda t: (t in seen, ''), ex.Params(held_monitor=True, precheck_timeout_s=1.0,
                                                     precheck_retry_s=.1, detector_fail_open=False))
    Hand(h)
    h.x.heard('p1', '문 닫아')
    h.plan('p1', [('close', ['fridge_door'])])
    assert h.cmds == []                            # gripper fine, the door not seen yet
    seen.add('fridge_door')
    h.advance(.2)
    assert h.cmds == [('vla', 0, 'close', 'p1')]


# --- while a step runs ---------------------------------------------------------------------

def test_losing_the_object_mid_step_cancels_it_and_ends_the_plan():
    h = harness()
    hand = Hand(h)
    h.x.heard('p1', 'x')
    h.plan('p1', CARRY)
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.advance(.3)
    hand.state = hold.ABSENT
    h.advance(.1)
    assert h.cancels == [('nav', 'p1', 0)] and h.x.phase == 'STOPPING'
    assert h.says[-1] == '오이를 놓쳤습니다. 작업을 중단합니다.'
    assert hand.lost == ['cucumber']
    assert events(h, 'HOLD_LOST')[0]['action'] == 'move_to'
    h.state('nav', ex.IDLE, '', 0, 'cancelled')
    assert h.x.phase == 'IDLE' and h.cmds == [('nav', 0, 'move_to', 'p1')]
    assert ('SESSION_END', 'p1', 'failed') in h.sessions()


def test_putting_the_held_object_down_is_not_watched():
    h = harness()
    hand = Hand(h)
    h.x.heard('p1', 'x')
    h.plan('p1', CARRY)
    run_step(h, 'nav', 'p1', 0)
    assert h.cmds[-1] == ('vla', 1, 'place', 'p1')
    h.state('vla', ex.RUNNING, 'p1', 1)
    hand.state = hold.ABSENT                       # let go on purpose
    h.advance(.5)
    assert h.cancels == [] and h.x.phase == 'RUNNING'


def test_nothing_is_watched_when_disabled_or_empty_handed():
    for params, held in (({'held_monitor': False}, 'cucumber'), ({}, 'none')):
        h = harness(**params)
        Hand(h, held=held, state=hold.ABSENT)
        h.x.heard('p1', 'x')
        h.plan('p1', CARRY)
        h.state('nav', ex.RUNNING, 'p1', 0)
        h.advance(.5)
        assert h.cmds == [('nav', 0, 'move_to', 'p1')] and h.cancels == []


def test_object_the_detector_does_not_judge_is_not_watched():
    h = harness()
    Hand(h, held='apple', state=hold.ABSENT)
    h.x.heard('p1', 'x')
    h.plan('p1', [('move_to', ['table'])])
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.advance(.5)
    assert h.cmds == [('nav', 0, 'move_to', 'p1')] and h.cancels == []


def test_a_pick_is_not_gated_by_what_is_already_held():
    h = harness()
    Hand(h, state=hold.ABSENT)
    h.x.heard('p1', 'x')
    h.plan('p1', [('pick', ['cucumber'])])
    assert h.cmds == [('vla', 0, 'pick', 'p1')]


# --- gripper camera / detector unavailable -------------------------------------------------

def test_unknown_warns_and_goes_on_by_default():
    h = harness()
    Hand(h, state=hold.UNKNOWN, detail='stale')
    h.x.heard('p1', 'x')
    h.plan('p1', CARRY)
    assert h.cmds == [('nav', 0, 'move_to', 'p1')]
    assert (ex.T_GROUND, 0, '오이 들고 있는지 확인 불가 · 진행') in h.traces
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.advance(.5)
    assert h.cancels == [] and len(events(h, 'HOLD_UNKNOWN')) == 1   # logged once per step


def test_unknown_blocks_and_stops_when_configured_to_fail():
    h = harness(held_unknown='fail')
    hand = Hand(h, state=hold.UNKNOWN, detail='stale')
    h.x.heard('p1', 'x')
    h.plan('p1', CARRY)
    assert h.cmds == []
    hand.state = hold.PRESENT
    h.advance(.2)
    h.state('nav', ex.RUNNING, 'p1', 0)
    hand.state, hand.detail = hold.UNKNOWN, 'camera_not_ok'
    h.advance(.1)
    assert h.cancels == [('nav', 'p1', 0)]
    assert events(h, 'HOLD_LOST')[0]['detail'] == 'camera_not_ok'


def test_bad_unknown_policy_is_rejected():
    with pytest.raises(ValueError, match='held_unknown'):
        harness(held_unknown='ignore')


# --- hold.py ---------------------------------------------------------------------------------

def test_gripper_view_never_reports_an_old_state():
    v = hold.GripperView(timeout_s=1.0)
    assert v.state('cucumber', 0.) == (hold.UNKNOWN, 'no_message')
    v.update(10., True, {'cucumber': (hold.PRESENT, .9)})
    assert v.state('cucumber', 10.5) == (hold.PRESENT, '0.90')
    assert v.state('apple', 10.5) == (hold.UNSUPPORTED, 'not_judged')
    assert v.state('cucumber', 11.2) == (hold.UNKNOWN, 'stale')
    v.update(12., False, {'cucumber': (hold.PRESENT, .9)})
    assert v.state('cucumber', 12.) == (hold.UNKNOWN, 'camera_not_ok')


def test_hold_stats_share_of_each_state():
    s = hold.HoldStats('cucumber', 0.)
    assert s.sample(0., hold.PRESENT)               # first sample
    assert not s.sample(6., hold.PRESENT)
    assert s.sample(6., hold.ABSENT)
    assert s.sample(8., hold.UNKNOWN)
    out = s.summary(10.)
    assert out == dict(object='cucumber', duration_s=10.0, changes=2,
                       present=.6, absent=.2, unknown=.2)
