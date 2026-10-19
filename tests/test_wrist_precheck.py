"""Fresh wrist evidence gates each cucumber-carrying step; ego stays advisory."""
from pathlib import Path

import pytest

from cortex_cognition import executor as ex, planner, rewrite
from cortex_perception.detection import DetectionRecord, PresenceWindow

ROOT = Path(__file__).resolve().parents[1]
CFG = planner.load_config(str(ROOT/'src/cortex_cognition/config/actions.yaml'))
CUCUMBER = DetectionRecord('cucumber', .9, .7, .7, .2, .2)
FRIDGE = DetectionRecord('refrigerator', .9, .5, .5, .8, .8)


class Trial:
    def __init__(self, held='cucumber', enabled=True, fail_open=False, rewriter=None):
        self.t, self.held = 1., held
        self.sent, self.calls, self.status, self.speech = [], [], [], []
        self.head = PresenceWindow({'fridge_door': {'refrigerator'}, 'cucumber': {'cucumber'}},
                                   ['refrigerator', 'cucumber'], min_frames=3, require_latest_hit=True)
        self.wrist = PresenceWindow({'held_cucumber': {'cucumber'}}, ['cucumber'], min_frames=3,
                                    require_latest_hit=True, check_roi=(.5, .5, 1., 1.), roi_min_overlap=.5)
        self.failure = None
        self.ports = ex.Ports(now=lambda: self.t, send_cmd=self.send,
            send_cancel=lambda *args: None, check_target=self.check_head, say=self.speech.append,
            stop_speech=lambda: None, trace=lambda *args: None, status=lambda *a: self.status.append(a),
            held_object=lambda: self.held, check_holding=self.check_wrist)
        self.executor = ex.Executor(CFG, self.ports, ex.Params(precheck_timeout_s=1.,
            precheck_retry_s=.1, detector_fail_open=fail_open, holding_precheck_enabled=enabled),
            rewriter=rewriter)

    def send(self, module, step, pid):
        self.sent.append((step.action, step.index, pid))

    def check_head(self, target):
        self.calls.append(('ego', target))
        r = self.head.check(target, self.t)
        return r.found, r.detail

    def check_wrist(self, since):
        self.calls.append(('wrist', since))
        if self.failure is not None:
            return False, self.failure
        r = self.wrist.check('held_cucumber', self.t, max_age_s=max(1e-6, self.t-since))
        return r.found, r.detail

    def frame(self, head=(FRIDGE,), wrist=(CUCUMBER,)):
        self.t += .125
        stamp = round(self.t*1e9)
        self.head.add(head, self.t, stamp, self.t)
        self.wrist.add(wrist, self.t, stamp, self.t)
        self.executor.tick()

    def start(self, action='close', args=None):
        self.executor.heard('p1', 'test')
        self.executor.on_step('p1', 0, 0, action, args or ['fridge_door'], '', '', '')


@pytest.mark.parametrize('head', [(FRIDGE, CUCUMBER), ()])
@pytest.mark.parametrize('wrist,expected', [((CUCUMBER,), True), ((), False),
    ((DetectionRecord('cucumber', .99, .1, .1, .1, .1),), False)])
def test_close_requires_only_wrist_evidence_in_gripper_region(head, wrist, expected):
    t = Trial()
    t.start()
    for _ in range(3):
        t.frame(head=head, wrist=wrist)
    assert bool(t.sent) is expected   # ego cucumber never substitutes for missing wrist evidence


def test_observations_before_close_precheck_cannot_authorize():
    t = Trial()
    for _ in range(3):
        t.frame()
    t.t += .01  # the previous source frame predates entry to PRECHECK
    t.start()
    assert not t.sent
    for _ in range(2):
        t.frame()
    assert not t.sent
    t.frame()
    assert [s[0] for s in t.sent] == ['close']


def test_wrist_can_recover_while_ego_becomes_invisible():
    t = Trial()
    t.start()
    for _ in range(3):
        t.frame(wrist=())
    assert not t.sent
    for _ in range(3):
        t.frame(head=())
    assert [s[0] for s in t.sent] == ['close']
    assert all(source == 'wrist' for source, _ in t.calls)


@pytest.mark.parametrize('failure', ['no_detector', 'stale', 'model_not_loaded',
                                     'inference_error', 'invalid_holding_evidence'])
def test_wrist_errors_block_even_with_legacy_fail_open(failure):
    t = Trial(fail_open=True)
    t.failure = failure
    t.start()
    for _ in range(3):
        t.frame()
    t.t += 1.
    t.executor.tick()
    assert not t.sent and t.executor.phase == 'IDLE'
    assert any('held_cucumber' in str(s) for s in t.status)


@pytest.mark.parametrize('fail_open', [False, True])
def test_guarded_close_never_waits_for_the_ego_gate(fail_open):
    t = Trial(fail_open=fail_open)
    def ego_must_not_be_called(target):
        raise AssertionError('a required ego query would consume the wrist deadline')
    t.ports.check_target = ego_must_not_be_called
    t.start()
    t.head.invalidate('no_frame')
    for _ in range(3):
        t.t += .125
        t.wrist.add([CUCUMBER], t.t, round(t.t*1e9), t.t)
        t.executor.tick()
    assert [s[0] for s in t.sent] == ['close']


@pytest.mark.parametrize('held,enabled,action,args', [
    ('none', True, 'close', ['fridge_door']), ('apple', True, 'close', ['fridge_door']),
    ('cucumber', False, 'close', ['fridge_door']), ('cucumber', True, 'open', ['fridge_door']),
    ('cucumber', True, 'pick', ['cucumber']),
    ('none', True, 'take_out', ['cucumber', 'fridge'])])
def test_other_actions_and_disabled_configuration_keep_existing_behavior(held, enabled, action, args):
    t = Trial(held=held, enabled=enabled)
    for _ in range(3):
        t.frame(head=(FRIDGE, CUCUMBER), wrist=())
    t.start(action, args)
    assert len(t.sent) == 1 and all(source != 'wrist' for source, _ in t.calls)



@pytest.mark.parametrize('held,enabled,action,args', [
    ('none', True, 'close', ['fridge_door']), ('apple', True, 'close', ['fridge_door']),
    ('cucumber', False, 'close', ['fridge_door']), ('cucumber', True, 'open', ['fridge_door']),
    ('none', True, 'pick', ['cucumber']), ('none', True, 'take_out', ['cucumber', 'fridge'])])
def test_other_prechecks_still_block_missing_ego_even_when_wrist_sees_cucumber(held, enabled, action, args):
    t = Trial(held=held, enabled=enabled)
    t.start(action, args)
    for _ in range(3):
        t.frame(head=())
    t.t += 1.
    t.executor.tick()
    assert not t.sent and t.executor.phase == 'IDLE'
    assert all(source == 'ego' for source, _ in t.calls)


def test_miss_cannot_remove_the_latched_holding_requirement():
    t = Trial()
    t.start()
    t.held = 'none'  # even a later metadata change cannot bypass this pending check
    for _ in range(3):
        t.frame(wrist=())
    assert t.executor._holding_required and not t.sent


@pytest.mark.parametrize('head', [(), (FRIDGE, CUCUMBER)], ids=['ego_missing', 'ego_visible'])
@pytest.mark.parametrize('blocked_index,failure', [(None, None)] + [
    (index, failure) for index in (1, 2, 3) for failure in ('', 'stale', 'no_detector')])
def test_carrying_sequence_rechecks_wrist_before_each_vla_and_nav_step(head, blocked_index, failure):
    rw = rewrite.Rewriter.load(str(ROOT/'src/cortex_cognition/config/plan_rewrites.yaml'), CFG)
    t = Trial(held='none', fail_open=True, rewriter=rw)
    for _ in range(3):
        t.frame(head=(FRIDGE, CUCUMBER), wrist=())
    t.start('pick', ['cucumber'])
    t.executor.on_step('p1', 0, 1, 'close', ['fridge_door'], '', '', '')
    t.executor.on_step('p1', 0, 2, 'move_to', ['table'], '', '', '')
    t.executor.on_step('p1', 1, 3, '', [], '', '', '')
    actions = ['pick', 'step_back', 'close', 'move_to']
    assert [s.action for s in t.executor.steps.values()] == actions
    assert [s.exec for s in t.executor.steps.values()] == ['vla', 'vla', 'vla', 'nav']
    # The node sets expected holding from matching DONE before forwarding to executor.
    t.held = 'cucumber'
    t.calls.clear()
    for index in (1, 2, 3):
        t.executor.on_state('vla', ex.DONE, 'p1', index-1, '', 1.)
        assert [s[0] for s in t.sent] == actions[:index]  # DONE is not IDLE
        t.t += .01  # all earlier source frames now predate this step's PRECHECK
        t.executor.on_state('vla', ex.IDLE, '', 0, '', 0.)
        assert t.executor._st == 'PRECHECK' and t.executor._holding_required
        assert [s[0] for s in t.sent] == actions[:index]
        if index == blocked_index:
            t.failure = failure or None
            for _ in range(3):
                t.frame(head=head, wrist=())
            t.t += 1.
            t.executor.tick()
            assert t.executor.phase == 'IDLE'
            assert [s[0] for s in t.sent] == actions[:index]
            assert any('held_cucumber' in str(s) for s in t.status)
            break
        for _ in range(2):
            t.frame(head=head)
            assert [s[0] for s in t.sent] == actions[:index]
        t.frame(head=head)
        assert [s[0] for s in t.sent] == actions[:index+1]
    else:
        t.executor.on_state('nav', ex.DONE, 'p1', 3, '', 1.)
        t.executor.on_state('nav', ex.IDLE, '', 0, '', 0.)
        assert t.executor.phase == 'IDLE'
        assert [s[0] for s in t.status[-2:]] == [ex.S_SUCCEEDED, ex.S_IDLE]
    assert all(source == 'wrist' for source, _ in t.calls)


@pytest.mark.parametrize('held,enabled,destination,guarded', [
    ('cucumber', True, 'table', True), ('none', True, 'table', False),
    ('apple', True, 'table', False), ('cucumber', False, 'table', False),
    ('cucumber', True, 'fridge', False), ('cucumber', True, 'home', False)])
def test_only_cucumber_carrying_table_navigation_requires_wrist(held, enabled, destination, guarded):
    t = Trial(held=held, enabled=enabled)
    t.start('move_to', [destination])
    assert t.executor._holding_required is guarded
    assert bool(t.sent) is not guarded
    if guarded:
        t.held = 'none'  # pending requirements cannot disappear on a metadata change
        t.t += 1.1
        t.executor.tick()
        assert not t.sent and t.executor.phase == 'IDLE'
    else:
        assert not t.calls


def test_busy_nav_must_be_idle_before_collecting_table_wrist_evidence():
    t = Trial()
    t.executor.on_state('nav', ex.RUNNING, 'old', 0, '', .5)
    t.start('move_to', ['table'])
    assert t.executor._st == 'WAIT_READY' and not t.calls and not t.sent
    for _ in range(3):
        t.frame()
    t.t += .01
    t.executor.on_state('nav', ex.IDLE, '', 0, '', 0.)
    assert t.executor._st == 'PRECHECK' and not t.sent
    for _ in range(2):
        t.frame(head=())
        assert not t.sent
    t.frame(head=())
    assert [s[0] for s in t.sent] == ['move_to']


def test_requirement_survives_new_plan_but_release_removes_it():
    t = Trial()
    t.start()
    t.t += 1.1
    t.executor.tick()
    # A retry command is still guarded even though the previous plan ended.
    t.start()
    assert t.executor._holding_required and not t.sent
    t.t += 1.1
    t.executor.tick()
    t.held = 'none'  # completed place/put_in/handover, not a detector miss
    for _ in range(3):
        t.frame(wrist=())
    t.start()
    assert not t.executor._holding_required and len(t.sent) == 1


def test_high_confidence_outside_roi_does_not_hide_a_valid_candidate():
    t = Trial()
    outside = DetectionRecord('cucumber', .99, .1, .1, .1, .1)
    t.start()
    for _ in range(3):
        t.frame(wrist=(outside, CUCUMBER))
    assert len(t.sent) == 1
    r = t.wrist.check('held_cucumber', t.t)
    assert r.detection == CUCUMBER


def test_latest_miss_and_expired_frames_do_not_pass():
    t = Trial()
    for _ in range(3):
        t.frame()
    t.frame(wrist=())
    assert not t.wrist.check('held_cucumber', t.t).found
    assert not t.wrist.check('held_cucumber', t.t+1.).found


@pytest.mark.parametrize('roi,overlap', [([0., 0., 1.], .5), ([.8, .1, .3, 1.], .5),
    ([float('nan'), 0., 1., 1.], .5), ([0., 0., 1., 1.], 1.1)])
def test_invalid_roi_is_rejected(roi, overlap):
    with pytest.raises(ValueError, match='ROI'):
        PresenceWindow({}, [], check_roi=roi, roi_min_overlap=overlap)
