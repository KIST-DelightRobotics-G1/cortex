# -*- coding: utf-8 -*-
"""executor state machine with a fake clock and recorded ports.

Scenarios follow Subtask_state_interface_spec v0.1 (06절 sequences).
"""
import os

from cortex_cognition import executor as ex
from cortex_cognition import planner

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = planner.load_config(os.path.join(HERE, '..', 'config', 'actions.yaml'))

SUB, END, REPLY, ERROR = 0, 1, 2, 3


class Harness:
    def __init__(self, detector=None, params=None, rewriter=None):
        self.t = 0.0
        self.cmds, self.cancels, self.says, self.traces, self.statuses = [], [], [], [], []
        self.detector = detector or (lambda target: (True, 'ok'))
        ports = ex.Ports(
            now=lambda: self.t,
            send_cmd=lambda which, step, pid: self.cmds.append((which, step.index, step.action, pid)),
            send_cancel=lambda which, pid, i: self.cancels.append((which, pid, i)),
            check_target=lambda target: self.detector(target),
            say=lambda s: self.says.append(s),
            stop_speech=lambda: self.says.append('<stop>'),
            trace=lambda k, pid, i, title, body: self.traces.append((k, i, title)),
            status=lambda st, task, sub, i, n, d: self.statuses.append((st, i, n, d)),
        )
        self.x = ex.Executor(CFG, ports, params or ex.Params(), rewriter=rewriter)

    # --- helpers -----------------------------------------------------------
    def advance(self, dt: float, step: float = 0.1):
        end = self.t + dt
        while self.t + step <= end + 1e-9:
            self.t += step
            self.x.tick()

    def plan(self, pid, lines, end=True):
        for i, (a, args) in enumerate(lines):
            self.x.on_step(pid, SUB, i, a, args, f'{a} say', '', '')
        if end:
            self.x.on_step(pid, END, len(lines), '', [], '', '', '')

    def state(self, which, status, pid, i, detail=''):
        self.x.on_state(which, status, pid, i, detail, 0.0)

    def kinds(self):
        return [k for k, _, _ in self.traces]


CUCUMBER = [('move_to', ['fridge']), ('open', ['fridge_door']), ('pick', ['cucumber']),
            ('close', ['fridge_door']), ('move_to', ['user']), ('handover', ['cucumber', 'user'])]


def run_step(h, which, pid, i):
    """module accepts and completes the current step: RUNNING → DONE x3 → IDLE"""
    h.state(which, ex.RUNNING, pid, i)
    h.advance(0.5)
    for _ in range(3):
        h.state(which, ex.DONE, pid, i)
        h.advance(0.1)
    h.state(which, ex.IDLE, '', 0)


def test_happy_path_streams_and_completes():
    h = Harness()
    assert h.x.heard('p1', '냉장고에서 오이 가져와줘') == 'idle'
    # first line arrives → step 0 dispatched before END
    h.x.on_step('p1', SUB, 0, 'move_to', ['fridge'], '냉장고로 갑니다.', '', '')
    assert h.cmds == [('nav', 0, 'move_to', 'p1')]
    assert '냉장고로 갑니다.' in h.says
    run_step(h, 'nav', 'p1', 0)
    # step 1 not here yet → WAIT_STEP; then it arrives
    assert h.x._st == 'WAIT_STEP'
    h.x.on_step('p1', SUB, 1, 'open', ['fridge_door'], '문을 엽니다.', '', '')
    assert h.cmds[-1] == ('vla', 1, 'open', 'p1')
    assert (ex.T_GROUND, 1, '냉장고 문 확인됨') in h.traces
    for i, (a, args) in enumerate(CUCUMBER[2:], start=2):
        h.x.on_step('p1', SUB, i, a, args, f'{a}', '', '')
    h.x.on_step('p1', END, 6, '', [], '', '', '')
    for i in range(1, 6):
        run_step(h, planner.exec_of(CFG, CUCUMBER[i][0]), 'p1', i)
    assert h.x.phase == 'IDLE'
    assert ex.T_PLAN_DONE in h.kinds()
    assert h.statuses[-2][0] == ex.S_SUCCEEDED


def test_reply_none_says_and_stays_idle():
    h = Harness()
    h.x.heard('p1', '사과 가져와')
    h.x.on_step('p1', REPLY, 0, '', [], '사과는 아직 못 합니다.', 'none', '')
    assert h.says == ['사과는 아직 못 합니다.'] and h.x.phase == 'IDLE' and h.cmds == []


def test_confirm_then_yes_uses_awaiting_state():
    h = Harness()
    h.x.heard('p1', '오리 가져와')
    h.x.on_step('p1', REPLY, 0, '', [], '오이 말씀이신가요?', 'confirm', '')
    assert h.x.phase == 'CONFIRM'
    assert h.x.heard('p2', '응').startswith('awaiting_confirm:')
    h.x.on_step('p2', SUB, 0, 'move_to', ['fridge'], '갑니다.', '', '')
    assert h.cmds == [('nav', 0, 'move_to', 'p2')]


def test_not_accepted_resends_once_then_fails():
    h = Harness()
    h.x.heard('p1', 'x')
    h.x.on_step('p1', SUB, 0, 'move_to', ['fridge'], '갑니다.', '', '')
    h.advance(0.6)
    assert len(h.cmds) == 2                       # one resend
    h.advance(0.6)
    assert h.x.phase == 'IDLE'
    assert any('시작하지 못했습니다' in s for s in h.says)
    assert (ex.S_FAILED, 0, 1, 'not accepted') in h.statuses


def test_stale_state_fails_step():
    h = Harness()
    h.x.heard('p1', 'x')
    h.x.on_step('p1', SUB, 0, 'move_to', ['fridge'], '갑니다.', '', '')
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.advance(1.2)                                # no messages for > stale_s
    assert h.x.phase == 'IDLE' and any('응답하지 않습니다' in s for s in h.says)


def test_step_timeout_cancels_and_fails():
    h = Harness(params=ex.Params(step_timeout_s={'nav': 2.0, 'vla': 30.0}))
    h.x.heard('p1', 'x')
    h.x.on_step('p1', SUB, 0, 'move_to', ['fridge'], '갑니다.', '', '')
    for _ in range(25):                           # module keeps reporting RUNNING
        h.state('nav', ex.RUNNING, 'p1', 0)
        h.advance(0.1)
    # one cancel (resent once if still unanswered after cancel_ack_s)
    assert h.cancels[0] == ('nav', 'p1', 0) and len(h.cancels) <= 2
    assert set(h.cancels) == {('nav', 'p1', 0)} and h.x.phase == 'STOPPING'
    assert (ex.S_FAILED, 0, 1, 'timeout') in h.statuses
    h.state('nav', ex.IDLE, '', 0, 'cancelled')   # rule 2: over only once the module is IDLE
    assert h.x.phase == 'IDLE'


def test_module_failed_aborts_plan():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER)
    run_step(h, 'nav', 'p1', 0)
    h.state('vla', ex.RUNNING, 'p1', 1)
    h.state('vla', ex.FAILED, 'p1', 1, 'unsupported: action open')
    assert h.x.phase == 'IDLE' and (ex.T_STEP_FAILED, 1, 'unsupported: action open') in h.traces
    assert len(h.cmds) == 2                       # nothing after the failure


def test_precheck_not_visible_blocks_step():
    h = Harness(detector=lambda t: (False, ''))    # a real "not found"
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER)
    run_step(h, 'nav', 'p1', 0)
    assert h.cmds == [('nav', 0, 'move_to', 'p1')]  # open was never sent
    assert '냉장고 문이 보이지 않습니다.' in h.says and h.x.phase == 'IDLE'


def test_precheck_no_detector_is_fail_open():
    h = Harness(detector=lambda t: (False, 'no_detector'))
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER)
    run_step(h, 'nav', 'p1', 0)
    assert h.cmds[-1] == ('vla', 1, 'open', 'p1')


def test_user_stop_during_nav_cancels_immediately():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER)
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.x.stop('user')
    assert h.cancels == [('nav', 'p1', 0)] and h.x.phase == 'STOPPING'
    h.state('nav', ex.IDLE, '', 0, 'cancelled')
    assert h.x.phase == 'IDLE' and '멈춥니다.' in h.says


def test_user_stop_during_vla_cancels_immediately():
    """rule 2: vla gets the cancel too; deferring to a safe point is the module's call"""
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER)
    run_step(h, 'nav', 'p1', 0)
    h.state('vla', ex.RUNNING, 'p1', 1)
    h.x.stop('user')
    assert h.cancels == [('vla', 'p1', 1)] and h.x.phase == 'STOPPING'
    for _ in range(3):                                  # module chose to finish at its safe point
        h.state('vla', ex.DONE, 'p1', 1, 'cancelled_at_safe_point')
    assert len(h.cmds) == 2                             # step 2 NOT dispatched
    assert h.kinds().count(ex.T_STEP_DONE) == 2         # nav + this one, shown once
    h.state('vla', ex.IDLE, '', 0)
    assert h.x.phase == 'IDLE'


def test_new_command_preempts_running_plan():
    h = Harness()
    h.x.heard('p1', '냉장고 오이')
    h.plan('p1', CUCUMBER)
    h.state('nav', ex.RUNNING, 'p1', 0)
    assert h.x.heard('p2', '테이블로 가') == 'running:move_to'
    # chat reply while running: nothing changes
    h.x.on_step('p2', REPLY, 0, '', [], '네, 가고 있어요.', 'chat', '')
    assert h.x.phase == 'RUNNING' and h.x.plan_id == 'p1'
    # a real new plan: first line preempts
    h.x.heard('p3', '테이블로 가')
    h.x.on_step('p3', SUB, 0, 'move_to', ['table'], '테이블로 갑니다.', '', '')
    assert h.cancels == [('nav', 'p1', 0)] and h.x.phase == 'STOPPING'
    h.x.on_step('p3', END, 1, '', [], '', '', '')
    h.state('nav', ex.IDLE, '', 0, 'cancelled')          # module clean → new plan starts
    assert h.x.plan_id == 'p3' and h.cmds[-1] == ('nav', 0, 'move_to', 'p3')
    run_step(h, 'nav', 'p3', 0)
    assert h.x.phase == 'IDLE' and h.statuses[-2][0] == ex.S_SUCCEEDED


def test_validation_error_mid_plan_finishes_vla_step_then_stops():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER[:2], end=False)
    run_step(h, 'nav', 'p1', 0)
    h.state('vla', ex.RUNNING, 'p1', 1)
    h.x.on_step('p1', ERROR, 0, '', [], '', '', 'arg sink not in vocab')
    assert h.x.phase == 'STOPPING' and h.cancels == []
    h.state('vla', ex.DONE, 'p1', 1)
    h.state('vla', ex.IDLE, '', 0)
    assert h.x.phase == 'IDLE' and len(h.cmds) == 2


def test_validation_error_shows_the_spoken_sentence():
    """막힌 이유가 화면에도 그대로 간다. 안내 문구는 actions.yaml 에서 온다."""
    h = Harness()
    h.x.heard('p1', '화장실로 가')
    h.x.on_step('p1', ERROR, 0, '', [], '', '', 'unknown_place|bathroom')
    said = planner.phrase(CFG, 'unknown_place', 'bathroom')
    assert h.says[-1] == said
    assert (ex.T_NOTE, -1, said) in h.traces


def test_plan_stall_times_out():
    h = Harness()
    h.x.heard('p1', 'x')
    h.x.on_step('p1', SUB, 0, 'move_to', ['fridge'], '갑니다.', '', '')
    run_step(h, 'nav', 'p1', 0)
    h.advance(5.5)                                # END never comes
    assert h.x.phase == 'IDLE' and (ex.S_FAILED, 0, 1, 'plan stalled') in h.statuses


def test_late_state_from_old_plan_is_ignored():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER[:1])
    h.state('nav', ex.DONE, 'p0', 0)              # stale report from a previous plan
    assert h.x.phase == 'RUNNING' and h.x._st == 'ACCEPT'


# --- rule 1: the next Cmd waits for the previous module's IDLE ---------------------------

def test_next_step_waits_for_idle_not_done():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER)
    h.state('nav', ex.RUNNING, 'p1', 0)
    for _ in range(3):                            # DONE x3 at 10 Hz
        h.state('nav', ex.DONE, 'p1', 0)
        h.advance(0.1)
    assert h.cmds == [('nav', 0, 'move_to', 'p1')] and h.x._st == 'WAIT_IDLE'
    assert ex.T_STEP_DONE in h.kinds()            # the screen still hears about DONE at once
    h.state('nav', ex.IDLE, '', 0)
    assert h.cmds[-1] == ('vla', 1, 'open', 'p1')


def test_same_module_back_to_back_waits_for_idle():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER)
    run_step(h, 'nav', 'p1', 0)
    h.state('vla', ex.RUNNING, 'p1', 1)
    h.state('vla', ex.DONE, 'p1', 1)
    assert h.cmds[-1] == ('vla', 1, 'open', 'p1')  # pick not sent while vla says DONE
    h.state('vla', ex.IDLE, '', 0)
    assert h.cmds[-1] == ('vla', 2, 'pick', 'p1')


def test_last_step_waits_for_idle_before_plan_done():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER[:1])
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.state('nav', ex.DONE, 'p1', 0)
    assert ex.T_PLAN_DONE not in h.kinds() and h.x.phase == 'RUNNING'
    h.state('nav', ex.IDLE, '', 0)
    assert ex.T_PLAN_DONE in h.kinds() and h.x.phase == 'IDLE'


def test_done_without_idle_fails_the_plan():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER)
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.state('nav', ex.DONE, 'p1', 0)
    h.advance(1.2)                                # idle_wait_s 1.0, IDLE never comes
    assert h.x.phase == 'IDLE' and len(h.cmds) == 1
    assert '이동 모듈이 응답하지 않습니다.' in h.says
    assert (ex.S_FAILED, 0, 6, 'did not go idle') in h.statuses


# --- rule 2: every stop is cancel → IDLE ---------------------------------------------

def test_new_command_preempts_running_vla_with_a_cancel():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER)
    run_step(h, 'nav', 'p1', 0)
    h.state('vla', ex.RUNNING, 'p1', 1)
    h.x.heard('p2', '테이블로 가')
    h.x.on_step('p2', SUB, 0, 'move_to', ['table'], '테이블로 갑니다.', '', '')
    assert h.cancels == [('vla', 'p1', 1)] and h.x.phase == 'STOPPING'
    h.x.on_step('p2', END, 1, '', [], '', '', '')
    h.state('vla', ex.IDLE, '', 0, 'cancelled')
    assert h.x.plan_id == 'p2' and h.cmds[-1] == ('nav', 0, 'move_to', 'p2')


def test_module_that_does_not_stop_blocks_the_new_plan():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER)
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.x.heard('p2', '테이블로 가')
    h.x.on_step('p2', SUB, 0, 'move_to', ['table'], '테이블로 갑니다.', '', '')
    h.x.on_step('p2', END, 1, '', [], '', '', '')
    for _ in range(35):                           # past safe_stop nav 3 s, still RUNNING
        h.state('nav', ex.RUNNING, 'p1', 0)
        h.advance(0.1)
    assert h.x.phase == 'IDLE'
    assert h.cmds == [('nav', 0, 'move_to', 'p1')]  # p2 never sent
    assert planner.phrase(CFG, 'stop_failed') in h.says
    assert (ex.S_FAILED, 0, 0, 'module did not stop') in h.statuses


def test_idle_that_predates_the_cmd_is_not_trusted():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER[:1])
    h.x.stop('user')                              # the Cmd may still be in flight
    assert h.cancels == [('nav', 'p1', 0)]
    h.state('nav', ex.IDLE, '', 0)                # old IDLE, before the Cmd landed
    assert h.x.phase == 'STOPPING'
    h.advance(0.6)                                # past cancel_ack_s
    h.state('nav', ex.IDLE, '', 0, 'cancelled')
    assert h.x.phase == 'IDLE'


def test_stop_during_wait_idle_waits_without_a_cancel():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER)
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.state('nav', ex.DONE, 'p1', 0)
    h.x.stop('user')
    assert h.cancels == [] and h.x.phase == 'STOPPING'   # nothing left to cancel
    assert (ex.T_CANCEL, -1, '다음 단계 전에 중단') in h.traces
    h.state('nav', ex.IDLE, '', 0)
    assert h.x.phase == 'IDLE' and len(h.cmds) == 1


def test_stop_during_graceful_finish_cancels_now():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER[:2], end=False)
    run_step(h, 'nav', 'p1', 0)
    h.state('vla', ex.RUNNING, 'p1', 1)
    h.x.on_step('p1', ERROR, 0, '', [], '', '', 'arg sink not in vocab')
    assert h.cancels == []                        # plan error: the step may finish
    h.x.stop('user')
    assert h.cancels == [('vla', 'p1', 1)]
    h.state('vla', ex.IDLE, '', 0, 'cancelled')
    assert h.x.phase == 'IDLE'


def test_graceful_finish_is_cancelled_after_the_step_timeout():
    h = Harness(params=ex.Params(step_timeout_s={'nav': 60.0, 'vla': 2.0}))
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER[:2], end=False)
    run_step(h, 'nav', 'p1', 0)
    h.state('vla', ex.RUNNING, 'p1', 1)
    h.x.on_step('p1', ERROR, 0, '', [], '', '', 'arg sink not in vocab')
    for _ in range(25):                           # a vla that never reports DONE
        h.state('vla', ex.RUNNING, 'p1', 1)
        h.advance(0.1)
    # one cancel (resent once if still unanswered after cancel_ack_s)
    assert h.cancels[0] == ('vla', 'p1', 1) and len(h.cancels) <= 2
    assert set(h.cancels) == {('vla', 'p1', 1)} and h.x.phase == 'STOPPING'
    h.state('vla', ex.IDLE, '', 0, 'cancelled')
    assert h.x.phase == 'IDLE'


# Bounded object-presence prechecks [SYS-REQ-44]. No module dispatch until verified.
def waiting_harness(detector=None):
    return Harness(detector or (lambda t: (False, '')),
                   ex.Params(detector_fail_open=False, precheck_timeout_s=1.0, precheck_retry_s=.1))


def begin_pick(h, pid='p1'):
    h.x.heard(pid, '오이')
    h.plan(pid, [('pick', ['cucumber'])])


def test_precheck_recovers_and_sends_once_without_resetting_deadline():
    h = waiting_harness()
    begin_pick(h)
    assert h.x._st == 'PRECHECK' and not h.cmds
    h.advance(.6)
    assert not h.cmds
    h.detector = lambda t: (True, 'cucumber .8')
    h.advance(.1)
    assert h.cmds == [('vla', 0, 'pick', 'p1')]
    h.state('vla', ex.RUNNING, 'p1', 0)
    h.advance(.1)
    assert len(h.cmds) == 1


def test_precheck_persistent_absence_stops_at_deadline_and_never_late_dispatches():
    h = waiting_harness()
    begin_pick(h)
    h.t = 1.0
    h.x.tick()
    assert not h.cmds and h.x.phase == 'IDLE'
    assert any('precheck_timeout' in d for _, _, _, d in h.statuses)
    h.detector = lambda t: (True, '')
    h.advance(.3)
    assert not h.cmds


def test_precheck_done_before_dispatch_cannot_advance():
    h = waiting_harness()
    begin_pick(h)
    h.state('vla', ex.DONE, 'p1', 0)
    assert h.x._st == 'PRECHECK' and not h.cmds


def test_precheck_user_stop_needs_no_module_cancel_or_done():
    h = waiting_harness()
    begin_pick(h)
    h.x.stop()
    assert h.x.phase == 'IDLE' and not h.cancels and not h.cmds
    h.detector = lambda t: (True, '')
    h.advance(.5)
    assert not h.cmds


def test_precheck_new_plan_preempts_without_waiting_for_unsent_vla():
    h = waiting_harness()
    begin_pick(h)
    h.x.heard('p2', '테이블로')
    h.plan('p2', [('move_to', ['table'])])
    assert h.cmds == [('nav', 0, 'move_to', 'p2')] and not h.cancels


def test_precheck_unavailable_is_distinct_from_absent_and_can_recover():
    h = waiting_harness(lambda t: (False, 'insufficient_frames'))
    begin_pick(h)
    assert h.statuses[-1][-1] == 'precheck_wait: insufficient_frames'
    h.t = 1.0
    h.x.tick()
    assert any('확인할 수 없어' in s for s in h.says) and not h.cmds


def test_precheck_unsupported_target_fails_immediately():
    h = waiting_harness(lambda t: (False, 'unsupported_class'))
    begin_pick(h)
    assert h.x.phase == 'IDLE' and not h.cmds
    assert any('unsupported_class' in d for _, _, _, d in h.statuses)


def test_precheck_response_after_deadline_does_not_dispatch():
    h = waiting_harness()
    def slow(target):
        h.t = 1.1
        return True, 'late positive'
    h.detector = slow
    begin_pick(h)
    assert h.x.phase == 'IDLE' and not h.cmds


def test_precheck_queries_are_rate_limited():
    queries = []
    h = waiting_harness(lambda t: (queries.append(t) and True, 'no_frame'))
    begin_pick(h)
    h.t = .05
    h.x.tick()
    assert len(queries) == 1
    h.t = .1
    h.x.tick()
    assert len(queries) == 2


def test_precheck_after_done_waits_for_idle_and_rechecks_current_visibility():
    queried = []
    visible = {'value': True}
    def detector(target):
        queried.append(target)
        return visible['value'], ''
    h = waiting_harness(detector)
    h.x.heard('p1', 'x')
    h.plan('p1', [('open', ['fridge_door']), ('pick', ['cucumber'])])
    h.state('vla', ex.RUNNING, 'p1', 0)
    h.state('vla', ex.DONE, 'p1', 0)
    assert queried == ['fridge_door'] and h.x._st == 'WAIT_IDLE'
    visible['value'] = False
    h.advance(.3)
    h.state('vla', ex.IDLE, '', 0)
    assert queried == ['fridge_door', 'cucumber'] and h.x._st == 'PRECHECK'
    assert [c[2] for c in h.cmds] == ['open']
    h.x.stop()
    assert not h.cancels and h.x.phase == 'IDLE'


def test_precheck_recovery_then_stop_obeys_cancel_and_idle_contract():
    h = waiting_harness()
    begin_pick(h)
    h.detector = lambda target: (True, '')
    h.advance(.2)
    h.state('vla', ex.RUNNING, 'p1', 0)
    h.x.stop()
    assert h.cancels == [('vla', 'p1', 0)] and h.x.phase == 'STOPPING'
    h.state('vla', ex.DONE, 'p1', 0)
    assert h.x.phase == 'STOPPING'
    h.state('vla', ex.IDLE, '', 0)
    assert h.x.phase == 'IDLE' and len(h.cmds) == 1



def begin_take_out(h, pid='p1'):
    h.x.heard(pid, '냉장고에서 오이를 꺼내')
    h.plan(pid, [('take_out', ['cucumber', 'fridge'])])


def test_take_out_requires_both_targets_in_each_retry():
    queried = []
    visible = {'fridge'}
    h = waiting_harness(lambda t: (queried.append(t) or t in visible, ''))
    begin_take_out(h)
    assert queried == ['fridge', 'cucumber'] and not h.cmds
    assert h.statuses[-1][-1] == 'precheck_wait: cucumber: not_visible'
    visible.clear(); visible.add('cucumber')
    h.advance(.2)
    assert not h.cmds and h.x._precheck_target == 'fridge'
    visible.add('fridge')
    h.advance(.2)
    assert h.cmds == [('vla', 0, 'take_out', 'p1')]
    assert queried[-2:] == ['fridge', 'cucumber']
    h.state('vla', ex.RUNNING, 'p1', 0)
    h.advance(.2)
    assert len(h.cmds) == 1


def test_take_out_reports_actual_missing_target_at_shared_deadline():
    h = waiting_harness(lambda t: (t == 'fridge', ''))
    begin_take_out(h)
    h.t = 1.0; h.x.tick()
    assert not h.cmds and h.x.phase == 'IDLE'
    assert any('precheck_timeout: cucumber: not_visible' in d for _, _, _, d in h.statuses)
    assert any('오이' in s for s in h.says)


def test_take_out_unavailable_target_blocks_then_recovers():
    h = waiting_harness(lambda t: (True, '') if t == 'fridge' else (False, 'stale'))
    begin_take_out(h)
    assert not h.cmds and h.statuses[-1][-1] == 'precheck_wait: cucumber: stale'
    h.detector = lambda t: (True, '')
    h.advance(.2)
    assert len(h.cmds) == 1


def test_take_out_unsupported_target_is_terminal_even_if_other_target_absent():
    h = waiting_harness(lambda t: (False, '') if t == 'fridge' else (False, 'unsupported_class'))
    begin_take_out(h)
    assert h.x.phase == 'IDLE' and not h.cmds
    assert any('cucumber: unsupported_class' in d for _, _, _, d in h.statuses)


def test_take_out_slow_second_query_cannot_extend_step_deadline():
    h = waiting_harness()
    queried = []
    def slow(t):
        queried.append(t)
        h.t += .6
        return True, ''
    h.detector = slow
    begin_take_out(h)
    assert queried == ['fridge', 'cucumber'] and not h.cmds and h.x.phase == 'IDLE'
    assert any('precheck_timeout: cucumber: detector_timeout' in d for _, _, _, d in h.statuses)


def test_take_out_stop_while_waiting_sends_no_cancel_or_late_command():
    h = waiting_harness(lambda t: (t == 'fridge', ''))
    begin_take_out(h)
    h.x.stop()
    h.detector = lambda t: (True, '')
    h.advance(.2)
    assert h.x.phase == 'IDLE' and not h.cmds and not h.cancels


def test_take_out_replacement_plan_does_not_reuse_old_positive():
    h = waiting_harness(lambda t: (t == 'fridge', ''))
    begin_take_out(h)
    h.x.heard('p2', '테이블로')
    h.plan('p2', [('move_to', ['table'])])
    assert h.cmds == [('nav', 0, 'move_to', 'p2')] and not h.cancels


def test_take_out_checks_after_previous_idle_and_next_step_waits_for_its_idle():
    queried = []
    h = waiting_harness(lambda t: (queried.append(t) or True, ''))
    h.x.heard('p1', '꺼내고 닫아')
    h.plan('p1', [('open', ['fridge_door']), ('take_out', ['cucumber', 'fridge']),
                  ('close', ['fridge_door'])])
    h.state('vla', ex.DONE, 'p1', 0)
    assert queried == ['fridge_door']
    h.state('vla', ex.IDLE, '', 0)
    assert queried == ['fridge_door', 'fridge', 'cucumber']
    h.state('vla', ex.DONE, 'p1', 1)
    assert [c[2] for c in h.cmds] == ['open', 'take_out']
    h.state('vla', ex.IDLE, '', 0)
    assert [c[2] for c in h.cmds] == ['open', 'take_out', 'close']


def test_invalid_runtime_precheck_does_not_bypass_gate():
    h = waiting_harness(lambda t: (True, ''))
    h.x.cfg = {**CFG, 'precheck': {**CFG['precheck'], 'take_out': ['typo']}}
    begin_take_out(h)
    assert not h.cmds and h.x.phase == 'IDLE'
    assert any('invalid_precheck' in d for _, _, _, d in h.statuses)


def test_take_out_preserves_explicit_demo_fail_open_policy():
    h = Harness(lambda t: (False, 'no_detector'), ex.Params(detector_fail_open=True))
    begin_take_out(h)
    assert len(h.cmds) == 1



def test_take_out_with_demo_rewrite_checks_both_after_approach():
    from cortex_cognition.rewrite import Rewriter
    rw = Rewriter.load(os.path.join(HERE, '..', 'config', 'plan_rewrites.yaml'), CFG)
    visible = {'fridge_door', 'fridge'}
    h = Harness(lambda t: (t in visible, ''), ex.Params(detector_fail_open=False,
        precheck_timeout_s=1.0, precheck_retry_s=.1), rewriter=rw)
    h.x.heard('p1', '오이를 꺼내고 닫아')
    h.plan('p1', [('open', ['fridge_door']), ('take_out', ['cucumber', 'fridge']),
                  ('close', ['fridge_door'])])
    assert [h.x.steps[i].action for i in sorted(h.x.steps)] == [
        'open', 'approach', 'take_out', 'step_back', 'close']
    run_step(h, 'vla', 'p1', 0)
    run_step(h, 'vla', 'p1', 1)
    assert h.x._st == 'PRECHECK' and [c[2] for c in h.cmds] == ['open', 'approach']
    visible.add('cucumber')
    h.advance(.2)
    assert [c[2] for c in h.cmds] == ['open', 'approach', 'take_out']


# --- rule 1: a Cmd goes only to an IDLE module (no preemption by Cmd) ------------------

def test_new_plan_waits_for_the_module_to_finish_its_failed_burst():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER[:1])
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.state('nav', ex.FAILED, 'p1', 0, 'no path')  # plan p1 abandoned at the first FAILED
    assert h.x.phase == 'IDLE'
    h.x.heard('p2', '테이블로 가')
    h.x.on_step('p2', SUB, 0, 'move_to', ['table'], '테이블로 갑니다.', '', '')
    assert h.cmds == [('nav', 0, 'move_to', 'p1')]  # nav is still FAILED → not sent yet
    assert h.x._st == 'WAIT_READY' and h.cancels == []
    h.state('nav', ex.FAILED, 'p1', 0, 'no path')
    h.state('nav', ex.IDLE, '', 0)
    assert h.cmds[-1] == ('nav', 0, 'move_to', 'p2') and h.x._st == 'ACCEPT'


def test_leftover_running_module_is_cancelled_before_the_next_cmd():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER[:1])
    h.advance(1.2)                                # not accepted → p1 abandoned
    assert h.x.phase == 'IDLE' and len(h.cmds) == 2
    h.state('nav', ex.RUNNING, 'p1', 0)           # …but the module started it late
    h.x.heard('p2', '테이블로 가')
    h.x.on_step('p2', SUB, 0, 'move_to', ['table'], '테이블로 갑니다.', '', '')
    assert h.cancels == [('nav', 'p1', 0)] and len(h.cmds) == 2
    h.state('nav', ex.IDLE, '', 0, 'cancelled')
    assert h.cmds[-1] == ('nav', 0, 'move_to', 'p2')


def test_module_that_never_frees_up_gets_no_cmd():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER[:1])
    h.advance(1.2)                                # not accepted → abandoned
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.x.heard('p2', '테이블로 가')
    h.x.on_step('p2', SUB, 0, 'move_to', ['table'], '테이블로 갑니다.', '', '')
    for _ in range(35):                           # past safe_stop nav 3 s, still RUNNING
        h.state('nav', ex.RUNNING, 'p1', 0)
        h.advance(0.1)
    assert h.x.phase == 'IDLE' and len(h.cmds) == 2   # p2 never sent
    assert planner.phrase(CFG, 'stop_failed') in h.says
    assert (ex.S_FAILED, 0, 1, 'module not idle') in h.statuses


def test_unanswered_cancel_is_resent_once():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER[:1])
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.x.stop('user')
    for _ in range(20):                           # keeps RUNNING, no cancel_deferred
        h.state('nav', ex.RUNNING, 'p1', 0)
        h.advance(0.1)
    assert h.cancels == [('nav', 'p1', 0), ('nav', 'p1', 0)]
    h.state('nav', ex.IDLE, '', 0, 'cancelled')
    assert h.x.phase == 'IDLE'


def test_deferred_cancel_is_not_resent():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER[:2])
    run_step(h, 'nav', 'p1', 0)
    h.state('vla', ex.RUNNING, 'p1', 1)
    h.x.stop('user')
    for _ in range(20):                           # going to its safe point
        h.state('vla', ex.RUNNING, 'p1', 1, 'cancel_deferred: hand on handle')
        h.advance(0.1)
    assert h.cancels == [('vla', 'p1', 1)]
    h.state('vla', ex.IDLE, '', 0, 'cancelled_at_safe_point')
    assert h.x.phase == 'IDLE'


# Integration of main's module readiness with bounded, multi-target checks.
def test_take_out_waits_for_idle_before_starting_its_precheck_deadline():
    queries = []
    visible = {'fridge'}
    h = waiting_harness(lambda t: (queries.append(t) or t in visible, ''))
    h.state('vla', ex.RUNNING, 'old', 7)
    begin_take_out(h)
    assert h.x._st == 'WAIT_READY' and queries == [] and not h.cmds
    assert h.cancels == [('vla', 'old', 7)]
    h.advance(2.0)  # Longer than the precheck budget, but still waiting for IDLE.
    h.state('vla', ex.IDLE, '', 0)
    assert h.x._st == 'PRECHECK' and queries == ['fridge', 'cucumber']
    h.advance(.5)
    visible.add('cucumber')
    h.advance(.1)
    assert h.cmds == [('vla', 0, 'take_out', 'p1')]


def test_take_out_readiness_timeout_never_queries_or_dispatches():
    queries = []
    h = waiting_harness(lambda t: (queries.append(t) or True, ''))
    h.state('vla', ex.FAILED, 'old', 7)
    begin_take_out(h)
    h.advance(10.2)
    h.state('vla', ex.IDLE, '', 0)
    assert h.x.phase == 'IDLE' and not queries and not h.cmds
    assert any(d == 'module not idle' for _, _, _, d in h.statuses)


def test_take_out_busy_during_retry_rechecks_both_after_idle():
    queries = []
    visible = {'fridge'}
    h = waiting_harness(lambda t: (queries.append(t) or t in visible, ''))
    begin_take_out(h)
    assert queries == ['fridge', 'cucumber']
    h.state('vla', ex.RUNNING, 'old', 7)
    h.advance(.2)
    assert h.x._st == 'WAIT_READY' and queries == ['fridge', 'cucumber']
    assert h.cancels == [('vla', 'old', 7)] and not h.cmds
    visible.clear()
    visible.add('cucumber')  # The old fridge hit must not carry over.
    h.state('vla', ex.IDLE, '', 0)
    assert queries[-2:] == ['fridge', 'cucumber']
    assert h.x._st == 'PRECHECK' and h.x._precheck_target == 'fridge' and not h.cmds
    visible.add('fridge')
    h.advance(.2)
    assert h.cmds == [('vla', 0, 'take_out', 'p1')]


def test_take_out_ready_guard_also_applies_after_successful_queries():
    h = waiting_harness()
    def detector(target):
        if target == 'cucumber':
            h.state('vla', ex.RUNNING, 'old', 7)
        return True, ''
    h.detector = detector
    begin_take_out(h)
    assert h.x._st == 'WAIT_READY' and not h.cmds
    h.detector = lambda t: (t == 'fridge', '')
    h.state('vla', ex.IDLE, '', 0)
    assert h.x._st == 'PRECHECK' and not h.cmds
    assert h.x._precheck_target == 'cucumber'


def test_take_out_replacement_during_ready_wait_still_requires_both():
    queries = []
    h = waiting_harness(lambda t: (queries.append(t) or t == 'fridge', ''))
    h.state('vla', ex.DONE, 'old', 7)
    begin_take_out(h)
    begin_take_out(h, 'p2')
    assert h.x.plan_id == 'p2' and h.x._st == 'WAIT_READY' and not queries
    h.state('vla', ex.IDLE, '', 0)
    assert queries == ['fridge', 'cucumber'] and not h.cmds
    h.x.stop()
    h.detector = lambda t: (True, '')
    h.advance(.2)
    assert h.x.phase == 'IDLE' and not h.cmds and not h.cancels


def test_take_out_training_sentence_survives_ready_and_precheck_gates(tmp_path):
    sentence = 'Take out the cucumber.  Keep this exact sentence.'
    path = tmp_path / 'prompts.yaml'
    path.write_text('prompts:\n  "take_out(cucumber,fridge)": "' + sentence + '"\n',
                    encoding='utf-8')
    prompts, empty = planner.load_vla_prompts(str(path), CFG['actions'])
    assert not empty
    h = waiting_harness(lambda t: (t == 'fridge', ''))
    h.x.cfg = {**CFG, 'vla_prompts': prompts}
    sent = []
    h.x.p.send_cmd = lambda which, step, pid: sent.append((step.instruction, pid))
    h.state('vla', ex.DONE, 'old', 7)
    begin_take_out(h)
    assert not sent
    h.state('vla', ex.IDLE, '', 0)
    assert h.x._st == 'PRECHECK' and not sent
    h.detector = lambda t: (True, '')
    h.advance(.2)
    assert sent == [(sentence, 'p1')]
