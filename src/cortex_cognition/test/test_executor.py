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
    def __init__(self, detector=None, params=None):
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
        self.x = ex.Executor(CFG, ports, params or ex.Params())

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
    """module accepts and completes the current step"""
    h.state(which, ex.RUNNING, pid, i)
    h.advance(0.5)
    h.state(which, ex.DONE, pid, i)


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
    assert h.cancels == [('nav', 'p1', 0)] and h.x.phase == 'IDLE'


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


def test_user_stop_during_vla_is_deferred_to_safe_point():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', CUCUMBER)
    run_step(h, 'nav', 'p1', 0)
    h.state('vla', ex.RUNNING, 'p1', 1)
    h.x.stop('user')
    assert h.cancels == [] and h.x.phase == 'STOPPING'   # no cancel for vla
    h.state('vla', ex.DONE, 'p1', 1)                    # safe point reached
    assert len(h.cmds) == 2                             # step 2 NOT dispatched
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
