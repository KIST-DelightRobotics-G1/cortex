"""executor — runs a streamed plan against the nav / VLA modules. Pure logic.

No rclpy here: the node feeds events in (PlanStep, SubtaskState, transcript,
tick) and the executor talks back through `Ports` callbacks. That keeps the
state machine unit-testable with a fake clock.

Contract implemented: Subtask_state_interface_spec v0.1
    accept_timeout_s 0.5 (+1 resend) · stale_s 1.0 · step_timeout nav 60 / vla 30
    step_wait_s 5 (next PlanStep) · cancel_ack_s 0.5 · safe_stop nav 3 / vla 10
    cancel: nav → immediately; vla → deferred until DONE/FAILED (safe point)
    IDLE from a module means "cleanup complete, ready for a new Cmd"

Phases
    IDLE       nothing running
    PLANNING   PlanRequest sent, waiting for the first PlanStep (or a reply)
    RUNNING    a step is dispatched (sub-state in `_st`: ACCEPT | RUN | WAIT_STEP)
    STOPPING   cancel sent / deferred; waiting for the module to reach IDLE
    CONFIRM    a confirm question is pending — next utterance answers it
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

from . import planner

# SubtaskState.status values (kept numeric here so the module has no ROS import)
IDLE, RUNNING, DONE, FAILED = 0, 1, 2, 3
STATUS_NAME = {IDLE: 'IDLE', RUNNING: 'RUNNING', DONE: 'DONE', FAILED: 'FAILED'}

# TraceEvent.kind (same numbering as the .msg)
T_HEARD, T_THINKING, T_PLAN_LINE, T_PLAN_END, T_REPLY, T_STEP_START, T_STEP_DONE, \
    T_STEP_FAILED, T_GROUND, T_CANCEL, T_PLAN_DONE, T_NOTE = range(12)

# TaskStatus.state
S_IDLE, S_RUNNING, S_SUCCEEDED, S_FAILED, S_PREEMPTED = range(5)


@dataclass
class Params:
    accept_timeout_s: float = 0.5
    accept_retries: int = 1
    stale_s: float = 1.0
    step_timeout_s: dict = field(default_factory=lambda: {'nav': 60.0, 'vla': 30.0})
    step_wait_s: float = 5.0
    cancel_ack_s: float = 0.5
    safe_stop_timeout_s: dict = field(default_factory=lambda: {'nav': 3.0, 'vla': 10.0})
    plan_timeout_s: float = 20.0          # PlanRequest → first line
    detector_fail_open: bool = True       # detector unavailable → proceed (only "not found" blocks)


@dataclass
class Ports:
    now: Callable[[], float]
    send_cmd: Callable[[str, 'Step', str], None]          # (exec, step, plan_id)
    send_cancel: Callable[[str, str, int], None]          # (exec, plan_id, index)
    check_target: Callable[[str], tuple]                  # target → (found: bool, detail: str)
    say: Callable[[str], None]
    stop_speech: Callable[[], None]
    trace: Callable[[int, str, int, str, str], None]      # (kind, plan_id, index, title, body)
    status: Callable[[int, str, str, int, int, str], None]  # (state, task, subtask, idx, count, detail)
    log: Callable[[str], None] = lambda s: None


@dataclass
class Step:
    index: int
    action: str
    args: list
    say: str
    exec: str
    instruction: str
    title: str
    raw: str = ''


@dataclass
class ModuleView:
    """Last SubtaskState seen from one module, with the local receive time."""
    status: int = IDLE
    plan_id: str = ''
    index: int = 0
    detail: str = ''
    progress: float = 0.0
    t_recv: float = -1.0


class Executor:
    def __init__(self, cfg: dict, ports: Ports, params: Params | None = None) -> None:
        self.cfg = cfg
        self.p = ports
        self.prm = params or Params()
        self.mod = {'nav': ModuleView(), 'vla': ModuleView()}
        self._reset()

    # ------------------------------------------------------------------ state
    def _reset(self) -> None:
        self.phase = 'IDLE'
        self.plan_id = ''
        self.utterance = ''
        self.steps: dict[int, Step] = {}
        self.count = -1                 # from END; -1 until known
        self.cur = -1                   # current step index
        self._st = ''                   # ACCEPT | RUN | WAIT_STEP
        self._t = 0.0                   # timestamp of the current sub-state entry
        self._retries = 0
        self._pending: Optional[dict] = None   # plan waiting to start after a stop
        self._stop_reason = ''
        self._confirm_summary = ''

    @property
    def state_for_llm(self) -> str:
        if self.phase == 'CONFIRM':
            return f'awaiting_confirm:{self._confirm_summary}'
        if self.phase in ('RUNNING', 'STOPPING') and self.cur in self.steps:
            return f'running:{self.steps[self.cur].action}'
        return 'idle'

    def _cur(self) -> Optional[Step]:
        return self.steps.get(self.cur)

    def _status(self, state: int, detail: str = '') -> None:
        st = self._cur()
        self.p.status(state, self.utterance, st.title if st else '', max(self.cur, 0),
                      self.count if self.count >= 0 else len(self.steps), detail)

    # --------------------------------------------------------------- inputs
    def heard(self, plan_id: str, utterance: str) -> str:
        """A final transcript arrived and the node is about to send a PlanRequest.
        Returns the state string to put in the request."""
        state = self.state_for_llm
        self.p.trace(T_HEARD, plan_id, -1, utterance, '')
        self.p.trace(T_THINKING, plan_id, -1, '생각하는 중', state)
        if self.phase == 'PLANNING':                     # latest wins: forget the unanswered one
            self.p.trace(T_CANCEL, self.plan_id, -1, '새 발화로 대체', '')
            self.phase = 'IDLE'
        if self.phase in ('IDLE', 'CONFIRM'):
            self.phase = 'PLANNING'
            self.plan_id = plan_id
            self.utterance = utterance
            self._t = self.p.now()
            self.count = -1
            self.steps = {}
        else:
            # running: keep going; the LLM's answer decides (new plan → preempt, chat → say)
            self._pending = {'plan_id': plan_id, 'utterance': utterance, 'steps': {}, 'count': -1,
                             'speculative': True}
        return state

    def on_step(self, plan_id: str, kind: int, index: int, action: str, args: list,
                say: str, reply_kind: str, detail: str) -> None:
        KIND_SUB, KIND_END, KIND_REPLY, KIND_ERROR = 0, 1, 2, 3
        target = self._route(plan_id)
        if target is None:
            self.p.log(f'ignore step for stale plan {plan_id}')
            return
        if kind == KIND_REPLY:
            self.p.trace(T_REPLY, plan_id, -1, say, reply_kind)
            self.p.say(say)
            if target == 'pending':                      # chat while running → nothing else changes
                self._pending = None
                return
            if reply_kind == 'confirm':
                self.phase = 'CONFIRM'
                self._confirm_summary = self.utterance
            else:
                self.phase = 'IDLE'
                self._status(S_IDLE)
            return
        if kind == KIND_ERROR:
            self.p.trace(T_NOTE, plan_id, -1, '계획을 만들지 못했습니다', detail)
            if target == 'pending':
                self._pending = None
                return
            if self.phase == 'PLANNING':                 # nothing started yet
                self.p.say(planner.phrase(self.cfg, 'plan_error'))
                self._reset()
                self._status(S_FAILED, detail)
            else:                                        # mid-plan: finish current step, then stop
                self._begin_stop('plan_error: ' + detail)
            return
        if kind == KIND_END:
            if target == 'pending':
                self._pending['count'] = index
            else:
                self.count = index
                self.p.trace(T_PLAN_END, plan_id, index, f'계획 {index}단계', '')
                if self._st == 'WAIT_STEP':
                    self._advance()
            return
        # KIND_SUB
        step = Step(index, action, list(args), say, planner.exec_of(self.cfg, action),
                    planner.instruction_for(self.cfg, action, args),
                    planner.step_title(self.cfg, action, args), detail)
        if target == 'pending':
            self._pending['steps'][index] = step
            if self._pending.get('speculative') and index == 0:
                # The LLM decided this is a new command → preempt the running plan
                self._pending['speculative'] = False
                self._begin_stop('preempted by new command', keep_pending=True)
            return
        self.steps[index] = step
        self.p.trace(T_PLAN_LINE, plan_id, index, step.title, detail)
        if self.phase == 'PLANNING' and index == 0:
            self.phase = 'RUNNING'
            self._dispatch(0)
        elif self._st == 'WAIT_STEP' and index == self.cur + 1:
            self._advance()

    def on_state(self, exec: str, status: int, plan_id: str, index: int,
                 detail: str, progress: float) -> None:
        m = self.mod[exec]
        m.status, m.plan_id, m.index, m.detail, m.progress = status, plan_id, index, detail, progress
        m.t_recv = self.p.now()
        st = self._cur()
        if st is None or st.exec != exec:
            return
        mine = (plan_id == self.plan_id and index == st.index)
        if self.phase == 'RUNNING':
            if not mine:
                return
            if status == RUNNING and self._st == 'ACCEPT':
                self._st, self._t = 'RUN', self.p.now()
                self._status(S_RUNNING)
            elif status == DONE and self._st in ('ACCEPT', 'RUN'):
                self.p.trace(T_STEP_DONE, self.plan_id, st.index, st.title, '')
                self._advance()
            elif status == FAILED and self._st in ('ACCEPT', 'RUN'):
                self._fail_step(detail or 'failed')
        elif self.phase == 'STOPPING':
            if status == IDLE:                           # cleanup complete
                self._finish_stop()
            elif mine and status in (DONE, FAILED) and self._stop_deferred:
                # vla reached its safe point; it goes IDLE by itself after 3 reports
                self.p.trace(T_STEP_DONE if status == DONE else T_STEP_FAILED,
                             self.plan_id, st.index, st.title, 'cancelled_at_safe_point')

    def stop(self, reason: str = 'user') -> None:
        """User said stop. Cancels whatever is running; pending plan is dropped."""
        self.p.stop_speech()
        self._pending = None
        if self.phase == 'IDLE':
            return
        if self.phase == 'STOPPING':                     # already stopping; just drop the pending plan
            return
        if self.phase in ('PLANNING', 'CONFIRM'):
            self.p.trace(T_CANCEL, self.plan_id, -1, '중단', reason)
            self._reset()
            self._status(S_PREEMPTED, reason)
            return
        self.p.say(planner.phrase(self.cfg, 'stopped'))
        self._begin_stop(reason)

    def tick(self) -> None:
        now = self.p.now()
        st = self._cur()
        if self.phase == 'PLANNING':
            if now - self._t > self.prm.plan_timeout_s:
                self.p.say(planner.phrase(self.cfg, 'plan_error'))
                self.p.trace(T_CANCEL, self.plan_id, -1, '계획 응답 없음', '')
                self._reset()
                self._status(S_FAILED, 'plan timeout')
            return
        if self.phase == 'RUNNING' and st is not None:
            m = self.mod[st.exec]
            if self._st == 'ACCEPT':
                if now - self._t > self.prm.accept_timeout_s:
                    if self._retries < self.prm.accept_retries:
                        self._retries += 1
                        self._t = now
                        self.p.send_cmd(st.exec, st, self.plan_id)
                        self.p.trace(T_NOTE, self.plan_id, st.index, '명령 재전송', '')
                    else:
                        self._fail_step('not accepted', say_key='not_accepted')
            elif self._st == 'RUN':
                if m.t_recv >= 0 and now - m.t_recv > self.prm.stale_s:
                    self._fail_step('module lost', say_key='module_lost')
                elif now - self._t > self.prm.step_timeout_s.get(st.exec, 30.0):
                    self.p.send_cancel(st.exec, self.plan_id, st.index)
                    self._fail_step('timeout')
            elif self._st == 'WAIT_STEP':
                if now - self._t > self.prm.step_wait_s:
                    self.p.say(planner.phrase(self.cfg, 'plan_error'))
                    self._abandon('plan stalled')
        elif self.phase == 'STOPPING' and st is not None:
            limit = self.prm.safe_stop_timeout_s.get(st.exec, 10.0)
            if self._stop_deferred:
                limit = self.prm.step_timeout_s.get(st.exec, 30.0)
            if now - self._t > limit:
                self.p.trace(T_NOTE, self.plan_id, st.index, '모듈이 멈추지 않음', '')
                self._finish_stop(module_ok=False)

    # -------------------------------------------------------------- internals
    def _route(self, plan_id: str) -> Optional[str]:
        if plan_id == self.plan_id and self.phase != 'IDLE':
            return 'active'
        if self._pending and plan_id == self._pending['plan_id']:
            return 'pending'
        return None

    def _dispatch(self, index: int) -> None:
        st = self.steps[index]
        self.cur = index
        target = planner.precheck_target(self.cfg, st.action, st.args)
        if target:
            found, detail = self.p.check_target(target)
            ko = planner.ko_name(self.cfg, target)
            if found:
                self.p.trace(T_GROUND, self.plan_id, index, f'{ko} 확인됨', detail)
            elif detail and self.prm.detector_fail_open:
                self.p.trace(T_GROUND, self.plan_id, index, f'{ko} 확인 불가 · 진행', detail)
            else:
                self.p.trace(T_GROUND, self.plan_id, index, f'{ko} 보이지 않음', detail)
                self.p.say(planner.phrase(self.cfg, 'not_visible', target))
                self._abandon(f'precheck: {target} not visible')
                return
        self._st, self._t, self._retries = 'ACCEPT', self.p.now(), 0
        self.p.send_cmd(st.exec, st, self.plan_id)
        self.p.say(st.say)
        self.p.trace(T_STEP_START, self.plan_id, index, st.say, '')
        self._status(S_RUNNING)

    def _advance(self) -> None:
        nxt = self.cur + 1
        if nxt in self.steps:
            self._dispatch(nxt)
        elif self.count >= 0 and nxt >= self.count:
            self.p.trace(T_PLAN_DONE, self.plan_id, -1, '완료', '')
            self._status(S_SUCCEEDED)
            self._reset()
            self._status(S_IDLE)
        else:
            self._st, self._t = 'WAIT_STEP', self.p.now()

    def _fail_step(self, reason: str, say_key: str = 'step_failed') -> None:
        st = self._cur()
        self.p.trace(T_STEP_FAILED, self.plan_id, st.index, reason, '')
        subject = {'nav': '이동', 'vla': '조작'}[st.exec] if say_key == 'module_lost' else st.title
        self.p.say(planner.phrase(self.cfg, say_key, subject))
        self._abandon(reason)

    def _abandon(self, reason: str) -> None:
        self.p.trace(T_CANCEL, self.plan_id, -1, '계획 중단', reason)
        self._status(S_FAILED, reason)
        self._reset()
        self._status(S_IDLE)

    @property
    def _stop_deferred(self) -> bool:
        st = self._cur()
        return st is not None and st.exec == 'vla'

    def _begin_stop(self, reason: str, keep_pending: bool = False) -> None:
        if not keep_pending:
            self._pending = None
        st = self._cur()
        self._stop_reason = reason
        if st is None or self._st == 'WAIT_STEP' or self.phase != 'RUNNING':
            self._finish_stop()
            return
        self.phase = 'STOPPING'
        self._t = self.p.now()
        if st.exec == 'nav':
            self.p.send_cancel('nav', self.plan_id, st.index)
            self.p.trace(T_CANCEL, self.plan_id, st.index, '이동 취소', reason)
        else:
            self.p.trace(T_CANCEL, self.plan_id, st.index, '현재 동작 완료 후 중단', reason)
        self._status(S_PREEMPTED, reason)

    def _finish_stop(self, module_ok: bool = True) -> None:
        pending = self._pending
        self._pending = None
        self._reset()
        if pending and pending.get('steps'):
            self.phase = 'RUNNING'
            self.plan_id = pending['plan_id']
            self.utterance = pending['utterance']
            self.steps = pending['steps']
            self.count = pending['count']
            for i in sorted(self.steps):
                s = self.steps[i]
                self.p.trace(T_PLAN_LINE, self.plan_id, i, s.title, s.raw)
            if self.count >= 0:
                self.p.trace(T_PLAN_END, self.plan_id, self.count, f'계획 {self.count}단계', '')
            self._dispatch(0)
        else:
            self._status(S_IDLE, '' if module_ok else 'module did not stop')
