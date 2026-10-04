"""executor — runs a streamed plan against the nav / VLA modules. Pure logic.

No rclpy here: the node feeds events in (PlanStep, SubtaskState, transcript,
tick) and the executor talks back through `Ports` callbacks. That keeps the
state machine unit-testable with a fake clock.

Contract implemented: docs ICD RULE-SUBTASK (from Subtask_state_interface_spec v0.1)
    accept_timeout_s 0.5 (+1 resend) · stale_s 1.0 · step_timeout nav 60 / vla 30
    step_wait_s 5 (next PlanStep) · cancel_ack_s 0.5 · safe_stop nav 3 / vla 10
    idle_wait_s 1 (DONE → IDLE)
    IDLE from a module means "cleanup complete, ready for a new Cmd"

Two rules keep every module handled the same way:
    1. A Cmd goes out only to a module whose last reported status is IDLE.
       After a step, that means waiting for the module's IDLE, not its first
       DONE — nav and VLA both drive the body through gearsonic; IDLE is "I have
       let go", DONE is only "I got there". Before any other dispatch (a new
       plan's first step, a step on the other module) a module still RUNNING
       from an abandoned step is cancelled and its IDLE awaited (WAIT_READY).
       Modules therefore never see a Cmd while busy: there is no preemption by
       Cmd — a busy module may ignore one.
    2. Every stop (user "stop", a new plan replacing the running one, step
       timeout) sends a cancel and waits for IDLE. Whether to defer to a safe
       point is the module's call (appendix A: cancel_deferred /
       cancelled_at_safe_point). An unanswered cancel is resent once after
       cancel_ack_s. If the module does not reach IDLE in time, no new plan is
       started.
    The one graceful stop is a plan error mid-plan: the running step finishes
    (bounded by its step timeout, then cancelled) and nothing after it starts.

Phases
    IDLE       nothing running
    PLANNING   PlanRequest sent, waiting for the first PlanStep (or a reply)
    RUNNING    checking or executing a step (sub-state in _st:
               WAIT_READY | PRECHECK | ACCEPT | RUN | WAIT_IDLE | WAIT_STEP)
    STOPPING   cancel sent (or the step is allowed to finish); waiting for the module's IDLE
    CONFIRM    a confirm question is pending — next utterance answers it
"""

from __future__ import annotations

import math
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
    idle_wait_s: float = 1.0              # DONE → IDLE (DONE is repeated 3x at 10 Hz, ~0.3 s)
    cancel_ack_s: float = 0.5
    safe_stop_timeout_s: dict = field(default_factory=lambda: {'nav': 3.0, 'vla': 10.0})
    plan_timeout_s: float = 20.0          # PlanRequest → first line
    precheck_timeout_s: float = 0.0      # 0 preserves one-shot demo behavior
    precheck_retry_s: float = 0.1        # rate-limited by the executor tick
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
    def __init__(self, cfg: dict, ports: Ports, params: Params | None = None,
                 rewriter=None) -> None:
        self.cfg = cfg
        self.p = ports
        self.prm = params or Params()
        self.rw = rewriter                      # rewrite.Rewriter — demo-only inserts, or None
        if (not math.isfinite(self.prm.precheck_timeout_s) or self.prm.precheck_timeout_s < 0
                or not math.isfinite(self.prm.precheck_retry_s) or self.prm.precheck_retry_s <= 0):
            raise ValueError('invalid precheck retry settings')
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
        self._st = ''                   # WAIT_READY | PRECHECK | ACCEPT | RUN | WAIT_IDLE | WAIT_STEP
        self._t = 0.0                   # timestamp of the current sub-state entry
        self._retries = 0
        self._precheck_next = 0.0
        self._precheck_detail = ''
        self._precheck_target = ''
        self._busy_seen = False         # module reported the current step as non-IDLE
        self._final_traced = False      # STEP_DONE / STEP_FAILED already shown for the current step
        self._stop_mode = ''            # STOPPING: 'cancel' (cancel sent) | 'finish' (let the step end)
        self._cancel_acked = False      # STOPPING: module answered the cancel (IDLE or cancel_deferred)
        self._cancel_resent = False     # STOPPING: the one cancel resend is spent
        self._pending: Optional[dict] = None   # plan waiting to start after a stop
        self._stop_reason = ''
        self._confirm_summary = ''
        self._offset = 0                # steps inserted by rewrites so far (LLM index → executor index)

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
                             'offset': 0, 'speculative': True}
        return state

    def _error_say(self, detail: str) -> str:
        """KIND_ERROR 의 detail 을 사용자에게 들려줄 한 문장으로.

        검증기 2 가 막았으면 llm_node 가 "<phrases 키>|<대상>" 으로 보낸다
        (unknown_place|kitchen → "그곳은 아직 갈 수 없습니다."). 그 밖에는 일반 문구.
        """
        key, _, target = detail.partition('|')
        if key in self.cfg.get('phrases', {}):
            return planner.phrase(self.cfg, key, target)
        return planner.phrase(self.cfg, 'plan_error')

    def on_step(self, plan_id: str, kind: int, index: int, action: str, args: list,
                say: str, reply_kind: str, detail: str) -> None:
        KIND_SUB, KIND_END, KIND_REPLY, KIND_ERROR = 0, 1, 2, 3
        target = self._route(plan_id)
        if target is None:
            # reply(chat/none) 로 끝난 계획은 그 자리에서 IDLE 이 되므로, 뒤따라 오는 END 줄은
            # 늘 여기로 온다. 정상이라 굳이 남기지 않는다.
            if kind != KIND_END:
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
            say_text = self._error_say(detail)
            self.p.trace(T_NOTE, plan_id, -1, say_text, detail)
            if target == 'pending':
                self._pending = None
                return
            if self.phase == 'PLANNING':                 # nothing started yet
                self.p.say(say_text)
                self._reset()
                self._status(S_FAILED, detail)
            else:                                        # mid-plan: finish current step, then stop
                self._begin_stop('plan_error: ' + detail, cancel=False)
            return
        if kind == KIND_END:
            if target == 'pending':
                self._pending['count'] = index + self._pending['offset']
            else:
                self.count = index + self._offset
                self.p.trace(T_PLAN_END, plan_id, self.count, f'계획 {self.count}단계', '')
                if self._st == 'WAIT_STEP':
                    self._advance()
            return
        # KIND_SUB
        step = Step(index, action, list(args), say, planner.exec_of(self.cfg, action),
                    planner.instruction_for(self.cfg, action, args),
                    planner.step_title(self.cfg, action, args), detail)
        if target == 'pending':
            self._pending['offset'], _ = self._place(self._pending['steps'],
                                                     self._pending['offset'], step)
            if self._pending.get('speculative') and index == 0:
                # The LLM decided this is a new command → preempt the running plan
                self._pending['speculative'] = False
                self._begin_stop('preempted by new command', keep_pending=True)
            return
        self._offset, added = self._place(self.steps, self._offset, step)
        for s in added:
            self.p.trace(T_PLAN_LINE, plan_id, s.index, s.title, s.raw)
        if self.phase == 'PLANNING' and index == 0:
            self.phase = 'RUNNING'
            self._dispatch(0)
        elif self._st == 'WAIT_STEP' and added[0].index == self.cur + 1:
            self._advance()

    def _place(self, steps: dict, offset: int, step: Step) -> tuple:
        """An LLM line → its executor index, with any demo rewrite inserted in front of it.

        Rewrites (rewrite.py) look at the pair (previous line, this line). Inserted steps
        shift everything after them, so the executor numbers steps itself from here on:
        executor index = LLM index + steps inserted so far. Returns (offset, added steps)."""
        step.index += offset
        added = []
        prev = steps.get(step.index - 1)
        if prev is not None and self.rw is not None:
            for ins in self.rw.between(prev.action, prev.args, step.action, step.args):
                s = Step(step.index, ins.action, ins.args, '', ins.exec, ins.instruction,
                         ins.title, f'rewrite:{ins.rule}')
                steps[s.index] = s
                added.append(s)
                self.p.log(f'rewrite {ins.rule}: {ins.action}{ins.args} before {step.action}{step.args}')
                step.index += 1
                offset += 1
        steps[step.index] = step
        added.append(step)
        return offset, added

    def on_state(self, exec: str, status: int, plan_id: str, index: int,
                 detail: str, progress: float) -> None:
        m = self.mod[exec]
        m.status, m.plan_id, m.index, m.detail, m.progress = status, plan_id, index, detail, progress
        m.t_recv = self.p.now()
        st = self._cur()
        if st is None or st.exec != exec:
            return
        mine = (plan_id == self.plan_id and index == st.index)
        if mine and status != IDLE:
            self._busy_seen = True
        if self.phase == 'RUNNING':
            if self._st == 'WAIT_IDLE':
                if status == IDLE:                       # rule 1: the module let go → next step
                    self._advance()
                return
            if self._st == 'WAIT_READY':
                if status == IDLE:                       # rule 1: free → check current evidence
                    self._start_precheck(self.cur)
                return
            if not mine:
                return
            if status == RUNNING and self._st == 'ACCEPT':
                self._st, self._t = 'RUN', self.p.now()
                self._status(S_RUNNING)
            elif status == DONE and self._st in ('ACCEPT', 'RUN'):
                self.p.trace(T_STEP_DONE, self.plan_id, st.index, st.title, '')
                self._final_traced = True
                self._st, self._t = 'WAIT_IDLE', self.p.now()
            elif status == FAILED and self._st in ('ACCEPT', 'RUN'):
                self._fail_step(detail or 'failed')
        elif self.phase == 'STOPPING':
            if mine and status == RUNNING and detail.startswith('cancel_deferred'):
                self._cancel_acked = True
            if status == IDLE and self._module_settled():
                self._finish_stop()
            elif mine and status in (DONE, FAILED) and not self._final_traced:
                # the step ended before the IDLE: at its safe point, or on its own
                # (graceful stop). Shown once — DONE / FAILED repeat 3x.
                self.p.trace(T_STEP_DONE if status == DONE else T_STEP_FAILED,
                             self.plan_id, st.index, st.title, detail)
                self._final_traced = True

    def stop(self, reason: str = 'user') -> None:
        """User said stop. Cancels whatever is running; pending plan is dropped."""
        self.p.stop_speech()
        self._pending = None
        if self.phase == 'IDLE':
            return
        if self.phase == 'STOPPING':                     # already stopping; drop the pending plan
            st = self._cur()
            if self._stop_mode == 'finish' and st is not None:
                # the step was being allowed to finish — "stop" means now
                self.p.say(planner.phrase(self.cfg, 'stopped'))
                self._send_cancel(st, reason)
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
            if self._st == 'WAIT_READY':
                if now - self._t > self.prm.safe_stop_timeout_s.get(st.exec, 10.0):
                    # rule 1/2: the module is still busy — put nothing on top of it
                    self.p.trace(T_NOTE, self.plan_id, st.index, '모듈이 비지 않음',
                                 STATUS_NAME.get(m.status, str(m.status)))
                    self.p.say(planner.phrase(self.cfg, 'stop_failed'))
                    self._abandon('module not idle')
            elif self._st == 'PRECHECK':
                if now - self._t >= self.prm.precheck_timeout_s:
                    self._precheck_failed(self._precheck_target,
                                          self._precheck_detail or 'not_visible', timeout=True)
                elif now >= self._precheck_next:
                    self._check_precondition()
            elif self._st == 'ACCEPT':
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
                    self._fail_step('timeout', cancel=True)
            elif self._st == 'WAIT_IDLE':
                if now - self._t > self.prm.idle_wait_s:
                    self._fail_step('did not go idle', say_key='module_lost')
            elif self._st == 'WAIT_STEP':
                if now - self._t > self.prm.step_wait_s:
                    self.p.say(planner.phrase(self.cfg, 'plan_error'))
                    self._abandon('plan stalled')
        elif self.phase == 'STOPPING' and st is not None:
            if self._stop_mode == 'finish':
                # graceful stop, but not forever: past the step timeout, cancel it
                if now - self._t > self.prm.step_timeout_s.get(st.exec, 30.0):
                    self._send_cancel(st, 'timeout')
                return
            if (not self._cancel_acked and not self._cancel_resent
                    and now - self._t > self.prm.cancel_ack_s):
                # no IDLE and no cancel_deferred yet: the cancel may be lost — resend
                # once. A Cmd no longer preempts, so the cancel is the only way to stop.
                self._cancel_resent = True
                self.p.send_cancel(st.exec, self.plan_id, st.index)
                self.p.trace(T_NOTE, self.plan_id, st.index, '취소 재전송', '')
            if now - self._t > self.prm.safe_stop_timeout_s.get(st.exec, 10.0):
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
        """Rule 1: send only to a module whose last reported status is IDLE.

        Otherwise wait for its IDLE (WAIT_READY). A module still RUNNING here is a
        leftover from an abandoned step (not accepted in time, lost, …) — cancel it
        by the id it reports. DONE / FAILED just finish their 3x burst."""
        st = self.steps[index]
        self.cur = index
        m = self.mod[st.exec]
        if m.status != IDLE:
            self._st, self._t = 'WAIT_READY', self.p.now()
            if m.status == RUNNING and m.plan_id:
                self.p.send_cancel(st.exec, m.plan_id, m.index)
                self.p.trace(T_CANCEL, m.plan_id, m.index, '이전 동작 취소', 'module busy')
            self.p.log(f'{st.exec} is {STATUS_NAME.get(m.status, m.status)} — waiting for IDLE '
                       f'before step {index}')
            return
        self._start_precheck(index)

    def _start_precheck(self, index: int) -> None:
        self.cur = index
        self._st, self._t = 'PRECHECK', self.p.now()
        self._precheck_detail = ''
        self._precheck_target = ''
        self._check_precondition()

    def _check_precondition(self) -> None:
        st = self._cur()
        # A module can become busy while evidence is pending. Discard prior
        # evidence and restart the precheck only after it reports IDLE again.
        if self.mod[st.exec].status != IDLE:
            self._dispatch(self.cur)
            return
        try:
            targets = planner.precheck_targets(self.cfg, st.action, st.args)
        except ValueError as error:
            self.p.say(planner.phrase(self.cfg, 'plan_error'))
            self._abandon(f'invalid_precheck: {error}')
            return
        # Each attempt queries every target again. Never latch an earlier hit
        # across retries: fridge-only then cucumber-only must not authorize.
        results = []
        for target in targets:
            found, detail = self.p.check_target(target)
            now = self.p.now()
            # All queries share the original step deadline, including slow calls.
            if self.prm.precheck_timeout_s > 0 and now - self._t >= self.prm.precheck_timeout_s:
                self._precheck_failed(target, 'detector_timeout' if found else detail or 'not_visible',
                                      timeout=True)
                return
            results.append((target, found, detail))
        blocked = [(t, d or 'not_visible') for t, found, d in results
                   if not found and not (d and self.prm.detector_fail_open)]
        if blocked:
            terminal_reasons = ('unsupported_class', 'unknown_target', 'invalid_request')
            target, reason = next(((t, d) for t, d in blocked if d in terminal_reasons), blocked[0])
            if self.prm.precheck_timeout_s > 0 and reason not in terminal_reasons:
                if (target, reason) != (self._precheck_target, self._precheck_detail):
                    ko = planner.ko_name(self.cfg, target)
                    self.p.trace(T_GROUND, self.plan_id, st.index, f'{ko} 확인 대기', reason)
                    description = f'{target}: {reason}' if len(targets) > 1 else reason
                    self._status(S_RUNNING, 'precheck_wait: ' + description)
                self._precheck_target, self._precheck_detail = target, reason
                self._precheck_next = self.p.now() + self.prm.precheck_retry_s
                return
            self._precheck_failed(target, reason)
            return
        if self.mod[st.exec].status != IDLE:
            self._dispatch(self.cur)
            return
        for target, found, detail in results:
            ko = planner.ko_name(self.cfg, target)
            title = f'{ko} 확인됨' if found else f'{ko} 확인 불가 · 진행'
            self.p.trace(T_GROUND, self.plan_id, st.index, title, detail)
        self._st, self._t, self._retries = 'ACCEPT', self.p.now(), 0
        self._busy_seen = self._final_traced = False
        self.p.send_cmd(st.exec, st, self.plan_id)
        if st.say:
            self.p.say(st.say)
        self.p.trace(T_STEP_START, self.plan_id, st.index, st.say or st.title, '')
        self._status(S_RUNNING)

    def _precheck_failed(self, target: str, detail: str, timeout: bool = False) -> None:
        ko = planner.ko_name(self.cfg, target)
        title = f'{ko} 확인 시간 초과' if timeout else f'{ko} 확인 실패'
        self.p.trace(T_GROUND, self.plan_id, self.cur, title, detail)
        key = 'not_visible' if detail == 'not_visible' else 'precheck_unavailable'
        self.p.say(planner.phrase(self.cfg, key, target))
        reason = 'precheck_timeout' if timeout else 'precheck'
        self._abandon(f'{reason}: {target}: {detail}')

    def _advance(self) -> None:
        nxt = self.cur + 1
        if nxt in self.steps:
            self._dispatch(nxt)
        elif self.count >= 0 and nxt >= self.count:
            self.p.trace(T_PLAN_DONE, self.plan_id, -1, '완료', '')
            self._status(S_SUCCEEDED)
            # 실행 중에 들어온 다음 발화가 대기하고 있으면 이어서 시작한다. _reset() 이
            # _pending 을 지우므로 _finish_stop() 에 넘겨 승격시킨다 — 그렇게 하지 않으면
            # "마지막 동작이 끝나기 직전에 말한 문장" 이 조용히 사라진다.
            self._finish_stop()
        else:
            self._st, self._t = 'WAIT_STEP', self.p.now()

    def _fail_step(self, reason: str, say_key: str = 'step_failed', cancel: bool = False) -> None:
        st = self._cur()
        self.p.trace(T_STEP_FAILED, self.plan_id, st.index, reason, '')
        self._final_traced = True
        subject = {'nav': '이동', 'vla': '조작'}[st.exec] if say_key == 'module_lost' else st.title
        self.p.say(planner.phrase(self.cfg, say_key, subject))
        if cancel:
            # rule 2: the module is still acting — cancel it and see it reach IDLE
            # before anything else runs (a pending utterance is dropped with the plan)
            self.p.trace(T_CANCEL, self.plan_id, -1, '계획 중단', reason)
            self._begin_stop(reason, state=S_FAILED, trace=False)
        else:
            self._abandon(reason)

    def _abandon(self, reason: str) -> None:
        self.p.trace(T_CANCEL, self.plan_id, -1, '계획 중단', reason)
        self._status(S_FAILED, reason)
        self._reset()
        self._status(S_IDLE)

    def _module_settled(self) -> bool:
        """Is an IDLE seen now really the stop's result?

        If the module never reported the current step (the Cmd may still be in
        flight), an IDLE could predate it. Trust it only after cancel_ack_s."""
        return self._busy_seen or self.p.now() - self._t >= self.prm.cancel_ack_s

    def _send_cancel(self, st: Step, reason: str, trace: bool = True) -> None:
        self._stop_mode, self._t = 'cancel', self.p.now()
        self.p.send_cancel(st.exec, self.plan_id, st.index)
        if trace:
            title = '이동 취소' if st.exec == 'nav' else '동작 취소'
            self.p.trace(T_CANCEL, self.plan_id, st.index, title, reason)

    def _begin_stop(self, reason: str, keep_pending: bool = False, cancel: bool = True,
                    state: int = S_PREEMPTED, trace: bool = True) -> None:
        if not keep_pending:
            self._pending = None
        st = self._cur()
        self._stop_reason = reason
        if self.phase == 'RUNNING' and self._st == 'PRECHECK':
            if trace:
                self.p.trace(T_CANCEL, self.plan_id, self.cur, '확인 대기 중단', reason)
            self._status(state, reason)
            self._finish_stop()  # No module command has been sent; no cancel/DONE wait.
            return
        between_steps = self._st in ('WAIT_STEP', 'WAIT_IDLE', 'WAIT_READY')
        if trace and between_steps and self.phase == 'RUNNING':
            # nothing to cancel, but the screen still has to show the stop
            self.p.trace(T_CANCEL, self.plan_id, -1, '다음 단계 전에 중단', reason)
        if st is None or self._st in ('WAIT_STEP', 'WAIT_READY') or self.phase != 'RUNNING':
            # nothing of ours is moving (WAIT_READY: our Cmd was never sent; a leftover
            # was already cancelled and the next dispatch waits for its IDLE again)
            self._finish_stop()                          # nothing is moving
            return
        self.phase = 'STOPPING'
        self._t = self.p.now()
        if self._st == 'WAIT_IDLE':                      # step already over; only its IDLE is missing
            self._stop_mode = 'cancel'
            self._cancel_acked = True                    # no cancel was sent, so none to resend
        elif cancel:
            self._send_cancel(st, reason, trace)
        else:
            self._stop_mode = 'finish'
            if trace:
                self.p.trace(T_CANCEL, self.plan_id, st.index, '현재 동작 완료 후 중단', reason)
        self._status(state, reason)

    def _finish_stop(self, module_ok: bool = True) -> None:
        pending = self._pending
        self._pending = None
        self._reset()
        if not module_ok:
            # rule 2: the module may still be moving — start nothing new on top of it
            self.p.say(planner.phrase(self.cfg, 'stop_failed'))
            self._status(S_FAILED, 'module did not stop')
            self._status(S_IDLE)
            return
        if pending and pending.get('steps'):
            self.phase = 'RUNNING'
            self.plan_id = pending['plan_id']
            self.utterance = pending['utterance']
            self.steps = pending['steps']
            self.count = pending['count']
            self._offset = pending['offset']
            for i in sorted(self.steps):
                s = self.steps[i]
                self.p.trace(T_PLAN_LINE, self.plan_id, i, s.title, s.raw)
            if self.count >= 0:
                self.p.trace(T_PLAN_END, self.plan_id, self.count, f'계획 {self.count}단계', '')
            self._dispatch(0)
        elif pending:
            # 계획이 아직 안 왔을 뿐, 사용자는 이미 말했다. 그 plan_id 로 계속 기다린다.
            # 여기서 버리면 "앞 동작이 끝나기 직전에 말한 문장" 이 통째로 사라진다.
            self.phase = 'PLANNING'
            self.plan_id = pending['plan_id']
            self.utterance = pending['utterance']
            self.count = pending['count']
            self._t = self.p.now()
        else:
            self._status(S_IDLE)
