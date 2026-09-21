"""orchestrator_node — hook-driven scenario orchestrator (TaskSrv form, rclpy).

No LLM, no router: a scenario declares its own trigger keywords and an STT
transcript is matched against them. A scenario is a list of sub-tasks, each a
small lifecycle machine:

    on_create → on_start → (poll `success` each tick) → on_success | on_fail

A hook step is {label: payload}; the label picks a Connector (speak / navigation
/ vla). `success` is a polymorphic Criterion (always / delay / voice_keyword /
composite). Scenarios are JSON5 under config/scenarios/. The former `vlm`
criterion (scene judgment by vlm_node) was removed with vlm_node; scene
grounding now happens in llm mode via detector_node before each step.

planner_mode (parameter):
    static  the engine above — transcripts match scenario triggers.
    llm     every final transcript becomes a PlanRequest to llm_node; the streamed
            PlanStep lines are executed by cortex_cognition.executor against the
            nav / VLA modules over SubtaskCmd / SubtaskState (spec v0.1), with a
            detector precheck (CheckTarget) before manipulation steps. The static
            engine is bypassed entirely in this mode.

OPEN-LOOP (static mode): connectors fire commands (ActionCmd for speak, stubs for
nav/vla) and never wait for confirmation. Preemption is immediate — a new trigger cancels the
current commands (fire-and-forget) and starts the new scenario right away, with no
CANCELING wait or status monitoring. The old and new motions may briefly overlap;
that trade-off is accepted for simplicity.
"""

import glob
import os
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import json5
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from cortex_msgs.msg import (ActionCmd, PlanRequest, PlanStep, SubtaskCmd, SubtaskState,
                             TaskStatus, TraceEvent)
from cortex_msgs.srv import CheckTarget
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy

from . import executor as ex
from . import planner


# ===========================================================================
# Scenario model + loader
# ===========================================================================
@dataclass
class SubTaskDef:
    name: str
    on_create: list = field(default_factory=list)   # [{label: payload}, ...]
    on_start: list = field(default_factory=list)
    on_success: list = field(default_factory=list)
    on_fail: list = field(default_factory=list)     # fired on timeout; owns the message
    success: dict = field(default_factory=dict)     # raw spec
    criterion: 'Criterion' = None                   # built at LOAD -> fail fast
    timeout_s: float = 30.0


@dataclass
class Scenario:
    name: str
    triggers: list           # keyword substrings matched against transcripts
    sub_tasks: list          # [SubTaskDef]. Empty == a pure "stop": preempt, then idle.


def _load_subtask(s: dict) -> SubTaskDef:
    spec = s.get('success', {})
    return SubTaskDef(
        name=s['name'],
        on_create=s.get('on_create', []),
        on_start=s.get('on_start', []),
        on_success=s.get('on_success', []),
        on_fail=s.get('on_fail', []),
        success=spec,
        criterion=build_criterion(spec),            # raises ScenarioConfigError here
        timeout_s=float(spec.get('timeout_s', 30.0)),
    )


def load_scenarios(scenario_dir: str) -> list:
    """Load every *.json5 under scenario_dir into Scenario objects.

    Criteria are built here, so a bad scenario stops the node at startup with the
    offending file named — rather than loading fine and then never passing.
    """
    scenarios = []
    for path in sorted(glob.glob(os.path.join(scenario_dir, '*.json5'))):
        with open(path, 'r', encoding='utf-8') as f:
            raw = json5.load(f)
        try:
            subs = [_load_subtask(s) for s in raw.get('sub_tasks', [])]
            scenarios.append(Scenario(raw['name'], raw.get('triggers', []), subs))
        except (ScenarioConfigError, KeyError, TypeError, ValueError) as exc:
            raise ScenarioConfigError(f'{path}: {exc}') from exc
    return scenarios


# ===========================================================================
# Criteria — polymorphic success rules
# ===========================================================================
class ScenarioConfigError(ValueError):
    """Bad scenario. Raised at LOAD, not run — a typo must fail fast, not degrade
    into a sub-task that never passes and 'fails' on timeout 15 s later."""


class Criterion(ABC):
    @abstractmethod
    def evaluate(self, node: 'OrchestratorNode', subtask_id: str) -> bool: ...


@dataclass
class DelayCriterion(Criterion):
    """Pass ``seconds`` after on_start (not entry — see _tick). Placeholder that
    asserts nothing about the world; replace with a real arrival signal once wired."""

    seconds: float = 0.0

    def evaluate(self, node, subtask_id) -> bool:
        return node.elapsed() >= self.seconds


@dataclass
class VoiceKeywordCriterion(Criterion):
    """Pass on an utterance heard SINCE THIS SUB-TASK STARTED. For context replies
    ("응" / "오리엔탈로"); an independent command belongs in `triggers`, not here."""

    keywords: list = field(default_factory=list)

    def evaluate(self, node, subtask_id) -> bool:
        return any(kw in t for t in node.transcripts for kw in self.keywords)


@dataclass
class CompositeCriterion(Criterion):
    """All-of (AND). Worth it only when children catch different failure modes and
    a false positive is costlier — P(all) is a product, so AND lowers the pass
    rate and adds false negatives. (grasp: joint AND scene; arrival: uwb alone.)"""

    children: list = field(default_factory=list)

    def evaluate(self, node, subtask_id) -> bool:
        return all(c.evaluate(node, subtask_id) for c in self.children)


class AlwaysCriterion(Criterion):
    """Pass immediately. For smoke-testing the dispatch path."""

    def evaluate(self, node, subtask_id) -> bool:
        return True


def _req(spec: dict, key: str, tag: str):
    if key not in spec:
        raise ScenarioConfigError(f'criterion {tag!r} requires {key!r}')
    return spec[key]


_CRITERION_BUILDERS = {
    'always': lambda s: AlwaysCriterion(),
    'delay': lambda s: DelayCriterion(seconds=float(_req(s, 'seconds', 'delay'))),
    'voice_keyword': lambda s: VoiceKeywordCriterion(
        keywords=list(_req(s, 'keywords', 'voice_keyword'))),
    'composite': lambda s: CompositeCriterion(
        children=[build_criterion(c) for c in _req(s, 'children', 'composite')]),
}

# Known in the workstation but not ported — named so they fail at load with a
# reason instead of looking like a typo (or worse, silently never passing).
_NOT_PORTED = {
    'uwb_pose': 'needs an onboard pose subscription (not wired)',
    'joint_state': 'needs an onboard joint_states subscription (not wired)',
    'voice_choice': 'needs the scenario blackboard (not ported)',
}


def build_criterion(spec: dict) -> Criterion:
    tag = spec.get('type')   # required — no implicit default
    if tag is None:
        raise ScenarioConfigError(
            f'success.type is required; known: {sorted(_CRITERION_BUILDERS)}')
    builder = _CRITERION_BUILDERS.get(tag)
    if builder is None:
        if tag in _NOT_PORTED:
            raise ScenarioConfigError(
                f'criterion {tag!r} is not available yet: {_NOT_PORTED[tag]}')
        raise ScenarioConfigError(
            f'unknown criterion type {tag!r}; known: {sorted(_CRITERION_BUILDERS)}')
    return builder(spec)


# ===========================================================================
# Connectors — capability-separated dispatch channels (label -> connector)
# ===========================================================================
class Connector(ABC):
    # Open-loop: dispatch fires a command, cancel fires a stop. Neither waits for
    # confirmation. Both abstract — cancel is safety-relevant, no silent no-op.
    @abstractmethod
    def dispatch(self, node: 'OrchestratorNode', payload) -> None: ...

    @abstractmethod
    def cancel(self, node: 'OrchestratorNode') -> None: ...


class SpeakConnector(Connector):
    """`speak` -> tts_node (ActionCmd)."""

    def dispatch(self, node, payload) -> None:
        node.say_pub.publish(ActionCmd(text=str(payload)))

    def cancel(self, node) -> None:
        # Barge-in (fire-and-forget). Only cuts PC-side synthesis; audio already
        # published to the speaker is not recalled.
        node.stop_pub.publish(Bool(data=True))


class NavigationConnector(Connector):
    """`navigation` -> LocoCommand / named goal -> Gearsonic Handler (stub)."""

    def dispatch(self, node, payload) -> None:
        node.get_logger().info(f'(stub) navigation dispatch: {payload!r}')

    def cancel(self, node) -> None:
        node.get_logger().info('(stub) navigation cancel')


class VlaConnector(Connector):
    """`vla` -> arm/hand joint inference -> Gearsonic Handler (stub)."""

    def dispatch(self, node, payload) -> None:
        node.get_logger().info(f'(stub) vla dispatch: {payload!r}')

    def cancel(self, node) -> None:
        node.get_logger().info('(stub) vla cancel')


# ===========================================================================
# Node
# ===========================================================================
class OrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__('orchestrator_node')

        default_dir = os.path.join(
            get_package_share_directory('cortex_cognition'), 'scenarios')
        self.declare_parameter('scenario_dir', default_dir)
        self.declare_parameter('transcript_topic', '/cortex/stt/transcript')
        self.declare_parameter('status_topic', '/cortex/task_status')
        self.declare_parameter('say_topic', '/cortex/tts/say')
        self.declare_parameter('stop_topic', '/cortex/tts/stop')   # tts barge-in
        self.declare_parameter('tick_rate_hz', 10.0)
        # --- llm mode ---
        self.declare_parameter('planner_mode', 'static')          # static | llm
        self.declare_parameter('actions_path', os.path.join(
            get_package_share_directory('cortex_cognition'), 'config', 'actions.yaml'))
        self.declare_parameter('llm_request_topic', '/cortex/llm/request')
        self.declare_parameter('llm_step_topic', '/cortex/llm/step')
        self.declare_parameter('nav_cmd_topic', '/cortex/nav/cmd')
        self.declare_parameter('nav_state_topic', '/cortex/nav/state')
        self.declare_parameter('vla_cmd_topic', '/cortex/vla/cmd')
        self.declare_parameter('vla_state_topic', '/cortex/vla/state')
        self.declare_parameter('trace_topic', '/cortex/trace')
        self.declare_parameter('detector_service', '/cortex/detector/check')
        self.declare_parameter('detector_call_timeout_s', 0.3)
        self.declare_parameter('stop_keywords', ['그만', '멈춰', '정지', '스톱'])
        self.declare_parameter('accept_timeout_s', 0.5)
        self.declare_parameter('stale_s', 1.0)
        self.declare_parameter('step_timeout_nav_s', 60.0)
        self.declare_parameter('step_timeout_vla_s', 30.0)
        self.declare_parameter('step_wait_s', 5.0)
        self.declare_parameter('safe_stop_nav_s', 3.0)
        self.declare_parameter('safe_stop_vla_s', 10.0)
        self.declare_parameter('plan_timeout_s', 20.0)
        self.declare_parameter('detector_fail_open', True)

        g = self.get_parameter
        scenario_dir = g('scenario_dir').value
        tick_hz = float(g('tick_rate_hz').value)
        self.mode = g('planner_mode').value

        # --- connectors (label -> connector) ------------------------------
        self.connectors = {
            'speak': SpeakConnector(),
            'navigation': NavigationConnector(),
            'vla': VlaConnector(),
        }

        # --- scenarios ----------------------------------------------------
        self.scenarios = load_scenarios(scenario_dir)
        self.get_logger().info(
            f'loaded {len(self.scenarios)} scenario(s) from {scenario_dir}')

        # --- run state (mutated only inside the mutually-exclusive callback
        #     group below, so never by two threads at once — see grp) --------
        self._active = None              # active Scenario
        self._index = 0                  # current sub-task index
        self._criterion = None           # current sub-task's Criterion
        self._started = False            # on_start fired for current sub-task?
        self._t0 = 0.0                   # current sub-task start time
        # Utterances heard since the current sub-task started (voice_keyword).
        self._transcripts: list = []

        # --- io -----------------------------------------------------------
        # One mutually-exclusive group for transcript / tick so state
        # (_active, _index, ...) has a single writer at a time.
        grp = MutuallyExclusiveCallbackGroup()
        self.status_pub = self.create_publisher(TaskStatus, g('status_topic').value, 10)
        self.say_pub = self.create_publisher(ActionCmd, g('say_topic').value, 10)
        self.stop_pub = self.create_publisher(Bool, g('stop_topic').value, 10)   # tts barge-in

        self.create_subscription(
            String, g('transcript_topic').value, self._on_transcript, 10, callback_group=grp)

        self.create_timer(1.0 / tick_hz, self._tick, callback_group=grp)

        # --- llm mode wiring ------------------------------------------------
        self.trace_pub = self.create_publisher(TraceEvent, g('trace_topic').value, 50)
        self._exec = None
        if self.mode == 'llm':
            self._setup_llm_mode(grp)
        self.get_logger().info(f'orchestrator_node up (mode={self.mode}, tick={tick_hz}Hz)')

    # ======================================================================
    # llm mode
    # ======================================================================
    def _setup_llm_mode(self, grp) -> None:
        g = self.get_parameter
        self.cfg = planner.load_config(g('actions_path').value)
        self._seq = 0
        self._epoch = int(time.time())
        self.stop_keywords = list(g('stop_keywords').value)
        self.plan_req_pub = self.create_publisher(PlanRequest, g('llm_request_topic').value, 10)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = {
            'nav': self.create_publisher(SubtaskCmd, g('nav_cmd_topic').value, reliable),
            'vla': self.create_publisher(SubtaskCmd, g('vla_cmd_topic').value, reliable),
        }
        self.create_subscription(PlanStep, g('llm_step_topic').value, self._on_plan_step, 50,
                                 callback_group=grp)
        self.create_subscription(SubtaskState, g('nav_state_topic').value,
                                 lambda m: self._on_module_state('nav', m), reliable,
                                 callback_group=grp)
        self.create_subscription(SubtaskState, g('vla_state_topic').value,
                                 lambda m: self._on_module_state('vla', m), reliable,
                                 callback_group=grp)
        # Detector client lives in its own reentrant group so a synchronous wait
        # inside the (mutually exclusive) orchestration callbacks can complete.
        self._det_grp = ReentrantCallbackGroup()
        self.det_client = self.create_client(CheckTarget, g('detector_service').value,
                                             callback_group=self._det_grp)
        self._det_timeout = float(g('detector_call_timeout_s').value)

        prm = ex.Params(
            accept_timeout_s=float(g('accept_timeout_s').value),
            stale_s=float(g('stale_s').value),
            step_timeout_s={'nav': float(g('step_timeout_nav_s').value),
                            'vla': float(g('step_timeout_vla_s').value)},
            step_wait_s=float(g('step_wait_s').value),
            safe_stop_timeout_s={'nav': float(g('safe_stop_nav_s').value),
                                 'vla': float(g('safe_stop_vla_s').value)},
            plan_timeout_s=float(g('plan_timeout_s').value),
            detector_fail_open=bool(g('detector_fail_open').value),
        )
        ports = ex.Ports(
            now=self._now,
            send_cmd=self._send_cmd,
            send_cancel=self._send_cancel,
            check_target=self._check_target,
            say=self._say,
            stop_speech=lambda: self.stop_pub.publish(Bool(data=True)),
            trace=self._trace,
            status=self._status_llm,
            log=lambda s: self.get_logger().info(s),
        )
        self._exec = ex.Executor(self.cfg, ports, prm)
        # 100 ms absence monitor (accept / stale / step timeouts); everything else is event-driven.
        self.create_timer(0.1, self._exec.tick, callback_group=grp)
        self.get_logger().info(
            f'llm mode: {len([a for a, s in self.cfg["actions"].items() if s.get("enabled")])} '
            f'actions, detector={g("detector_service").value}')

    def _new_plan_id(self) -> str:
        self._seq += 1
        return f'p-{self._epoch}-{self._seq:04d}'

    def _on_transcript_llm(self, text: str) -> None:
        if any(k in text for k in self.stop_keywords):
            self._trace(ex.T_HEARD, self._exec.plan_id, -1, text, 'stop')
            self._exec.stop('user')
            return
        plan_id = self._new_plan_id()
        state = self._exec.heard(plan_id, text)
        self.plan_req_pub.publish(PlanRequest(plan_id=plan_id, utterance=text, state=state))

    def _on_plan_step(self, m: PlanStep) -> None:
        self._exec.on_step(m.plan_id, m.kind, m.index, m.action, list(m.args), m.say,
                           m.reply_kind, m.detail)

    def _on_module_state(self, which: str, m: SubtaskState) -> None:
        self._exec.on_state(which, m.status, m.plan_id, m.index, m.detail, m.progress)

    # --- ports --------------------------------------------------------------
    def _say(self, text: str) -> None:
        if text:
            self.say_pub.publish(ActionCmd(text=text))

    def _send_cmd(self, which: str, step, plan_id: str) -> None:
        msg = SubtaskCmd(plan_id=plan_id, index=step.index, action=step.action,
                         args=list(step.args), instruction=step.instruction, cancel=False)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub[which].publish(msg)

    def _send_cancel(self, which: str, plan_id: str, index: int) -> None:
        msg = SubtaskCmd(plan_id=plan_id, index=index, cancel=True)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub[which].publish(msg)

    def _check_target(self, target: str):
        """Synchronous detector query -> (found, detail). A non-empty detail with
        found=False means 'no verdict' (service missing / timeout / stale frame),
        which the executor treats as fail-open."""
        if not self.det_client.service_is_ready():
            return False, 'no_detector'
        fut = self.det_client.call_async(CheckTarget.Request(target=target))
        deadline = time.monotonic() + self._det_timeout
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not fut.done():
            return False, 'detector_timeout'
        r = fut.result()
        return bool(r.found), (f'{r.label} {r.confidence:.2f}' if r.found else r.detail)

    def _trace(self, kind: int, plan_id: str, index: int, title: str, body: str) -> None:
        msg = TraceEvent(plan_id=plan_id, kind=int(kind), index=int(index), title=title, body=body)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.trace_pub.publish(msg)

    def _status_llm(self, state: int, task: str, subtask: str, idx: int, count: int,
                    detail: str) -> None:
        msg = TaskStatus(task_name=task, current_subtask=subtask, subtask_index=int(idx),
                         subtask_count=int(count), state=int(state), detail=detail)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(msg)

    # --- inputs -----------------------------------------------------------
    def _on_transcript(self, msg: String) -> None:
        text = msg.data
        if self._exec is not None:                       # llm mode: no trigger matching
            self._on_transcript_llm(text)
            return
        # Buffer BEFORE the trigger check: a voice_keyword criterion reads this,
        # and an early return on a trigger match must not swallow the utterance.
        self._transcripts.append(text)
        for sc in self.scenarios:
            if any(kw in text for kw in sc.triggers):
                self._request(sc)
                return
        # No trigger matched — ignore (not every utterance is a command).

    # --- scenario lifecycle -----------------------------------------------
    def _request(self, sc: Scenario) -> None:
        """A trigger matched. Preempt the running scenario (fire-and-forget cancel)
        and start the new one immediately — open-loop, no wait."""
        if self._active is not None:
            self._publish_status(TaskStatus.STATE_PREEMPTED, detail='preempted by new trigger')
            self._stop_current()
        self._begin(sc)

    def _begin(self, sc: Scenario) -> None:
        self.get_logger().info(f'begin scenario {sc.name!r} ({len(sc.sub_tasks)} sub-tasks)')
        self._active = sc
        self._index = 0
        if not sc.sub_tasks:
            # A pure "stop" scenario: _request already preempted, nothing to run.
            self._publish_status(TaskStatus.STATE_SUCCEEDED, detail='stop')
            self._reset_exec()
            return
        self._enter_subtask()

    def _enter_subtask(self) -> None:
        st = self._current()
        if st is None:
            return
        self._transcripts.clear()        # voice_keyword sees only THIS sub-task's speech
        self._criterion = st.criterion
        self._started = False            # _t0 is set on on_start, not here — see _tick
        self._dispatch(st.on_create)                 # announce
        self._publish_status(TaskStatus.STATE_RUNNING, current_subtask=st.name)

    def _tick(self) -> None:
        st = self._current()
        if st is None:
            return
        if not self._started:
            # _t0 here (on_start), not on entry, so timeout/delay mean "since motion
            # began". Safe this late: the timeout below only runs once _started.
            self._t0 = self._now()
            self._dispatch(st.on_start)
            self._started = True
            return
        if self._criterion.evaluate(self, st.name):
            self._dispatch(st.on_success)
            self._advance()
        elif self._now() - self._t0 >= st.timeout_s:
            self._fail(f'{st.name} timeout')

    def _advance(self) -> None:
        self._index += 1
        if self._current() is not None:
            self._enter_subtask()
        else:
            self._publish_status(TaskStatus.STATE_SUCCEEDED)
            self._reset_exec()

    def _fail(self, reason: str) -> None:
        # Capture on_fail and publish FAILED while _active still stands (gone after
        # _stop_current → _reset_exec); cancel BEFORE announcing so barge-in
        # doesn't cut the on_fail message.
        st = self._current()
        on_fail = st.on_fail if st else []
        self._publish_status(TaskStatus.STATE_FAILED, detail=reason)
        self._stop_current()
        self._dispatch(on_fail)

    def _stop_current(self) -> None:
        """Fire-and-forget cancel of the current commands + clear exec state.
        The caller publishes the status (PREEMPTED / FAILED) first."""
        for conn in self.connectors.values():
            conn.cancel(self)
        self._reset_exec()

    # --- helpers ----------------------------------------------------------
    def _dispatch(self, hooks: list) -> None:
        # Each hook step routes to its connector (fire-and-forget, open-loop).
        for step in hooks:
            for label, payload in step.items():
                conn = self.connectors.get(label)
                if conn is None:
                    self.get_logger().warning(f'no connector for label {label!r}')
                    continue
                conn.dispatch(self, payload)

    def _current(self):
        if self._active and 0 <= self._index < len(self._active.sub_tasks):
            return self._active.sub_tasks[self._index]
        return None

    def _reset_exec(self) -> None:
        """Clear sub-task execution state (back to idle)."""
        self._active = None
        self._index = 0
        self._criterion = None
        self._started = False

    def _now(self) -> float:
        return time.monotonic()

    # --- read-only state for Criterion.evaluate ---------------------------
    def elapsed(self) -> float:
        """Seconds since the current sub-task's on_start dispatch."""
        return self._now() - self._t0

    @property
    def transcripts(self) -> list:
        """Utterances heard since the current sub-task started."""
        return self._transcripts

    def _publish_status(self, state, current_subtask='', detail='') -> None:
        msg = TaskStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_name = self._active.name if self._active else ''
        msg.current_subtask = current_subtask
        msg.subtask_index = self._index
        msg.subtask_count = len(self._active.sub_tasks) if self._active else 0
        msg.state = state
        msg.detail = detail
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrchestratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
