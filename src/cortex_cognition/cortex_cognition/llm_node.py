"""llm_node — utterance → streamed subtask lines.

    PlanRequest (/cortex/llm/request) ──> worker thread ──> PlanStep × N (/cortex/llm/step)

One request, one LLM call, one PlanStep per NDJSON line as it arrives — the
orchestrator starts step 0 before the LLM has finished the plan. Lines are
validated here (planner.validate_line) against config/actions.yaml; a line that
fails validation ends the stream with KIND_ERROR, and nothing after it is sent.

Latest-wins: a new PlanRequest aborts the stream in flight (the worker checks a
generation counter between chunks) and starts the new one. The orchestrator
correlates by plan_id, so a stale step that still slips out is ignored there.

Backends (parameter `backend`):
    gemini   google-genai, model e.g. gemini-3.6-flash   (GOOGLE_API_KEY)
    openai   openai SDK,   model e.g. gpt-4.1-mini        (OPENAI_API_KEY)
    dummy    replays the few-shot examples from actions.yaml with a fake
             per-line delay — the whole loop runs with no network.
"""

import json
import os
import queue
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cortex_msgs.msg import PlanRequest, PlanStep

from . import planner


class LlmNode(Node):
    def __init__(self) -> None:
        super().__init__('llm_node')
        default_yaml = os.path.join(
            get_package_share_directory('cortex_cognition'), 'config', 'actions.yaml')
        self.declare_parameter('request_topic', '/cortex/llm/request')
        self.declare_parameter('step_topic', '/cortex/llm/step')
        self.declare_parameter('actions_path', default_yaml)
        self.declare_parameter('backend', 'dummy')          # dummy | gemini | openai
        self.declare_parameter('model', 'gemini-3.6-flash')
        self.declare_parameter('request_timeout_s', 15.0)      # 전체 응답 제한
        self.declare_parameter('first_line_timeout_s', 5.0)   # 첫 줄이 이 안에 와야 한다 (측정 p95 1.9s)
        self.declare_parameter('retries', 1)                  # 타임아웃·API 오류 시 재시도 횟수
        self.declare_parameter('dummy_line_delay_s', 0.15)  # mimics streaming cadence

        g = self.get_parameter
        self.cfg = planner.load_config(g('actions_path').value)
        self.backend = g('backend').value
        self.model = g('model').value
        self.timeout_s = float(g('request_timeout_s').value)
        self.first_line_timeout_s = float(g('first_line_timeout_s').value)
        self.retries = int(g('retries').value)
        self.dummy_delay = float(g('dummy_line_delay_s').value)

        self._client = None
        if self.backend in ('gemini', 'openai'):
            key = os.environ.get('GOOGLE_API_KEY' if self.backend == 'gemini' else 'OPENAI_API_KEY')
            if not key:
                self.get_logger().error(
                    f'backend={self.backend} but no API key in the environment; '
                    'falling back to dummy')
                self.backend = 'dummy'
            else:
                self._client = planner.make_client(self.model, key)

        self._gen = 0                              # bumped per request → aborts the older stream
        self._q: queue.Queue = queue.Queue()
        self.step_pub = self.create_publisher(PlanStep, g('step_topic').value, 10)
        self.create_subscription(PlanRequest, g('request_topic').value, self._on_request, 10)
        threading.Thread(target=self._worker, daemon=True, name='llm-worker').start()
        self.get_logger().info(
            f'llm_node up (backend={self.backend}, model={self.model}, '
            f'{len([a for a, s in self.cfg["actions"].items() if s.get("enabled") is not False])} actions, '
            f'{len(self.cfg["nav"]["places"])} places)')

    # --- input ------------------------------------------------------------
    def _on_request(self, msg: PlanRequest) -> None:
        self._gen += 1
        self._q.put((self._gen, msg))

    # --- worker -----------------------------------------------------------
    def _worker(self) -> None:
        while rclpy.ok():
            try:
                gen, req = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            if gen != self._gen:                   # superseded while queued
                continue
            self._serve(gen, req)

    def _serve(self, gen: int, req: PlanRequest) -> None:
        stopped = False                            # set once an error line closes the stream

        def should_stop() -> bool:
            return stopped or gen != self._gen

        emitted = 0        # 실제로 내보낸 subtask 수 — 건너뛴 줄이 있으면 index 를 다시 매긴다

        def on_line(ln: planner.Line) -> None:
            nonlocal stopped, emitted
            if should_stop():
                return
            step = PlanStep(plan_id=req.plan_id)
            if ln.kind == 'sub':
                # 검증기 2: 별칭을 모듈이 아는 키로 바꾼 뒤 실행 가능성을 본다.
                planner.normalize(ln, self.cfg)
                ok, reason = planner.feasibility(ln, self.cfg)
                if reason == 'skip_nearby':
                    # 문·서랍처럼 장소가 아닌 대상에 붙은 move_to — 이 줄만 버리고 계획은 잇는다.
                    self.get_logger().info(f'skip {ln.action}({", ".join(ln.args)}): not a place')
                    return
                if not ok:
                    # 할 수 없는 일이다. 계획을 여기서 끊고 사유를 오케스트레이터에 넘긴다.
                    # detail 형식 "<phrases 키>|<대상>" — 오케스트레이터가 한국어 안내로 바꾼다.
                    target = ln.args[0] if ln.args else ''
                    self.get_logger().info(f'block {ln.action}({", ".join(ln.args)}): {reason}')
                    self.step_pub.publish(PlanStep(plan_id=req.plan_id, kind=PlanStep.KIND_ERROR,
                                                   detail=f'{reason}|{target}'))
                    stopped = True
                    return
                step.kind = PlanStep.KIND_SUB
                step.index = emitted
                emitted += 1
                step.action = ln.action
                step.args = list(ln.args)
                step.say = ln.say
                if ln.say_warn:
                    self.get_logger().warning(f'say warn {ln.say_warn} on {ln.raw.strip()}')
            elif ln.kind == 'end':
                step.kind = PlanStep.KIND_END
                step.index = emitted        # LLM 이 센 n 이 아니라 실제로 내보낸 수
            elif ln.kind == 'reply':
                step.kind = PlanStep.KIND_REPLY
                step.reply_kind = ln.reply_type
                step.say = ln.say
            else:                                  # error
                step.kind = PlanStep.KIND_ERROR
                step.detail = f'{ln.error}: {ln.raw.strip()}'
                stopped = True
            step.detail = step.detail or ln.raw.strip()   # raw line rides along for the debug view
            self.step_pub.publish(step)

        at = req.at or 'home'
        holding = req.holding or 'none'
        self.get_logger().info(
            f'plan {req.plan_id}: {req.utterance!r} (mode={req.state}, at={at}, holding={holding})')
        if self.backend == 'dummy':
            try:
                self._dummy(req, on_line, should_stop)
            except Exception as e:
                self._fail(req, should_stop, f'dummy: {e}')
            return
        # 재시도는 아직 아무 줄도 못 낸 경우에만 안전하다 — 이미 내보낸 뒤면 계획이 섞인다.
        for attempt in range(self.retries + 1):
            try:
                dt = planner.stream_plan(self._client, self.model, self.cfg, req.utterance,
                                         req.state, on_line, should_stop, at=at, holding=holding,
                                         timeout_s=self.timeout_s,
                                         first_line_timeout_s=self.first_line_timeout_s)
                self.get_logger().info(f'plan {req.plan_id}: stream done in {dt:.2f}s')
                return
            except Exception as e:
                if should_stop() or emitted or attempt >= self.retries:
                    self._fail(req, should_stop, f'{type(e).__name__}: {e}')
                    return
                self.get_logger().warning(f'plan {req.plan_id}: {e} — retrying ({attempt + 1})')

    def _fail(self, req: PlanRequest, should_stop, why: str) -> None:
        self.get_logger().error(f'plan {req.plan_id}: {why}')
        if not should_stop():
            self.step_pub.publish(PlanStep(plan_id=req.plan_id, kind=PlanStep.KIND_ERROR,
                                           detail=f'api|{why}'))

    # --- dummy backend ----------------------------------------------------
    def _dummy(self, req: PlanRequest, on_line, should_stop) -> None:
        import time
        text = req.utterance
        ex = self._pick_example(text)
        lines = []
        if ex is None:
            lines.append(json.dumps({'reply': 'none', 'say': f'{text[:12]}… 는 아직 못 합니다.'},
                                    ensure_ascii=False))
        elif 'lines' in ex:
            for ln in ex['lines']:
                lines.append(json.dumps({'i': ln['i'], 'a': ln['a'], 'args': ln['args'],
                                         'say': ln['say']}, ensure_ascii=False))
        else:
            lines.append(json.dumps(ex['reply'], ensure_ascii=False))
        lines.append(json.dumps({'end': True, 'n': len([ln for ln in lines if '"i"' in ln])}))

        def pieces():
            for ln in lines:
                time.sleep(self.dummy_delay)
                yield ln + '\n'
        planner.stream_lines(pieces(), self.cfg, on_line, should_stop)

    def _pick_example(self, text: str):
        """Crude keyword routing over the few-shot examples — dummy only."""
        exs = self.cfg.get('examples', [])
        def find(pred):
            return next((e for e in exs if pred(e['utterance'])), None)
        for key in ('안녕', '고마'):
            if key in text:
                return find(lambda u: '안녕' in u)
        for key in ('리모컨', '컵', '찬장', '선풍기', '주전자', '따라', '창문', '노래'):
            if key in text:
                hit = find(lambda u, k=key: k in u)
                if hit:
                    return hit
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LlmNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
