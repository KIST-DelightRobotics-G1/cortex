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
        self.declare_parameter('request_timeout_s', 20.0)
        self.declare_parameter('dummy_line_delay_s', 0.15)  # mimics streaming cadence

        g = self.get_parameter
        self.cfg = planner.load_config(g('actions_path').value)
        self.backend = g('backend').value
        self.model = g('model').value
        self.timeout_s = float(g('request_timeout_s').value)
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
            f'{len([a for a, s in self.cfg["actions"].items() if s.get("enabled")])} actions enabled)')

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

        def on_line(ln: planner.Line) -> None:
            nonlocal stopped
            if should_stop():
                return
            step = PlanStep(plan_id=req.plan_id)
            if ln.kind == 'sub':
                step.kind = PlanStep.KIND_SUB
                step.index = ln.i
                step.action = ln.action
                step.args = list(ln.args)
                step.say = ln.say
                if ln.say_warn:
                    self.get_logger().warning(f'say warn {ln.say_warn} on {ln.raw.strip()}')
            elif ln.kind == 'end':
                step.kind = PlanStep.KIND_END
                step.index = ln.n
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

        self.get_logger().info(f'plan {req.plan_id}: {req.utterance!r} (state={req.state})')
        try:
            if self.backend == 'dummy':
                self._dummy(req, on_line, should_stop)
            else:
                dt = planner.stream_plan(self._client, self.model, self.cfg, req.utterance,
                                         req.state, on_line, should_stop)
                self.get_logger().info(f'plan {req.plan_id}: stream done in {dt:.2f}s')
        except Exception as e:                     # API failure → the orchestrator hears an error
            self.get_logger().error(f'plan {req.plan_id}: {e}')
            if not should_stop():
                self.step_pub.publish(PlanStep(plan_id=req.plan_id, kind=PlanStep.KIND_ERROR,
                                               detail=f'api: {e}'))

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
        if '안녕' in text or '고마' in text:
            return find(lambda u: u == '안녕')
        if '오이' in text and ('테이블' in text or '식탁' in text):
            return find(lambda u: '테이블' in u)
        if '오이' in text:
            return find(lambda u: '오이' in u and '가져' in u)
        if '트레이' in text:
            return find(lambda u: '트레이' in u)
        if '조리대' in text:
            return find(lambda u: u == '조리대로 가')
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
