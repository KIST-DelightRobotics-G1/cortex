"""mock_module_node — stands in for nav-planner or the VLA runner.

Implements the module side of Subtask_state_interface_spec v0.1 so the whole
llm-mode loop (llm_node → orchestrator → module → GUI) runs on a laptop:

    SubtaskCmd  (cmd_topic)   →  IDLE → RUNNING (≤100 ms) → DONE ×3 → IDLE
    cancel=true               →  nav: stop now, IDLE "cancelled" ×3
                                 vla: finish the step first (safe point), then IDLE
    state published at 10 Hz always (IDLE included), RELIABLE depth 10

Parameters
    role            nav | vla — cancel semantics and which actions are supported
    duration_s      how long a step "takes"
    fail_actions    actions that FAIL after duration_s (e.g. ['pick'] to test the failure path)
    ignore_cmds     do not respond at all (tests accept_timeout / not accepted)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from cortex_msgs.msg import SubtaskCmd, SubtaskState

# actions.yaml 의 exec 와 같아야 한다 — 여기가 좁으면 계획이 중간에 "지원하지 않음" 으로 끊긴다.
SUPPORTED = {
    'nav': {'move_to'},
    'vla': {'adjust', 'close', 'empty', 'fill', 'handover', 'insert', 'lock', 'open', 'pick', 'place', 'pour', 'put_in', 'receive', 'remove', 'take_out', 'turn_off', 'turn_on', 'unlock', 'wipe'},
}


class MockModuleNode(Node):
    def __init__(self) -> None:
        super().__init__('mock_module_node')
        self.declare_parameter('role', 'nav')
        self.declare_parameter('cmd_topic', '/cortex/nav/cmd')
        self.declare_parameter('state_topic', '/cortex/nav/state')
        self.declare_parameter('duration_s', 3.0)
        self.declare_parameter('fail_actions', [''])
        self.declare_parameter('ignore_cmds', False)
        self.declare_parameter('rate_hz', 10.0)
        g = self.get_parameter
        self.role = g('role').value
        self.duration = float(g('duration_s').value)
        self.fail_actions = {a for a in g('fail_actions').value if a}
        self.ignore = bool(g('ignore_cmds').value)

        self.status = SubtaskState.IDLE
        self.plan_id, self.index, self.action, self.detail = '', 0, '', ''
        self.progress = 0.0
        self._t0 = 0.0
        self._final_left = 0            # remaining DONE/FAILED/cancelled repeats
        self._cancel_deferred = False

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(SubtaskState, g('state_topic').value, reliable)
        self.create_subscription(SubtaskCmd, g('cmd_topic').value, self._on_cmd, reliable)
        self.create_timer(1.0 / float(g('rate_hz').value), self._tick)
        self.get_logger().info(f'mock {self.role} up: {g("cmd_topic").value} -> {g("state_topic").value}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, m: SubtaskCmd) -> None:
        if self.ignore:
            return
        if m.cancel:
            if self.status != SubtaskState.RUNNING or (m.plan_id, m.index) != (self.plan_id, self.index):
                return
            if self.role == 'nav':
                self._finish(SubtaskState.IDLE, 'cancelled')
                self.get_logger().info(f'cancel {m.plan_id}/{m.index}: stopped now')
            else:
                self._cancel_deferred = True
                self.get_logger().info(f'cancel {m.plan_id}/{m.index}: deferred to safe point')
            return
        # new command: preempts whatever is running (spec 02 transition table)
        if m.action not in SUPPORTED[self.role]:
            self.plan_id, self.index, self.action = m.plan_id, m.index, m.action
            self._finish(SubtaskState.FAILED, f'unsupported: action {m.action}')
            return
        self.plan_id, self.index, self.action = m.plan_id, m.index, m.action
        self.status, self.detail, self.progress = SubtaskState.RUNNING, '', 0.0
        self._t0 = self._now()
        self._final_left = 0
        self._cancel_deferred = False
        self.get_logger().info(f'run {m.plan_id}/{m.index} {m.action}{list(m.args)} '
                               f'{m.instruction!r}')

    def _finish(self, status: int, detail: str) -> None:
        self.status, self.detail = status, detail
        self.progress = 1.0 if status == SubtaskState.DONE else self.progress
        self._final_left = 3

    def _tick(self) -> None:
        if self.status == SubtaskState.RUNNING:
            el = self._now() - self._t0
            self.progress = min(el / self.duration, 1.0) if self.duration > 0 else 1.0
            if el >= self.duration:
                if self.action in self.fail_actions:
                    self._finish(SubtaskState.FAILED, 'mock failure')
                else:
                    self._finish(SubtaskState.DONE, '')
                if self._cancel_deferred:
                    self.detail = (self.detail + ' ' if self.detail else '') + 'cancelled_at_safe_point'
        msg = SubtaskState(plan_id=self.plan_id, index=self.index, action=self.action,
                           status=self.status, progress=float(self.progress), detail=self.detail)
        msg.stamp_ns = self.get_clock().now().nanoseconds
        self.pub.publish(msg)
        if self._final_left > 0:
            self._final_left -= 1
            if self._final_left == 0:            # 3 repeats done → IDLE, clean
                self.status = SubtaskState.IDLE
                self.plan_id, self.index, self.action, self.detail = '', 0, '', ''
                self.progress = 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockModuleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
