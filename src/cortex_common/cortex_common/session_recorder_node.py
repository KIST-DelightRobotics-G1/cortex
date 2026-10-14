"""session_recorder_node — /cortex/events → one JSONL file per voice command (session_log.py).

Parameters come from logging.yaml's session_log section (the launch file passes them).
"""
import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .events import EVENTS_TOPIC
from .session_log import SessionLog


class SessionRecorderNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('session_recorder_node', **kwargs)
        defaults = {'dir': '~/cortex_logs', 'pre_roll_s': 5.0, 'post_roll_s': 2.0,
                    'end_timeout_s': 120.0, 'keep_days': 14}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        g = lambda key: self.get_parameter(key).value
        self.log = SessionLog(g('dir'), float(g('pre_roll_s')), float(g('post_roll_s')),
                              float(g('end_timeout_s')), int(g('keep_days')))
        self._prune()
        self.create_subscription(String, EVENTS_TOPIC, lambda m: self.log.add(m.data),
                                 QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.5, lambda: self.log.tick(self.get_clock().now().nanoseconds))
        self.create_timer(3600.0, self._prune)
        self.get_logger().info(f'session_recorder_node up: {self.log.dir} '
                               f'(keep {self.log.keep_days} days)')

    def _prune(self) -> None:
        gone = self.log.prune(datetime.date.today())
        if gone:
            self.get_logger().info(f'session log: removed {len(gone)} old day(s): {", ".join(gone)}')

    def destroy_node(self):
        self.log.close_all()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SessionRecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
