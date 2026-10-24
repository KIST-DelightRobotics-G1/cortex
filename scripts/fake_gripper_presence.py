#!/usr/bin/env python3
"""Stand-in for the detector's gripper judgment: publish cortex_msgs/GripperPresence by hand.

Use it to try the holding monitor before the detector side exists, or to check the wiring
from the detector side (same topic, same message). Type a state and Enter to change it:

    p  PRESENT     a  ABSENT     u  UNKNOWN     c  camera not ok (toggle)     q  quit

    ros2 run ... (or) python3 scripts/fake_gripper_presence.py --key cucumber --state p

The message is re-sent every 0.5 s like the real detector (heartbeat). Stop the script and
the orchestrator reads UNKNOWN after held_heartbeat_timeout_s.
"""
import argparse
import sys
import threading

import rclpy
from rclpy.node import Node

from cortex_msgs.msg import GripperPresence, ObjectPresence

STATES = {'p': ObjectPresence.PRESENT, 'a': ObjectPresence.ABSENT, 'u': ObjectPresence.UNKNOWN}
NAMES = {v: k for k, v in (('PRESENT', 1), ('ABSENT', 2), ('UNKNOWN', 0))}


class FakeGripper(Node):
    def __init__(self, topic: str, key: str, state: int) -> None:
        super().__init__('fake_gripper_presence')
        self.key, self.state, self.camera_ok = key, state, True
        self.pub = self.create_publisher(GripperPresence, topic, 10)
        self.create_timer(0.5, self.publish)
        self.publish()

    def publish(self) -> None:
        msg = GripperPresence(camera_ok=self.camera_ok, objects=[ObjectPresence(
            key=self.key, state=self.state, confidence=0.9 if self.state == ObjectPresence.PRESENT else 0.0)])
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def set(self, line: str) -> bool:
        line = line.strip().lower()
        if line == 'q':
            return False
        if line == 'c':
            self.camera_ok = not self.camera_ok
        elif line in STATES:
            self.state = STATES[line]
        else:
            print('p / a / u / c / q')
            return True
        self.publish()                                   # a change goes out at once
        print(f'{self.key}: {NAMES[self.state]}  camera_ok={self.camera_ok}')
        return True


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/cortex/gripper_presence')
    ap.add_argument('--key', default='cucumber')
    ap.add_argument('--state', default='p', choices=sorted(STATES))
    args = ap.parse_args()
    rclpy.init()
    node = FakeGripper(args.topic, args.key, STATES[args.state])
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print(f'{args.key}: {NAMES[node.state]} on {args.topic} — p / a / u / c / q')
    try:
        for line in sys.stdin:
            if not node.set(line):
                break
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
