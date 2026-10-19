"""Real DDS topics/services with a deterministic backend; no GPU or weights needed."""
import time

import pytest

rclpy = pytest.importorskip('rclpy', reason='requires ROS 2 Humble and colcon-built interfaces')
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from cortex_msgs.msg import DetectionArray
from cortex_msgs.srv import CheckTarget
from cortex_perception import detector_node
from cortex_perception.detection import DetectionRecord


def test_two_camera_dds_inputs_and_roi_service_results_are_independent(monkeypatch):
    class Backend:
        names = {0: 'refrigerator', 1: 'cucumber'}
        def __init__(self, *args):
            pass
        def predict(self, frame):
            value = int(frame[0, 0, 0])
            label = 'refrigerator' if value == 1 else 'cucumber'
            xy = .1 if value == 3 else .7
            return (DetectionRecord(label, .6 if value == 4 else .9, xy, xy, .1, .1),)
    monkeypatch.setattr(detector_node, 'YoloDetector', Backend)
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    nodes = []
    try:
        probe = Node('wrist_check_probe')
        nodes.append(probe)
        clients, publishers, received = {}, {}, {'ego': [], 'wrist': []}
        for name, key, label in [('ego', 'fridge_door', 'refrigerator'),
                                 ('wrist', 'held_cucumber', 'cucumber')]:
            prefix = '/wrist_test/' + name
            values = dict(backend='yolo', camera_transport='raw', camera_topic=prefix+'/image',
                          service=prefix+'/check', detections_topic=prefix+'/detections',
                          rate_hz=30., window_s=.6, min_frames=3, require_latest_hit=True,
                          target_keys=[key], target_classes=[label])
            if name == 'wrist':
                values.update(check_roi=[.5, .5, 1., 1.], roi_min_overlap=.5)
            values['default_min_confidence'] = .7 if name == 'wrist' else .25
            node = detector_node.DetectorNode(
                cli_args=['--ros-args', '-r', '__node:='+name+'_detector'],
                parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
            nodes.append(node)
            clients[name] = probe.create_client(CheckTarget, prefix+'/check')
            publishers[name] = probe.create_publisher(Image, prefix+'/image', 10)
            probe.create_subscription(DetectionArray, prefix+'/detections', received[name].append, 10)
        for node in nodes:
            executor.add_node(node)

        def spin_until(predicate, timeout=5.):
            deadline = time.monotonic()+timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            assert predicate(), 'ROS discovery or callback timed out'

        def check(name, target, max_age=0.):
            future = clients[name].call_async(CheckTarget.Request(target=target, max_age_s=max_age))
            spin_until(future.done)
            return future.result()

        def feed(wrist_value):
            deadline = time.monotonic()+.8   # replace the entire previous .6 s window
            while time.monotonic() < deadline:
                for name, value in [('ego', 1), ('wrist', wrist_value)]:
                    msg = Image(height=1, width=1, encoding='bgr8', step=3, data=bytes([value]*3))
                    msg.header.stamp = probe.get_clock().now().to_msg()
                    msg.header.frame_id = name
                    publishers[name].publish(msg)
                executor.spin_once(timeout_sec=.01)
                time.sleep(.01)

        spin_until(lambda: all(c.service_is_ready() for c in clients.values()) and
                   all(p.get_subscription_count() for p in publishers.values()))
        assert check('wrist', 'held_cucumber').detail == 'no_frame'
        feed(2)  # ego refrigerator + wrist cucumber inside the configured region
        assert check('ego', 'fridge_door').found
        assert check('wrist', 'held_cucumber').found
        assert check('ego', 'held_cucumber').detail == 'unknown_target'
        assert not check('wrist', 'held_cucumber', 1e-6).found  # no reuse at PRECHECK entry
        feed(4)  # wrist candidate below its .70 threshold remains visible in raw output
        assert check('ego', 'fridge_door').found
        assert not check('wrist', 'held_cucumber').found
        assert received['wrist'][-1].detections[0].confidence == pytest.approx(.6)
        feed(3)  # cucumber outside wrist ROI is still published, but cannot authorize
        assert check('ego', 'fridge_door').found
        assert not check('wrist', 'held_cucumber').found
        assert received['wrist'][-1].detections[0].cx == pytest.approx(.1)
        assert all(m.header.frame_id == 'wrist' for m in received['wrist'])
        time.sleep(.65)
        assert check('wrist', 'held_cucumber').detail == 'stale'
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
