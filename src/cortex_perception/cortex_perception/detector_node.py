"""Camera -> YOLO -> DetectionArray and buffered CheckTarget [SYS-REQ-44].

The ROS-free adapter/policy lives in detection.py and is shared by offline tests.
The default 'always' backend is an explicit demo stub. Use the real-detector
launch profile with a local .pt weight for fridge/cucumber integration.
"""
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from cortex_msgs.msg import Detection, DetectionArray
from cortex_msgs.srv import CheckTarget
from .detection import (DetectionRecord, PresenceWindow, Verdict, YoloDetector,
                        fill_detection, fill_response, frame_time, target_mapping)


class DetectorNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('detector_node', **kwargs)
        defaults = {
            'camera_topic': '/bridge/sensors/camera/color/compressed',
            'camera_transport': 'compressed', 'detections_topic': '/cortex/detections',
            'service': '/cortex/detector/check', 'backend': 'always',
            'model': 'yolo26s.pt', 'model_sha256': '', 'device': '', 'rate_hz': 8.0, 'window_s': 0.6,
            'default_min_confidence': 0.4, 'infer_conf': 0.25, 'imgsz': 640,
            'min_frames': 3, 'allow_unstamped': False, 'require_latest_hit': False,
            'target_keys': ['fridge_door', 'fridge', 'cucumber', 'user'],
            'target_classes': ['refrigerator', 'refrigerator', 'cucumber', 'person'],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        g = lambda key: self.get_parameter(key).value
        self.window_s = float(g('window_s'))
        self.allow_unstamped = bool(g('allow_unstamped'))
        self.targets = target_mapping(list(g('target_keys')), list(g('target_classes')))
        self.backend, self.model_name = str(g('backend')), str(g('model'))
        rate, infer_conf = float(g('rate_hz')), float(g('infer_conf'))
        if self.backend not in ('always', 'yolo'):
            raise ValueError('backend must be always or yolo')
        if not math.isfinite(rate) or rate <= 0 or not 0 < infer_conf <= 1 or int(g('imgsz')) <= 0:
            raise ValueError('invalid inference settings')
        if g('camera_transport') not in ('raw', 'compressed'):
            raise ValueError('camera_transport must be raw or compressed')
        self._lock = threading.Lock()
        self._frame = None
        self._last_processed_stamp = -1
        self._model = None
        if self.backend == 'yolo':
            try:
                self._model = YoloDetector(self.model_name, str(g('device')), int(g('imgsz')), infer_conf,
                                           str(g('model_sha256')))
            except Exception as error:
                self.get_logger().error(f'YOLO not loaded: {error}')
        names = self._model.names.values() if self._model else []
        self._window = PresenceWindow(self.targets, names, self.window_s,
                                      float(g('default_min_confidence')), int(g('min_frames')),
                                      bool(g('require_latest_hit')))
        if self.backend == 'yolo' and self._model is None:
            self._window.invalidate('model_not_loaded')
        self.det_pub = self.create_publisher(DetectionArray, g('detections_topic'), 10)
        # Camera/check callbacks stay responsive while a single inference is running.
        # Reentrant inference timers can overlap for a large model taking >125 ms.
        self._io_group = ReentrantCallbackGroup()
        self._infer_group = MutuallyExclusiveCallbackGroup()
        camera_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        if g('camera_transport') == 'raw':
            self.create_subscription(Image, g('camera_topic'), self._on_raw, camera_qos,
                                     callback_group=self._io_group)
        else:
            self.create_subscription(CompressedImage, g('camera_topic'), self._on_compressed, camera_qos,
                                     callback_group=self._io_group)
        self.create_service(CheckTarget, g('service'), self._on_check, callback_group=self._io_group)
        self.create_timer(1.0/rate, self._infer, callback_group=self._infer_group)
        self.get_logger().info(f'detector backend={self.backend}, model={self.model_name}, '
                               f'loaded={self._model is not None}, min_frames={g("min_frames")}')

    def _accept_frame(self, frame, header):
        stamp_ns = header.stamp.sec*1_000_000_000 + header.stamp.nanosec
        timing = frame_time(stamp_ns, self.get_clock().now().nanoseconds,
                            time.monotonic(), self.window_s, self.allow_unstamped)
        if timing is None:
            return
        observed_t, stamp_ns = timing
        with self._lock:
            if self._frame is not None and stamp_ns <= self._frame[2]:
                return
            self._frame = (frame, observed_t, stamp_ns, header.frame_id)

    def _on_compressed(self, message):
        try:
            import cv2
            frame = cv2.imdecode(np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        except Exception:
            return
        if frame is not None:
            self._accept_frame(frame, message.header)

    def _on_raw(self, message):
        if message.encoding not in ('bgr8', 'rgb8') or message.height <= 0 or message.width <= 0:
            return
        if message.step < message.width*3 or len(message.data) < message.height*message.step:
            return
        rows = np.frombuffer(bytes(message.data), dtype=np.uint8,
                             count=message.height*message.step).reshape(message.height, message.step)
        frame = rows[:, :message.width*3].reshape(message.height, message.width, 3)
        if message.encoding == 'rgb8':
            frame = frame[:, :, ::-1]
        self._accept_frame(frame.copy(), message.header)

    def _infer(self):
        with self._lock:
            latest = self._frame
        if latest is None or self._model is None:
            return
        frame, observed_t, stamp_ns, frame_id = latest
        if stamp_ns <= self._last_processed_stamp:
            return
        self._last_processed_stamp = stamp_ns
        if time.monotonic()-observed_t > self.window_s:
            return
        started = time.perf_counter()
        try:
            detections = self._model.predict(frame)
        except Exception as error:
            self._window.invalidate('inference_error')
            self.get_logger().warning(f'YOLO predict failed: {error}')
            return
        ms = (time.perf_counter()-started)*1000
        if not self._window.add(detections, observed_t, stamp_ns, time.monotonic()):
            return
        out = DetectionArray(model=self.model_name, infer_ms=float(ms))
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        out.header.frame_id = frame_id
        out.detections = [fill_detection(Detection(), d) for d in detections]
        self.det_pub.publish(out)

    def _on_check(self, request, response):
        if self.backend == 'always':
            labels = self.targets.get(request.target)
            result = (Verdict(True, 'stub', DetectionRecord(sorted(labels)[0], 1., .5, .5, 0., 0.),
                              self.get_clock().now().nanoseconds)
                      if labels else Verdict(detail='unknown_target'))
        else:
            result = self._window.check(request.target, time.monotonic(),
                                        request.min_confidence, request.max_age_s)
        return fill_response(response, result)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
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
