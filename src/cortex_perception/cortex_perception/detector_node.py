"""Camera -> YOLO -> DetectionArray and buffered CheckTarget [SYS-REQ-44].

The ROS-free adapter/policy lives in detection.py and is shared by offline tests.
The default 'always' backend is an explicit demo stub. Use the real-detector
launch profile with a local .pt weight for fridge/cucumber integration.
"""
import hashlib
import math
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from kist_msgs.msg import CompressedColorFrame

from cortex_msgs.msg import Detection, DetectionArray
from cortex_msgs.srv import CheckTarget
from .detection import (DetectionRecord, PresenceWindow, Verdict, YoloDetector,
                        fill_detection, fill_response, frame_time, target_mapping)


from .detector_diagnostics import DetectorDiagnostics


class DetectorNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('detector_node', **kwargs)
        defaults = {
            'camera_topic': '/kist/camera/head/color/h264',
            'camera_transport': 'h264', 'detections_topic': '/cortex/detections',
            'service': '/cortex/detector/check', 'backend': 'always',
            'model': 'yolo26s.pt', 'model_sha256': '', 'device': '', 'rate_hz': 8.0, 'window_s': 0.6,
            'default_min_confidence': 0.4, 'infer_conf': 0.25, 'imgsz': 640,
            'diagnostics_enabled': False, 'diagnostics_period_s': 1.0,
            'diagnostics_topic': '/cortex/detector/diagnostics',
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
        if g('camera_transport') not in ('h264', 'raw', 'compressed'):
            raise ValueError('camera_transport must be h264, raw or compressed')
        self._diag = DetectorDiagnostics()
        self._diag_config = None
        if g('diagnostics_enabled'):
            period = float(g('diagnostics_period_s'))
            if not math.isfinite(period) or period < 0.1:
                raise ValueError('diagnostics_period_s must be finite and >= 0.1')
            pub = self.create_publisher(String, g('diagnostics_topic'),
                                       QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
            self._diag = DetectorDiagnostics(lambda data: pub.publish(String(data=data)),
                                             lambda: self.get_clock().now().nanoseconds)
            self._diag_config = {key: g(key) for key in defaults}
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
        if self._diag.enabled:
            try:
                actual_hash = hashlib.sha256(Path(self.model_name).read_bytes()).hexdigest()
            except OSError:
                actual_hash = None
            self._diag_config.update(actual_model_sha256=actual_hash,
                                     loaded=self._model is not None,
                                     classes=self._model.names if self._model else {})
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
        if self.backend == 'always':
            pass  # The demo stub does not decode camera frames.
        elif g('camera_transport') == 'h264':
            from .ext_sensor import H264Decoder
            self._decoder = H264Decoder()
            self._decode_group = MutuallyExclusiveCallbackGroup()
            self.create_subscription(CompressedColorFrame, g('camera_topic'), self._on_h264,
                                     camera_qos, callback_group=self._decode_group)
        elif g('camera_transport') == 'raw':
            self.create_subscription(Image, g('camera_topic'), self._on_raw, camera_qos,
                                     callback_group=self._io_group)
        else:
            self.create_subscription(CompressedImage, g('camera_topic'), self._on_compressed, camera_qos,
                                     callback_group=self._io_group)
        self.create_service(CheckTarget, g('service'), self._on_check, callback_group=self._io_group)
        self.create_timer(1.0/rate, self._infer, callback_group=self._infer_group)
        if self._diag.enabled:
            self.create_timer(period, lambda: self._diag.heartbeat(config=self._diag_config),
                              callback_group=self._io_group)
        self.get_logger().info(f'detector backend={self.backend}, model={self.model_name}, '
                               f'loaded={self._model is not None}, min_frames={g("min_frames")}')

    def _accept_frame(self, frame, header):
        stamp_ns = header.stamp.sec*1_000_000_000 + header.stamp.nanosec
        self._accept_frame_data(frame, stamp_ns, header.frame_id)

    def _accept_frame_data(self, frame, stamp_ns, frame_id):
        ros_now_ns = self.get_clock().now().nanoseconds
        age_ms = (ros_now_ns-stamp_ns)/1e6 if stamp_ns > 0 else None
        timing = frame_time(stamp_ns, ros_now_ns,
                            time.monotonic(), self.window_s, self.allow_unstamped)
        if timing is None:
            reason = 'unstamped' if stamp_ns <= 0 else 'future_stamp' if age_ms < -100 else 'stale_input'
            self._diag.record(reason, source_stamp_ns=stamp_ns, source_age_ms=age_ms)
            return
        observed_t, stamp_ns = timing
        with self._lock:
            if self._frame is not None and stamp_ns <= self._frame[2]:
                self._diag.record('non_increasing_stamp', source_stamp_ns=stamp_ns)
                return
            self._frame = (frame, observed_t, stamp_ns, frame_id)
        self._diag.record('accepted_frame', source_stamp_ns=stamp_ns, source_age_ms=age_ms)

    def _on_h264(self, message):
        from .ext_sensor import decoded_stamp_ns
        # Decode every packet to retain codec reference state. The decoded frame
        # may belong to an earlier packet; never stamp it with the newest packet.
        self._diag.record('camera_message')
        started = time.perf_counter()
        frame = self._decoder.decode(message.data, message.is_keyframe, message.stamp_ns)
        self._diag.record('h264_' + getattr(self._decoder, 'last_status', 'unknown'),
                          decode_ms=(time.perf_counter()-started)*1000)
        if frame is not None:
            stamp_ns = decoded_stamp_ns(frame)
            if stamp_ns is not None:
                self._accept_frame_data(frame.to_ndarray(format='bgr24'), stamp_ns, message.frame_id)
            else:
                self._diag.record('decoded_missing_pts')

    def _on_compressed(self, message):
        self._diag.record('camera_message')
        try:
            import cv2
            frame = cv2.imdecode(np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        except Exception:
            self._diag.record('image_decode_error')
            return
        if frame is not None:
            self._accept_frame(frame, message.header)
        else:
            self._diag.record('image_decode_error')

    def _on_raw(self, message):
        self._diag.record('camera_message')
        if message.encoding not in ('bgr8', 'rgb8') or message.height <= 0 or message.width <= 0:
            self._diag.record('invalid_raw_image')
            return
        if message.step < message.width*3 or len(message.data) < message.height*message.step:
            self._diag.record('invalid_raw_image')
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
            self._diag.record('stale_before_inference', source_stamp_ns=stamp_ns,
                              source_age_ms=(time.monotonic()-observed_t)*1000)
            return
        started = time.perf_counter()
        try:
            detections = self._model.predict(frame)
        except Exception as error:
            self._window.invalidate('inference_error')
            self._diag.record('inference_error', error=str(error))
            self.get_logger().warning(f'YOLO predict failed: {error}')
            return
        ms = (time.perf_counter()-started)*1000
        if self._diag.enabled:
            self._diag.record('inference_completed', source_stamp_ns=stamp_ns, infer_ms=ms,
                              source_age_ms=(time.monotonic()-observed_t)*1000,
                              max_confidence={label: max(d.confidence for d in detections if d.label == label)
                                              for label in {d.label for d in detections}})
        if not self._window.add(detections, observed_t, stamp_ns, time.monotonic()):
            self._diag.record('window_add_rejected', source_stamp_ns=stamp_ns)
            return
        out = DetectionArray(model=self.model_name, infer_ms=float(ms))
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        out.header.frame_id = frame_id
        out.detections = [fill_detection(Detection(), d) for d in detections]
        self.det_pub.publish(out)
        self._diag.record('detections_published')

    def _on_check(self, request, response):
        if self.backend == 'always':
            labels = self.targets.get(request.target)
            result = (Verdict(True, 'stub', DetectionRecord(sorted(labels)[0], 1., .5, .5, 0., 0.),
                              self.get_clock().now().nanoseconds)
                      if labels else Verdict(detail='unknown_target'))
        else:
            evidence = {} if self._diag.enabled else None
            result = self._window.check(request.target, time.monotonic(),
                                        request.min_confidence, request.max_age_s, evidence)
            if evidence is not None:
                self._diag.emit('check', target=request.target, **evidence)
        return fill_response(response, result)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
