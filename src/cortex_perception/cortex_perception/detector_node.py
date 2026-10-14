"""Camera -> YOLO -> DetectionArray and buffered CheckTarget [SYS-REQ-44].

The ROS-free adapter/policy lives in detection.py and is shared by offline tests.
The default 'always' backend is an explicit demo stub. Use the real-detector
launch profile with a local .pt weight for fridge/cucumber integration.
"""
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from kist_msgs.msg import CompressedColorFrame

from cortex_msgs.msg import Detection, DetectionArray
from cortex_msgs.srv import CheckTarget
from .detector_logging import DetectorLogStats
from .detection import (DetectionRecord, PresenceWindow, Verdict, YoloDetector,
                        fill_detection, fill_response, frame_time, target_mapping)


class DetectorNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('detector_node', **kwargs)
        defaults = {
            'camera_topic': '/kist/camera/head/color/h264',
            'camera_transport': 'h264', 'detections_topic': '/cortex/detections',
            'service': '/cortex/detector/check', 'backend': 'always',
            'model': 'yolo26s.pt', 'model_sha256': '', 'device': '', 'rate_hz': 8.0, 'window_s': 0.6,
            'default_min_confidence': 0.4, 'infer_conf': 0.25, 'imgsz': 640,
            'min_frames': 3, 'allow_unstamped': False, 'require_latest_hit': False,
            'diagnostics_log_enabled': False, 'diagnostics_log_period_s': 2.0,
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
        self._log_enabled = bool(g('diagnostics_log_enabled'))
        log_period = float(g('diagnostics_log_period_s'))
        if self._log_enabled and (not math.isfinite(log_period) or log_period < 0.5):
            raise ValueError('diagnostics_log_period_s must be finite and >= 0.5')
        self._stats = DetectorLogStats(self._log_enabled)
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
        self.get_logger().info(f'detector backend={self.backend}, model={self.model_name}, '
                               f'loaded={self._model is not None}, min_frames={g("min_frames")}')
        self._log('YOLO_CONFIG', backend=self.backend, model=self.model_name,
                  loaded=self._model is not None, configured_device=g('device'),
                  expected_model_sha256=g('model_sha256'),
                  camera_topic=g('camera_topic'), camera_transport=g('camera_transport'),
                  rate_hz=rate, infer_conf=infer_conf, imgsz=g('imgsz'), window_s=self.window_s,
                  min_frames=g('min_frames'), min_confidence=g('default_min_confidence'),
                  require_latest_hit=g('require_latest_hit'), allow_unstamped=self.allow_unstamped,
                  classes=dict(self._model.names) if self._model else {},
                  targets={key: sorted(labels) for key, labels in self.targets.items()})
        if self._log_enabled and self.backend == 'yolo':
            self.create_timer(log_period, self._log_status, callback_group=self._io_group)

    def _log(self, event, **fields):
        if self._log_enabled:
            self.get_logger().info(event + ' ' + json.dumps(fields, ensure_ascii=False))

    def _log_status(self):
        self._log('YOLO_STATUS', **self._stats.snapshot())

    def _accept_frame(self, frame, header):
        stamp_ns = header.stamp.sec*1_000_000_000 + header.stamp.nanosec
        self._accept_frame_data(frame, stamp_ns, header.frame_id)

    def _accept_frame_data(self, frame, stamp_ns, frame_id):
        ros_now = self.get_clock().now().nanoseconds
        timing = frame_time(stamp_ns, ros_now,
                            time.monotonic(), self.window_s, self.allow_unstamped)
        if timing is None:
            age_ms = (ros_now-stamp_ns)/1e6
            reason = 'unstamped' if stamp_ns <= 0 else 'future_stamp' if age_ms < -100 else 'stale_input'
            self._stats.record(reason, rejected_source_age_ms=round(age_ms, 1))
            return
        observed_t, stamp_ns = timing
        with self._lock:
            if self._frame is not None and stamp_ns <= self._frame[2]:
                self._stats.record('out_of_order')
                return
            self._frame = (frame, observed_t, stamp_ns, frame_id)
            self._stats.record('frame_accepted', source_stamp_ns=stamp_ns, source_observed_t=observed_t)

    def _on_h264(self, message):
        from .ext_sensor import decoded_stamp_ns
        # Decode every packet to retain codec reference state. The decoded frame
        # may belong to an earlier packet; never stamp it with the newest packet.
        self._stats.record('received')
        frame = self._decoder.decode(message.data, message.is_keyframe, message.stamp_ns)
        if frame is not None:
            stamp_ns = decoded_stamp_ns(frame)
            if stamp_ns is not None:
                self._accept_frame_data(frame.to_ndarray(format='bgr24'), stamp_ns, message.frame_id)
            else:
                self._stats.record('decoded_unstamped')
        else:
            # None also means keyframe wait or codec buffering, not necessarily corrupt input.
            self._stats.record('h264_no_output')

    def _on_compressed(self, message):
        self._stats.record('received')
        try:
            import cv2
            frame = cv2.imdecode(np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        except Exception:
            self._stats.record('compressed_decode_error')
            return
        if frame is not None:
            self._accept_frame(frame, message.header)
        else:
            self._stats.record('compressed_decode_error')

    def _on_raw(self, message):
        self._stats.record('received')
        if message.encoding not in ('bgr8', 'rgb8') or message.height <= 0 or message.width <= 0:
            self._stats.record('invalid_raw_frame')
            return
        if message.step < message.width*3 or len(message.data) < message.height*message.step:
            self._stats.record('invalid_raw_frame')
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
            self._stats.record('stale_before_inference')
            return
        started = time.perf_counter()
        try:
            detections = self._model.predict(frame)
        except Exception as error:
            self._window.invalidate('inference_error')
            self._stats.record('inference_error', last_error=str(error))
            self.get_logger().warning(f'YOLO predict failed: {error}')
            return
        ms = (time.perf_counter()-started)*1000
        completed_t = time.monotonic()
        added = self._window.add(detections, observed_t, stamp_ns, completed_t)
        if self._log_enabled:
            best = {}
            for d in detections:
                best[d.label] = round(max(best.get(d.label, 0.), d.confidence), 4)
            self._stats.record('inference_completed', infer_ms=round(ms, 2),
                               infer_source_age_ms=round(1000*(completed_t-observed_t), 1),
                               best_confidence=best)
        if not added:
            self._stats.record('window_add_rejected')
            return
        self._stats.record('window_added')
        out = DetectionArray(model=self.model_name, infer_ms=float(ms))
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        out.header.frame_id = frame_id
        out.detections = [fill_detection(Detection(), d) for d in detections]
        self.det_pub.publish(out)

    def _on_check(self, request, response):
        diagnostic = {} if self._log_enabled else None
        if self.backend == 'always':
            labels = self.targets.get(request.target)
            result = (Verdict(True, 'stub', DetectionRecord(sorted(labels)[0], 1., .5, .5, 0., 0.),
                              self.get_clock().now().nanoseconds)
                      if labels else Verdict(detail='unknown_target'))
            if diagnostic is not None:
                diagnostic.update(found=result.found, reason=result.detail, detail=result.detail)
        else:
            result = self._window.check(request.target, time.monotonic(),
                                        request.min_confidence, request.max_age_s, diagnostics=diagnostic)
        if diagnostic is not None:
            self._log('YOLO_CHECK', target=request.target, **diagnostic)
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
