"""detector_node — YOLO object presence for the orchestrator's precheck.

Think of it as a free-running ADC: it samples the camera continuously and keeps
a short window of results, so a "is X visible?" question is answered from the
window in a few ms instead of starting an inference on demand and flickering on
a single frame.

    camera (CompressedImage | Image) ──> YOLO @ rate_hz ──> ring buffer (window_s)
                                                          ├─> DetectionArray  (/cortex/detections, debug/GUI)
                                                          └─> CheckTarget srv (/cortex/detector/check)

CheckTarget rule: `target` (vocabulary key: fridge_door, cucumber, user, ...) maps
to YOLO class names via the `targets` parameter. found = the class appears with
conf ≥ min_confidence in a MAJORITY of the frames inside the window. No frames,
or only stale frames → found=false with detail "no_frame" / "stale" — the
orchestrator treats those as "no verdict" (fail-open), unlike a real "not found".

Backend: ultralytics (YOLO26s by default). Imported lazily; without it the node
still runs and answers detail="model_not_loaded" so the rest of the stack works.

COCO caveat: 'refrigerator' and 'person' exist in the pretrained classes;
'cucumber' does not — that target needs a fine-tuned weight file (parameter `model`).
"""

import collections
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from cortex_msgs.msg import Detection, DetectionArray
from cortex_msgs.srv import CheckTarget


class DetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('detector_node')
        self.declare_parameter('camera_topic', '/bridge/sensors/camera/color/compressed')
        self.declare_parameter('camera_transport', 'compressed')   # compressed | raw
        self.declare_parameter('detections_topic', '/cortex/detections')
        self.declare_parameter('service', '/cortex/detector/check')
        self.declare_parameter('model', 'yolo26s.pt')
        self.declare_parameter('device', '')          # '' = ultralytics default (cuda if available)
        self.declare_parameter('rate_hz', 8.0)
        self.declare_parameter('window_s', 0.6)
        self.declare_parameter('default_min_confidence', 0.4)
        self.declare_parameter('infer_conf', 0.25)   # NMS-free threshold passed to predict()
        self.declare_parameter('imgsz', 640)
        # target -> YOLO class names. Declared as flat string lists (rclpy has no nested dicts).
        self.declare_parameter('target_keys', ['fridge_door', 'fridge', 'cucumber', 'user'])
        self.declare_parameter('target_classes', ['refrigerator', 'refrigerator', 'cucumber', 'person'])

        g = self.get_parameter
        self.window_s = float(g('window_s').value)
        self.min_conf = float(g('default_min_confidence').value)
        self.infer_conf = float(g('infer_conf').value)
        self.imgsz = int(g('imgsz').value)
        keys, classes = list(g('target_keys').value), list(g('target_classes').value)
        self.targets = {k: set(c.split('|')) for k, c in zip(keys, classes)}
        self.model_name = str(g('model').value)

        self._lock = threading.Lock()
        self._frame = None                     # latest decoded BGR frame
        self._frame_t = 0.0
        self._window = collections.deque()     # (t, {label: max_conf})
        self._model = None
        self._names = {}
        self._load_model(str(g('device').value))

        grp = ReentrantCallbackGroup()
        self.det_pub = self.create_publisher(DetectionArray, g('detections_topic').value, 10)
        if g('camera_transport').value == 'raw':
            self.create_subscription(Image, g('camera_topic').value, self._on_raw, 1, callback_group=grp)
        else:
            self.create_subscription(CompressedImage, g('camera_topic').value, self._on_compressed, 1,
                                     callback_group=grp)
        self.create_service(CheckTarget, g('service').value, self._on_check, callback_group=grp)
        self.create_timer(1.0 / float(g('rate_hz').value), self._infer, callback_group=grp)
        self.get_logger().info(
            f'detector_node up (model={self.model_name}, loaded={self._model is not None}, '
            f'targets={ {k: sorted(v) for k, v in self.targets.items()} })')

    # --- model ------------------------------------------------------------
    def _load_model(self, device: str) -> None:
        try:
            from ultralytics import YOLO
            self._model = YOLO(self.model_name)
            if device:
                self._model.to(device)
            self._names = dict(self._model.names)
        except Exception as e:                 # missing package / weights: keep serving "model_not_loaded"
            self.get_logger().error(f'YOLO not loaded ({e}); CheckTarget will answer model_not_loaded')
            self._model = None

    # --- camera -----------------------------------------------------------
    def _on_compressed(self, m: CompressedImage) -> None:
        try:
            import cv2
            buf = np.frombuffer(bytes(m.data), dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        except Exception:
            return
        if frame is not None:
            with self._lock:
                self._frame, self._frame_t = frame, time.monotonic()

    def _on_raw(self, m: Image) -> None:
        if m.encoding not in ('bgr8', 'rgb8'):
            return
        arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, 3)
        if m.encoding == 'rgb8':
            arr = arr[:, :, ::-1]
        with self._lock:
            self._frame, self._frame_t = arr.copy(), time.monotonic()

    # --- inference loop ---------------------------------------------------
    def _infer(self) -> None:
        with self._lock:
            frame, t = self._frame, self._frame_t
        if frame is None or self._model is None:
            return
        if self._window and self._window[-1][0] >= t:      # same frame as last time
            return
        t0 = time.perf_counter()
        try:
            r = self._model.predict(frame, conf=self.infer_conf, imgsz=self.imgsz, verbose=False)[0]
        except Exception as e:
            self.get_logger().warning(f'predict failed: {e}')
            return
        ms = (time.perf_counter() - t0) * 1000.0
        h, w = frame.shape[:2]
        best: dict = {}
        out = DetectionArray(model=self.model_name, infer_ms=float(ms))
        out.header.stamp = self.get_clock().now().to_msg()
        if r.boxes is not None:
            for cls, conf, xyxy in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist(), r.boxes.xyxy.tolist()):
                label = str(self._names.get(int(cls), int(cls)))
                x1, y1, x2, y2 = xyxy
                d = Detection(label=label, confidence=float(conf),
                              cx=float((x1 + x2) / 2 / w), cy=float((y1 + y2) / 2 / h),
                              w=float((x2 - x1) / w), h=float((y2 - y1) / h))
                out.detections.append(d)
                if conf > best.get(label, (0.0, None))[0]:
                    best[label] = (float(conf), d)
        with self._lock:
            self._window.append((t, best))
            cutoff = time.monotonic() - self.window_s
            while self._window and self._window[0][0] < cutoff:
                self._window.popleft()
        self.det_pub.publish(out)

    # --- service ----------------------------------------------------------
    def _on_check(self, req: CheckTarget.Request, res: CheckTarget.Response) -> CheckTarget.Response:
        res.found = False
        if self._model is None:
            res.detail = 'model_not_loaded'
            return res
        classes = self.targets.get(req.target)
        if classes is None:
            res.detail = 'unknown_target'
            return res
        min_conf = req.min_confidence or self.min_conf
        max_age = req.max_age_s or self.window_s
        now = time.monotonic()
        with self._lock:
            frames = [(t, best) for t, best in self._window if now - t <= max_age]
            have_any = bool(self._window)
        if not frames:
            res.detail = 'stale' if have_any else 'no_frame'
            return res
        hits = []
        for t, best in frames:
            for label in classes:
                if label in best and best[label][0] >= min_conf:
                    hits.append((best[label][0], best[label][1], t))
                    break
        if len(hits) * 2 <= len(frames):                   # not a majority
            res.detail = ''
            return res
        conf, d, t = max(hits, key=lambda h: h[0])
        res.found, res.confidence, res.label = True, float(conf), d.label
        res.cx, res.cy, res.w, res.h = d.cx, d.cy, d.w, d.h
        res.stamp = self.get_clock().now().to_msg()
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectorNode()
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
