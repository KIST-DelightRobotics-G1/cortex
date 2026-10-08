"""ROS-free YOLO adapter and frame-window presence policy [SYS-REQ-44].

The ROS node and offline replay use this same implementation. A positive verdict
means class presence only; it does not establish door state or graspability.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import threading


@dataclass(frozen=True)
class DetectionRecord:
    label: str
    confidence: float
    cx: float
    cy: float
    w: float
    h: float


@dataclass(frozen=True)
class Verdict:
    found: bool = False
    detail: str = ''
    detection: DetectionRecord | None = None
    stamp_ns: int = 0
    samples: int = 0
    hits: int = 0


def target_mapping(keys, classes):
    if len(keys) != len(classes) or len(set(keys)) != len(keys):
        raise ValueError('target_keys and target_classes must be equal-length, unique-key lists')
    result = {k: frozenset(c.split('|')) for k, c in zip(keys, classes)}
    if any(not k or not c or '' in c for k, c in result.items()):
        raise ValueError('empty target or class')
    return result


def decode_boxes(rows, names, width, height):
    """Ultralytics xyxy/conf/cls rows -> the existing normalized ROS contract."""
    if width <= 0 or height <= 0:
        raise ValueError('invalid image dimensions')
    output = []
    for row in rows:
        if len(row) != 6 or not all(math.isfinite(float(v)) for v in row):
            raise ValueError('invalid detection row')
        x1, y1, x2, y2, confidence, cls = map(float, row)
        if cls != int(cls) or int(cls) not in names or not 0 <= confidence <= 1:
            raise ValueError('invalid class or confidence')
        x1, x2 = max(0., min(width, x1)), max(0., min(width, x2))
        y1, y2 = max(0., min(height, y1)), max(0., min(height, y2))
        if x2 <= x1 or y2 <= y1:
            continue
        output.append(DetectionRecord(str(names[int(cls)]), confidence,
                                      (x1+x2)/2/width, (y1+y2)/2/height,
                                      (x2-x1)/width, (y2-y1)/height))
    return tuple(output)


def frame_time(stamp_ns, ros_now_ns, received_t, max_age_s, allow_unstamped=False):
    """Map source ROS time to local monotonic age; reject stale/future images.

    ROS source/receiver clocks must be synchronized (or share /clock). Source
    age counts towards the window, including decode and inference time.
    """
    if stamp_ns <= 0:
        if not allow_unstamped:
            return None
        stamp_ns = ros_now_ns
    age = (ros_now_ns-stamp_ns)/1e9
    if age < -0.1 or age > max_age_s:
        return None
    return received_t-max(0., age), stamp_ns


class PresenceWindow:
    def __init__(self, targets, names, window_s=.6, min_confidence=.4, min_frames=3, require_latest_hit=False):
        if (not math.isfinite(window_s) or window_s <= 0 or
                not math.isfinite(min_confidence) or not 0 < min_confidence <= 1 or
                not isinstance(min_frames, int) or min_frames < 1):
            raise ValueError('invalid presence window settings')
        self.targets, self.names = targets, frozenset(names)
        self.window_s, self.min_confidence, self.min_frames = window_s, min_confidence, min_frames
        self.require_latest_hit = require_latest_hit
        self._frames = deque(maxlen=1024)
        self._last_stamp = -1
        self._ever_frame = False
        self._fault = ''
        self._lock = threading.Lock()

    def _prune(self, now):
        while self._frames and now-self._frames[0][0] > self.window_s:
            self._frames.popleft()

    def invalidate(self, detail):
        with self._lock:
            self._frames.clear()
            self._fault = detail

    def add(self, detections, observed_t, stamp_ns, now):
        with self._lock:
            self._prune(now)
            if stamp_ns <= self._last_stamp or not 0 <= now-observed_t <= self.window_s:
                return False
            best = {}
            for d in detections:
                if d.label not in best or d.confidence > best[d.label].confidence:
                    best[d.label] = d
            self._frames.append((observed_t, stamp_ns, best))
            self._last_stamp, self._ever_frame, self._fault = stamp_ns, True, ''
            return True

    def check(self, target, now, min_confidence=0., max_age_s=0., diagnostics=None):
        # Optional evidence is captured under the same lock as the verdict.
        # Diagnostic reasons never enter CheckTarget.detail (which has fail-open semantics).
        def finish(verdict, reason=None, **evidence):
            if diagnostics is not None:
                diagnostics.update(found=verdict.found, detail=verdict.detail,
                                   reason=reason or verdict.detail or 'allowed', **evidence)
            return verdict

        if diagnostics is not None:
            diagnostics.clear()
        labels = self.targets.get(target)
        if labels is None:
            return finish(Verdict(detail='unknown_target'))
        if not (math.isfinite(min_confidence) and 0 <= min_confidence <= 1 and
                math.isfinite(max_age_s) and max_age_s >= 0):
            return finish(Verdict(detail='invalid_request'))
        with self._lock:
            if self._fault:
                return finish(Verdict(detail=self._fault))
            labels = labels & self.names
            if not labels:
                return finish(Verdict(detail='unsupported_class'))
            self._prune(now)
            age = min(max_age_s or self.window_s, self.window_s)
            frames = [r for r in self._frames if 0 <= now-r[0] <= age]
            if not frames:
                return finish(Verdict(detail='stale' if self._ever_frame else 'no_frame'))
            conf = min_confidence or self.min_confidence
            hits = []
            for _, stamp, best in frames:
                ds = [best[label] for label in labels if label in best and best[label].confidence >= conf]
                if ds:
                    hits.append((max(ds, key=lambda d: d.confidence), stamp))
            latest_hit = bool(hits) and hits[-1][1] == frames[-1][1]
            if diagnostics is not None:
                scores = [d.confidence for _, _, best in frames
                          for label, d in best.items() if label in labels]
                last_hit_t = next((t for t, stamp, _ in reversed(frames)
                                   if hits and stamp == hits[-1][1]), None)
                diagnostics.update(samples=len(frames), hits=len(hits), latest_hit=latest_hit,
                                   min_frames=self.min_frames, confidence_threshold=conf,
                                   window_s=age, require_latest_hit=self.require_latest_hit,
                                   latest_frame_age_ms=(now-frames[-1][0])*1000,
                                   last_hit_age_ms=(now-last_hit_t)*1000 if last_hit_t is not None else None,
                                   max_candidate_confidence=max(scores) if scores else None)
            if len(frames) < self.min_frames:
                return finish(Verdict(detail='insufficient_frames', samples=len(frames)))
            if len(hits)*2 <= len(frames) or (self.require_latest_hit and not latest_hit):
                reason = ('no_hits' if not hits else 'below_majority'
                          if len(hits)*2 <= len(frames) else 'latest_miss')
                return finish(Verdict(samples=len(frames), hits=len(hits)), reason)
            d, stamp = max(hits, key=lambda item: item[0].confidence)
            return finish(Verdict(True, detection=d, stamp_ns=stamp, samples=len(frames), hits=len(hits)))


class YoloDetector:
    def __init__(self, model, device='', imgsz=640, infer_conf=.25, expected_sha256=''):
        # No implicit model download: deployment must explicitly supply a local weight.
        path = Path(model).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'YOLO weight not found: {path}')
        if expected_sha256 and hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha256:
            raise ValueError('YOLO weight SHA-256 does not match the selected profile')
        from ultralytics import YOLO
        self.model = YOLO(str(path))
        self.names = dict(self.model.names)
        self.device, self.imgsz, self.infer_conf = device, imgsz, infer_conf
        if self.model.task != 'detect':
            raise ValueError('detector_node requires a detection model')

    def predict(self, frame):
        kwargs = dict(conf=self.infer_conf, imgsz=self.imgsz, verbose=False)
        if self.device:
            kwargs['device'] = self.device
        result = self.model.predict(frame, **kwargs)[0]
        rows = result.boxes.data.cpu().tolist() if result.boxes is not None else []
        return decode_boxes(rows, self.names, frame.shape[1], frame.shape[0])


def fill_detection(message, detection):
    for key in ('label', 'confidence', 'cx', 'cy', 'w', 'h'):
        setattr(message, key, getattr(detection, key))
    return message


def fill_response(response, verdict):
    response.found, response.detail = verdict.found, verdict.detail
    response.confidence = 0.
    response.label = ''
    response.cx = response.cy = response.w = response.h = 0.
    response.stamp.sec, response.stamp.nanosec = divmod(verdict.stamp_ns, 1_000_000_000)
    if verdict.detection:
        fill_detection(response, verdict.detection)
    return response
