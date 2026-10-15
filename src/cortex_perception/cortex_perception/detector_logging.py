"""Bounded, ROS-free detector pipeline counters, attached to each YOLO_CHECK."""
from collections import Counter, deque
import threading
import time

RATE_WINDOW_S = 2.0
# Frames that made it through; every other recorded event is a drop and is counted.
_PASSED = ('received', 'frame_accepted', 'inference_completed', 'window_added')


class DetectorLogStats:
    def __init__(self, enabled, clock=time.monotonic):
        self.enabled, self._clock = enabled, clock
        self._counts = Counter()
        self._recent = {key: deque() for key in ('frame_accepted', 'inference_completed')}
        self._last = {}
        self._lock = threading.Lock()

    def record(self, event, **fields):
        if not self.enabled:
            return
        with self._lock:
            now = self._clock()
            self._counts[event] += 1
            if event in self._recent:
                self._recent[event].append(now)
                self._trim(now)
            self._last.update(fields)

    def _trim(self, now):
        for times in self._recent.values():
            while times and now-times[0] > RATE_WINDOW_S:
                times.popleft()

    def pipeline(self):
        """Camera and inference health at the moment of a check: rates over the last
        RATE_WINDOW_S, the last inference, and how many frames were dropped since start."""
        with self._lock:
            self._trim(self._clock())
            return dict(
                camera_fps=round(len(self._recent['frame_accepted'])/RATE_WINDOW_S, 1),
                infer_hz=round(len(self._recent['inference_completed'])/RATE_WINDOW_S, 1),
                infer_ms=self._last.get('infer_ms'),
                best_confidence=self._last.get('best_confidence', {}),
                dropped={k: v for k, v in sorted(self._counts.items()) if k not in _PASSED})
