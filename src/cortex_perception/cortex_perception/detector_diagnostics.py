"""Optional bounded diagnostics; no ROS, images, file I/O or decision changes."""
from collections import Counter
import json
import threading
import time


class DetectorDiagnostics:
    def __init__(self, publish=None, clock_ns=time.time_ns):
        self.publish, self.clock_ns = publish, clock_ns
        self._lock = threading.Lock()
        self._counts, self._last, self._previous = Counter(), {}, Counter()
        self._at = time.monotonic()

    @property
    def enabled(self):
        return self.publish is not None

    def record(self, event, **fields):
        if self.enabled:
            with self._lock:
                self._counts[event] += 1
                self._last[event] = dict(monotonic_s=time.monotonic(), **fields)

    def emit(self, event, **fields):
        if not self.enabled:
            return
        try:
            self.publish(json.dumps(dict(schema=1, event=event, ros_time_ns=self.clock_ns(),
                                         monotonic_s=time.monotonic(), **fields), allow_nan=False))
        except Exception:
            # Diagnostic output failures must not alter a CheckTarget response.
            with self._lock:
                self._counts['diagnostics_publish_error'] += 1

    def heartbeat(self, **fields):
        if not self.enabled:
            return
        now = time.monotonic()
        with self._lock:
            elapsed = max(now-self._at, 1e-9)
            rates = {key: (value-self._previous[key])/elapsed for key, value in self._counts.items()}
            snapshot = dict(counters=dict(self._counts), last=dict(self._last), rates_hz=rates)
            self._previous, self._at = self._counts.copy(), now
        self.emit('heartbeat', **snapshot, **fields)
