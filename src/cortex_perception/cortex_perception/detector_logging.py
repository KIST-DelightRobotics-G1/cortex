"""Bounded, ROS-free detector counters for periodic console diagnostics."""
from collections import Counter
import threading
import time


class DetectorLogStats:
    def __init__(self, enabled, clock=time.monotonic):
        self.enabled, self._clock = enabled, clock
        self._counts, self._previous = Counter(), Counter()
        self._last, self._times = {}, {}
        self._sampled_at = clock()
        self._lock = threading.Lock()

    def record(self, event, **fields):
        if not self.enabled:
            return
        with self._lock:
            self._counts[event] += 1
            self._times[event] = self._clock()
            self._last.update(fields)

    def snapshot(self):
        with self._lock:
            now = self._clock()
            elapsed = max(now-self._sampled_at, 1e-9)
            rates = {key: round((self._counts[key]-self._previous[key])/elapsed, 2)
                     for key in ('received', 'frame_accepted', 'inference_completed')}
            ages = {key+'_age_ms': round(1000*(now-self._times[key]), 1)
                    if key in self._times else None
                    for key in ('received', 'frame_accepted', 'inference_completed')}
            last = dict(self._last)
            observed = last.pop('source_observed_t', None)
            last['latest_frame_age_ms'] = round(1000*(now-observed), 1) if observed is not None else None
            result = dict(interval_s=round(elapsed, 3), rates_hz=rates,
                          counters=dict(self._counts), **ages, **last)
            self._previous, self._sampled_at = self._counts.copy(), now
            return result
