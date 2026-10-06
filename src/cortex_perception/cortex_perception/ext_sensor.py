"""ext_sensor — turn ext-sensor-io payloads into what the cortex nodes consume.

ext-sensor-io publishes device-native data (kist_msgs/AudioChunk: interleaved
PCM at the mic's own rate and channel count; kist_msgs/CompressedColorFrame:
an Annex-B H.264 stream). It stays generic on purpose, so the conversion lives
here, next to the consumers.

    chunk_to_mono_s16(...)   AudioChunk payload -> one channel, S16_LE, target rate
    level_dbfs(pcm)          S16_LE samples -> (rms, peak) in dBFS
    MicMonitor               what the mic path did since the last summary (pure, no rclpy)
    H264Decoder              CompressedColorFrame stream -> decoded frames (PyAV)
"""

from __future__ import annotations

import math
import threading

import numpy as np


def chunk_to_mono_s16(data, channels: int, channel: int, rate: int,
                      target_rate: int) -> bytes:
    """Interleaved S16_LE PCM -> the samples of one channel at target_rate.

    The reSpeaker XVF3800 array carries its processed (beamformed, echo-reduced)
    signal on channel 0 and the raw capsules after it, so the default is to take
    channel 0 rather than to average — averaging would mix raw noise back in.
    A partial trailing frame is dropped.
    """
    x = np.frombuffer(data, dtype='<i2')
    if channels > 1:
        x = x[: len(x) - len(x) % channels].reshape(-1, channels)[:, channel]
    if rate != target_rate and len(x):
        from scipy.signal import resample_poly
        g = math.gcd(rate, target_rate)
        y = resample_poly(x.astype(np.float32), target_rate // g, rate // g)
        x = np.clip(np.rint(y), -32768, 32767)
    return np.ascontiguousarray(x, dtype='<i2').tobytes()


SILENCE_DBFS = -120.0                 # what an all-zero chunk reports


def level_dbfs(pcm: bytes) -> tuple:
    """S16_LE samples -> (rms dBFS, peak dBFS). Full scale (32768) is 0 dBFS."""
    x = np.frombuffer(pcm, dtype='<i2').astype(np.float64)
    if not len(x):
        return SILENCE_DBFS, SILENCE_DBFS

    def db(v: float) -> float:
        return 20.0 * math.log10(v / 32768.0) if v > 0 else SILENCE_DBFS
    return db(float(np.sqrt(np.mean(x * x)))), db(float(np.max(np.abs(x))))


class MicMonitor:
    """Counts what the mic path did, so a delay can be placed on the timeline.

    stt_node feeds every AudioChunk through on_chunk() and the gate's verdict
    through on_outcome() ('sent' to the recognizer, 'muted' by the echo gate,
    'dropped' because the recognizer queue was full). summary() returns the
    counters since the previous call and resets them. check_stall() says when
    chunks stop arriving and on_chunk() when they come back.

    Thread-safe: rclpy may run the audio callback and the timers concurrently.
    """

    def __init__(self, stall_s: float = 1.0) -> None:
        self.stall_s = stall_s
        self._lock = threading.Lock()
        self._last_seq = None
        self._last_t = None
        self._stalled = False
        self.total = 0
        self._reset(None)

    def _reset(self, now) -> None:
        self._t0 = now
        self._n = self._gaps = 0
        self._out = {'sent': 0, 'muted': 0, 'dropped': 0}
        self._rms = []
        self._peak = SILENCE_DBFS
        self._ages = []

    def on_chunk(self, seq: int, now: float, age_ms, rms_db: float, peak_db: float) -> dict:
        """Record one chunk. Returns events: first (bool), gap (missing seqs before
        this one), resumed_after_s (seconds of silence if chunks had stalled)."""
        with self._lock:
            ev = {'first': self.total == 0, 'gap': 0, 'resumed_after_s': None}
            if self._last_seq is not None and seq > self._last_seq + 1:
                ev['gap'] = seq - self._last_seq - 1
                self._gaps += ev['gap']
            if self._stalled:
                ev['resumed_after_s'] = now - self._last_t
                self._stalled = False
            if self._t0 is None:
                self._t0 = now
            self._last_seq, self._last_t = seq, now
            self.total += 1
            self._n += 1
            self._rms.append(rms_db)
            self._peak = max(self._peak, peak_db)
            if age_ms is not None:
                self._ages.append(age_ms)
            return ev

    def on_outcome(self, outcome: str) -> None:
        with self._lock:
            self._out[outcome] = self._out.get(outcome, 0) + 1

    def check_stall(self, now: float) -> bool:
        """True exactly once when no chunk has come for stall_s (after the first)."""
        with self._lock:
            if self._stalled or self._last_t is None or now - self._last_t < self.stall_s:
                return False
            self._stalled = True
            return True

    def summary(self, now: float) -> dict:
        """Counters since the last call (then reset). rate is chunks/s over the window."""
        with self._lock:
            span = (now - self._t0) if self._t0 is not None else 0.0
            ages = sorted(self._ages)
            s = {
                'chunks': self._n,
                'span_s': span,
                'rate': self._n / span if span > 0 else 0.0,
                'gaps': self._gaps,
                'rms_db': sum(self._rms) / len(self._rms) if self._rms else SILENCE_DBFS,
                'peak_db': self._peak,
                'age_ms_p50': ages[len(ages) // 2] if ages else None,
                'age_ms_max': ages[-1] if ages else None,
                **self._out,
            }
            self._reset(now)
            return s


class H264Decoder:
    """Stateful H.264 decoder for one camera stream.

    H.264 frames depend on earlier ones, so every frame is decoded even when the
    consumer only wants a few. Decoding starts at the first keyframe (which
    carries SPS/PPS); after an error the decoder drops state and waits for the
    next keyframe instead of emitting corrupted images.
    """

    def __init__(self) -> None:
        self._ctx = None

    def decode(self, data, keyframe: bool, stamp_ns: int | None = None):
        """Feed one frame's NAL units; returns the newest av.VideoFrame or None."""
        if self._ctx is None and not keyframe:
            return None
        import av                                   # lazy: only camera consumers need PyAV
        if self._ctx is None:
            self._ctx = av.CodecContext.create('h264', 'r')
        try:
            packet = av.Packet(bytes(data))
            if stamp_ns is not None:
                from fractions import Fraction
                packet.pts = int(stamp_ns)
                packet.time_base = Fraction(1, 1_000_000_000)
            frames = self._ctx.decode(packet)
        except Exception:                           # corrupt / out-of-order: resync on next keyframe
            self._ctx = None
            return None
        return frames[-1] if frames else None


def decoded_stamp_ns(frame):
    """Return the decoded frame's source PTS; missing timing is not freshness."""
    if frame.pts is None or frame.time_base is None:
        return None
    return int(frame.pts * frame.time_base * 1_000_000_000)
