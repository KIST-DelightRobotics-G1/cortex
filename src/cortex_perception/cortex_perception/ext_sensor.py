"""ext_sensor — turn ext-sensor-io payloads into what the cortex nodes consume.

ext-sensor-io publishes device-native data (kist_msgs/AudioChunk: interleaved
PCM at the mic's own rate and channel count; kist_msgs/CompressedColorFrame:
an Annex-B H.264 stream). It stays generic on purpose, so the conversion lives
here, next to the consumers.

    chunk_to_mono_s16(...)   AudioChunk payload -> one channel, S16_LE, target rate
    H264Decoder              CompressedColorFrame stream -> decoded frames (PyAV)
"""

from __future__ import annotations

import math

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


class H264Decoder:
    """Stateful H.264 decoder for one camera stream.

    H.264 frames depend on earlier ones, so every frame is decoded even when the
    consumer only wants a few. Decoding starts at the first keyframe (which
    carries SPS/PPS); after an error the decoder drops state and waits for the
    next keyframe instead of emitting corrupted images.
    """

    def __init__(self) -> None:
        self._ctx = None

    def decode(self, data, keyframe: bool):
        """Feed one frame's NAL units; returns the newest av.VideoFrame or None."""
        if self._ctx is None and not keyframe:
            return None
        import av                                   # lazy: only camera consumers need PyAV
        if self._ctx is None:
            self._ctx = av.CodecContext.create('h264', 'r')
        try:
            frames = self._ctx.decode(av.Packet(bytes(data)))
        except Exception:                           # corrupt / out-of-order: resync on next keyframe
            self._ctx = None
            return None
        return frames[-1] if frames else None
