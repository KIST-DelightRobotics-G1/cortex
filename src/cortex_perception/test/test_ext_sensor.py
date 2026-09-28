# -*- coding: utf-8 -*-
import numpy as np

from cortex_perception.ext_sensor import H264Decoder, chunk_to_mono_s16


def _interleave(chans):
    return np.stack(chans, axis=1).astype('<i2').tobytes()


def test_picks_channel_zero_of_six():
    ch = [np.full(1600, 100 * (k + 1), dtype=np.int16) for k in range(6)]
    out = np.frombuffer(chunk_to_mono_s16(_interleave(ch), 6, 0, 16000, 16000), '<i2')
    assert len(out) == 1600 and (out == 100).all()


def test_picks_another_channel():
    ch = [np.arange(10, dtype=np.int16) + 1000 * k for k in range(2)]
    out = np.frombuffer(chunk_to_mono_s16(_interleave(ch), 2, 1, 16000, 16000), '<i2')
    assert list(out) == list(range(1000, 1010))


def test_mono_passes_through():
    x = np.arange(-5, 5, dtype=np.int16)
    assert chunk_to_mono_s16(x.astype('<i2').tobytes(), 1, 0, 16000, 16000) == x.astype('<i2').tobytes()


def test_partial_trailing_frame_is_dropped():
    data = _interleave([np.ones(4, np.int16)] * 2) + b'\x01\x00'   # one stray sample
    assert len(chunk_to_mono_s16(data, 2, 0, 16000, 16000)) == 4 * 2


def test_resamples_48k_to_16k():
    t = np.arange(4800) / 48000.0                                    # 100 ms of 440 Hz
    x = (8000 * np.sin(2 * np.pi * 440 * t)).astype(np.int16)
    out = np.frombuffer(chunk_to_mono_s16(_interleave([x, x]), 2, 0, 48000, 16000), '<i2')
    assert len(out) == 1600
    assert 6000 < np.abs(out[200:-200]).max() < 9000                   # tone survives, no clipping


def test_decoder_waits_for_a_keyframe():
    assert H264Decoder().decode(b'\x00\x00\x00\x01\x41', keyframe=False) is None
