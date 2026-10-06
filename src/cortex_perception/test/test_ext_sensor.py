# -*- coding: utf-8 -*-
import numpy as np

from cortex_perception.ext_sensor import (SILENCE_DBFS, H264Decoder, MicMonitor,
                                          chunk_to_mono_s16, level_dbfs)


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


def _h264_packets(bframes=0):
    import av
    from fractions import Fraction
    encoder = av.CodecContext.create('libx264', 'w')
    encoder.width, encoder.height = 80, 32
    encoder.pix_fmt = 'yuv420p'
    encoder.time_base = Fraction(1, 30)
    encoder.options = {'preset': 'ultrafast', 'crf': '18',
                       'x264-params': f'bframes={bframes}:b-adapt=0:keyint=8:scenecut=0'}
    packets = []
    for i in range(24):
        # Encode the frame index in five large binary tiles. Small brightness
        # differences are not stable across lossy H.264 / RGB-YUV conversions.
        pixels = np.empty((32, 80, 3), np.uint8)
        for bit in range(5):
            pixels[:, bit*16:(bit+1)*16] = 224 if i & (1 << bit) else 32
        frame = av.VideoFrame.from_ndarray(pixels, format='bgr24')
        frame.pts = i
        packets.extend(encoder.encode(frame))
    return packets + encoder.encode(None)


def test_decoder_preserves_source_pts_when_h264_reorders_frames():
    from cortex_perception.ext_sensor import decoded_stamp_ns
    decoder = H264Decoder()
    origin = 1_700_000_000_000_000_000
    seen, delayed = [], False
    for packet in _h264_packets(bframes=2):
        # One wire message's stamp belongs to that packet's source image.
        incoming = origin + packet.pts * 33_333_333
        frame = decoder.decode(bytes(packet), packet.is_keyframe, incoming)
        if frame is None:
            continue
        stamp = decoded_stamp_ns(frame)
        index, remainder = divmod(stamp-origin, 33_333_333)
        assert remainder == 0
        # Read identity from image content independently of packet/frame PTS.
        # Sample tile interiors to avoid compression at tile boundaries.
        pixels = frame.to_ndarray(format='bgr24')
        levels = [float(pixels[8:24, bit*16+4:bit*16+12].mean()) for bit in range(5)]
        assert all(level < 64 or level > 192 for level in levels), levels
        image_index = sum(1 << bit for bit, level in enumerate(levels) if level > 128)
        assert image_index == index, (image_index, index)
        seen.append(stamp)
        delayed |= stamp != incoming
    assert len(seen) >= 20 and seen == sorted(set(seen))
    assert delayed, 'exercise a decoded frame that is older than the incoming packet'


def test_decoder_corrupt_packet_resets_and_recovers_at_keyframe():
    from cortex_perception.ext_sensor import decoded_stamp_ns
    decoder = H264Decoder()
    packets = _h264_packets()
    assert decoder.decode(bytes(packets[0]), True, 1) is not None
    assert decoder.decode(b'invalid h264', False, 2) is None
    assert decoder._ctx is None
    assert decoder.decode(bytes(packets[1]), False, 3) is None
    keyframe = next(p for p in packets[1:] if p.is_keyframe)
    frame = decoder.decode(bytes(keyframe), True, 4)
    assert frame is not None and decoded_stamp_ns(frame) == 4


def test_decoded_frame_without_pts_cannot_claim_freshness():
    from types import SimpleNamespace
    from fractions import Fraction
    from cortex_perception.ext_sensor import decoded_stamp_ns
    assert decoded_stamp_ns(SimpleNamespace(pts=None, time_base=Fraction(1, 30))) is None
    assert decoded_stamp_ns(SimpleNamespace(pts=1, time_base=None)) is None


# --- mic input log ----------------------------------------------------------------------

def test_level_dbfs_full_scale_silence_and_half():
    assert level_dbfs(np.full(160, 32767, np.int16).tobytes())[1] > -0.01
    assert level_dbfs(bytes(320)) == (SILENCE_DBFS, SILENCE_DBFS)
    assert level_dbfs(b'') == (SILENCE_DBFS, SILENCE_DBFS)
    rms, peak = level_dbfs(np.full(160, 16384, np.int16).tobytes())
    assert abs(rms + 6.02) < 0.05 and abs(peak + 6.02) < 0.05


def test_monitor_first_chunk_and_summary_counts():
    m = MicMonitor()
    assert m.on_chunk(10, 0.0, 30.0, -40.0, -20.0)['first'] is True
    assert m.on_chunk(11, 0.1, 50.0, -30.0, -10.0)['first'] is False
    for outcome in ('sent', 'muted', 'dropped', 'sent'):
        m.on_outcome(outcome)
    s = m.summary(1.0)
    assert s['chunks'] == 2 and abs(s['rate'] - 2.0) < 1e-9
    assert (s['sent'], s['muted'], s['dropped']) == (2, 1, 1)
    assert s['rms_db'] == -35.0 and s['peak_db'] == -10.0
    assert s['age_ms_max'] == 50.0 and s['gaps'] == 0
    # counters reset; the window now starts at 1.0
    m.on_chunk(12, 1.5, None, -50.0, -40.0)
    s = m.summary(2.0)
    assert s['chunks'] == 1 and abs(s['span_s'] - 1.0) < 1e-9 and s['age_ms_p50'] is None


def test_monitor_counts_missing_seq():
    m = MicMonitor()
    m.on_chunk(1, 0.0, None, -40.0, -40.0)
    assert m.on_chunk(4, 0.1, None, -40.0, -40.0)['gap'] == 2
    assert m.summary(0.2)['gaps'] == 2


def test_monitor_stall_fires_once_then_resumes():
    m = MicMonitor(stall_s=1.0)
    assert m.check_stall(5.0) is False                 # nothing received yet: not a stall
    m.on_chunk(1, 0.0, None, -40.0, -40.0)
    assert m.check_stall(0.5) is False
    assert m.check_stall(1.2) is True
    assert m.check_stall(3.0) is False                 # reported once
    ev = m.on_chunk(2, 4.0, None, -40.0, -40.0)
    assert abs(ev['resumed_after_s'] - 4.0) < 1e-9
    assert m.check_stall(4.5) is False
