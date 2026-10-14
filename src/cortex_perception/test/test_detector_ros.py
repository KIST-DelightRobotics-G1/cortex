"""Actual generated ROS messages when Humble is installed; skipped otherwise."""
import time

import pytest

rclpy = pytest.importorskip('rclpy', reason='requires ROS 2 Humble and colcon-built interfaces')
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from cortex_msgs.srv import CheckTarget
from cortex_perception import detector_node
from cortex_perception.detection import DetectionRecord


def test_generated_message_adapter_preserves_source_time_and_raw_stride(monkeypatch):
    class Backend:
        names = {0: 'refrigerator', 1: 'cucumber'}
        def __init__(self, *args):
            self.last_frame = None
        def predict(self, frame):
            self.last_frame = frame
            return (DetectionRecord('cucumber', .9, .5, .5, .2, .1),)
    monkeypatch.setattr(detector_node, 'YoloDetector', Backend)
    rclpy.init()
    node = detector_node.DetectorNode(parameter_overrides=[Parameter('backend', value='yolo'),
                                                          Parameter('camera_transport', value='raw')])
    try:
        stamps = []
        for _ in range(3):
            msg = Image(height=1, width=2, encoding='rgb8', step=8,
                        data=bytes([1, 2, 3, 4, 5, 6, 0, 0]))
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            stamps.append((msg.header.stamp.sec, msg.header.stamp.nanosec))
            node._on_raw(msg)
            node._infer()
            time.sleep(.003)
        res = node._on_check(CheckTarget.Request(target='cucumber'), CheckTarget.Response())
        assert res.found and res.label == 'cucumber'
        assert (res.stamp.sec, res.stamp.nanosec) in stamps
        assert node._model.last_frame.tolist() == [[[3, 2, 1], [6, 5, 4]]]
        unsupported = node._on_check(CheckTarget.Request(target='user'), CheckTarget.Response())
        assert not unsupported.found and unsupported.detail == 'unsupported_class'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_h264_callback_uses_decoded_pts_not_packet_stamp():
    from fractions import Fraction
    from types import SimpleNamespace
    import av
    import numpy as np
    frame = av.VideoFrame.from_ndarray(np.zeros((2, 2, 3), np.uint8), format='bgr24')
    frame.pts, frame.time_base = 100, Fraction(1, 1_000_000_000)
    accepted = []
    fake = SimpleNamespace(
        _decoder=SimpleNamespace(decode=lambda *args: frame),
        _stats=SimpleNamespace(record=lambda *args, **kwargs: None),
        _json_log=lambda *args, **kwargs: None,
        _accept_frame_data=lambda *args: accepted.append(args))
    msg = SimpleNamespace(data=b'', is_keyframe=False, stamp_ns=200, frame_id='head')
    detector_node.DetectorNode._on_h264(fake, msg)
    assert accepted[0][1:] == (100, 'head')
    frame.pts = None
    detector_node.DetectorNode._on_h264(fake, msg)
    assert len(accepted) == 1


@pytest.mark.parametrize('enabled', [True, False])
def test_console_logs_preserve_check_result_and_report_pipeline(monkeypatch, enabled):
    import json
    from types import SimpleNamespace

    class Backend:
        names = {0: 'refrigerator', 1: 'cucumber'}
        def __init__(self, *args):
            pass
        def predict(self, frame):
            return ()
    monkeypatch.setattr(detector_node, 'YoloDetector', Backend)
    rclpy.init()
    node = detector_node.DetectorNode(parameter_overrides=[
        Parameter('backend', value='yolo'), Parameter('camera_transport', value='raw'),
        Parameter('log_events', value='config,status,check' if enabled else '')])
    logs = []
    monkeypatch.setattr(node, 'get_logger', lambda: SimpleNamespace(info=logs.append))
    try:
        for _ in range(3):
            msg = Image(height=1, width=1, encoding='bgr8', step=3, data=b'\0\0\0')
            msg.header.stamp = node.get_clock().now().to_msg()
            node._on_raw(msg)
            node._infer()
            time.sleep(.003)
        result = node._on_check(CheckTarget.Request(target='cucumber'), CheckTarget.Response())
        assert not result.found and result.detail == ''
        node._log_status()
        if enabled:
            check = json.loads(next(line.split(' ', 1)[1] for line in logs if line.startswith('YOLO_CHECK ')))
            assert check['reason'] == 'no_hits' and check['samples'] == 3 and check['hits'] == 0
            status = json.loads(next(line.split(' ', 1)[1] for line in logs if line.startswith('YOLO_STATUS ')))
            assert status['counters']['received'] == status['counters']['inference_completed'] == 3
            assert status['best_confidence'] == {}
            # A stale packet must be diagnosed but must not add fresh evidence.
            msg.header.stamp.sec -= 10
            node._on_raw(msg)
            snapshot = node._stats.snapshot()
            assert snapshot['counters']['stale_input'] == 1
            assert snapshot['counters']['frame_accepted'] == 3
        else:
            assert logs == []
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('console,jsonl', [(False, True), (True, True), (True, False)])
def test_optional_jsonl_records_frames_and_check_without_changing_decision(monkeypatch, tmp_path, console, jsonl):
    import json
    from types import SimpleNamespace

    class Backend:
        names = {0: 'cucumber'}
        def __init__(self, *args):
            self.calls = 0
        def predict(self, frame):
            self.calls += 1
            return (DetectionRecord('cucumber', .9, .5, .5, .2, .1),) if self.calls < 3 else ()
    monkeypatch.setattr(detector_node, 'YoloDetector', Backend)
    rclpy.init()
    node = detector_node.DetectorNode(parameter_overrides=[
        Parameter('backend', value='yolo'), Parameter('camera_transport', value='raw'),
        Parameter('require_latest_hit', value=True),
        Parameter('log_events', value='check' if console else ''),
        Parameter('diagnostics_jsonl_enabled', value=jsonl),
        Parameter('diagnostics_jsonl_dir', value=str(tmp_path))])
    logs = []
    monkeypatch.setattr(node, 'get_logger', lambda: SimpleNamespace(info=logs.append, warning=logs.append))
    try:
        stamps = []
        for _ in range(3):
            msg = Image(height=1, width=1, encoding='bgr8', step=3, data=b'\0\0\0')
            msg.header.stamp = node.get_clock().now().to_msg()
            stamps.append(msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec)
            node._on_raw(msg)
            node._infer()
            time.sleep(.003)
        result = node._on_check(CheckTarget.Request(target='cucumber'), CheckTarget.Response())
        assert not result.found and result.detail == ''
        node._log_status()
        msg.header.stamp.sec -= 10
        node._on_raw(msg)
        assert any(line.startswith('YOLO_CHECK ') for line in logs) is console
        if jsonl:
            assert node._jsonl.close()
            rows = [json.loads(line) for line in node._jsonl.path.read_text().splitlines()]
            assert rows[0]['event'] == 'YOLO_CONFIG'
            inferred = [r['data'] for r in rows if r['event'] == 'INFERENCE_RESULT']
            assert [r['source_stamp_ns'] for r in inferred] == stamps
            assert inferred[0]['detections'][0]['confidence'] == .9
            assert inferred[-1]['detections'] == []
            check = next(r['data'] for r in rows if r['event'] == 'YOLO_CHECK')
            assert check['reason'] == 'latest_miss'
            assert [r['hit'] for r in check['frames']] == [True, True, False]
            assert [r['source_stamp_ns'] for r in check['frames']] == stamps
            assert all('ros_time_ns' in r['data'] for r in rows)
            assert rows[-1]['event'] == 'FRAME_REJECTED' and rows[-1]['data']['reason'] == 'stale_input'
        else:
            assert node._jsonl is None and list(tmp_path.iterdir()) == []
    finally:
        node.destroy_node()
        rclpy.shutdown()
