"""Real Humble bag serialization and MP4 export, with source PTS preservation."""
import json
from pathlib import Path
import subprocess
import sys

import pytest
rosbag2_py = pytest.importorskip('rosbag2_py')
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import String


def test_export_raw_bag_preserves_variable_source_timestamps_and_events(tmp_path):
    cv2 = pytest.importorskip('cv2')
    pytest.importorskip('av')
    bag, output = tmp_path/'bag', tmp_path/'export'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topic = '/test/camera'
    diagnostic = '/cortex/detector/diagnostics'
    for name, kind in [(topic, 'sensor_msgs/msg/Image'), (diagnostic, 'std_msgs/msg/String')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format='cdr'))
    origin = 1_700_000_000_000_000_000
    times = [0, .1, .25, .5]
    for index, seconds in enumerate(times):
        message = Image(height=24, width=32, encoding='rgb8', step=96,
                        data=bytes([40+index*20, 70, 120])*32*24)
        stamp = origin+round(seconds*1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        writer.write(topic, serialize_message(message), stamp+50_000_000)
    writer.write(diagnostic, serialize_message(String(data='{"event":"check","reason":"latest_miss"}')), origin+600_000_000)
    del writer
    script = Path(__file__).resolve().parents[1]/'scripts/export_yolo_bag.py'
    result = subprocess.run([sys.executable, str(script), '--bag', str(bag), '--output', str(output),
                             '--camera-topic', topic], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout+result.stderr
    frames = [json.loads(line) for line in (output/'video_frames.jsonl').read_text().splitlines()]
    assert [r['source_s'] for r in frames] == times
    event = json.loads((output/'events.jsonl').read_text().splitlines()[0])
    assert event['message']['reason'] == 'latest_miss'
    cap = cv2.VideoCapture(str(output/'camera.mp4'))
    actual = []
    while cap.read()[0]:
        actual.append(cap.get(cv2.CAP_PROP_POS_MSEC)/1000)
    cap.release()
    assert actual == pytest.approx(times, abs=.005)


def test_export_h264_bag_keeps_decoded_frame_time_not_current_packet_time(tmp_path):
    import importlib.util
    from kist_msgs.msg import CompressedColorFrame
    cv2 = pytest.importorskip('cv2')
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('h264_fixtures', root/'src/cortex_perception/test/test_ext_sensor.py')
    fixtures = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixtures)
    bag, output = tmp_path/'bag', tmp_path/'export'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    topic = '/kist/camera/head/color/h264'
    writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type='kist_msgs/msg/CompressedColorFrame', serialization_format='cdr'))
    origin = 1_700_000_000_000_000_000
    for index, packet in enumerate(fixtures._h264_packets(bframes=2)):
        message = CompressedColorFrame(width=80, height=32, seq=index,
                                       stamp_ns=origin+packet.pts*33_333_333,
                                       is_keyframe=packet.is_keyframe, frame_id='head', data=bytes(packet))
        writer.write(topic, serialize_message(message), origin+index*33_333_333)
    del writer
    result = subprocess.run([sys.executable, str(root/'scripts/export_yolo_bag.py'), '--bag', str(bag),
                             '--output', str(output)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout+result.stderr
    frames = [json.loads(line) for line in (output/'video_frames.jsonl').read_text().splitlines()]
    cap = cv2.VideoCapture(str(output/'camera.mp4'))
    decoded = 0
    for record in frames:
        ok, pixels = cap.read()
        assert ok
        image_index = sum(1 << bit for bit in range(5)
                          if pixels[8:24, bit*16+4:bit*16+12].mean() > 128)
        assert record['source_stamp_ns'] == origin+image_index*33_333_333
        decoded += 1
    cap.release()
    assert decoded >= 20
