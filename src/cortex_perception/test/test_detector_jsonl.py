"""Exercise real disk writes, bounded queues and failures without ROS or YOLO."""
import json
import threading

import pytest

from cortex_perception.detector_jsonl import DetectorJsonl, default_log_dir


def read_records(writer):
    # Backups are oldest at the highest suffix; the unsuffixed file is newest.
    paths = [writer.path.with_name(writer.path.name + f'.{i}')
             for i in range(writer.backup_count, 0, -1)] + [writer.path]
    return [json.loads(line) for path in paths if path.exists()
            for line in path.read_text().splitlines()]


def test_ros_path_overrides_and_unique_runs(tmp_path, monkeypatch):
    monkeypatch.delenv('ROS_LOG_DIR', raising=False)
    monkeypatch.delenv('ROS_HOME', raising=False)
    monkeypatch.setenv('HOME', str(tmp_path))
    assert default_log_dir() == tmp_path / '.ros/log/yolo_diagnostics'
    monkeypatch.setenv('ROS_HOME', str(tmp_path / 'ros-home'))
    assert default_log_dir() == tmp_path / 'ros-home/log/yolo_diagnostics'
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'launch-log'))
    assert default_log_dir() == tmp_path / 'launch-log/yolo_diagnostics'
    first, second = DetectorJsonl(), DetectorJsonl()
    assert first.close() and second.close()
    assert first.path != second.path
    assert first.path.parent.parent == default_log_dir()
    assert first.path.is_file() and second.path.is_file()
    custom = DetectorJsonl(str(tmp_path / 'custom'))
    assert custom.close()
    assert custom.path.parent.parent == tmp_path / 'custom'


def test_jsonl_round_trip_and_shutdown_drain(tmp_path):
    writer = DetectorJsonl(str(tmp_path))
    assert writer.record('INFERENCE_RESULT', ros_time_ns=123, source_stamp_ns=100,
                         detections=[dict(label='오이', confidence=.8, cx=.5)])
    assert writer.record('YOLO_CHECK', found=False, reason='latest_miss')
    assert writer.close()
    rows = read_records(writer)
    assert [r['event'] for r in rows] == ['INFERENCE_RESULT', 'YOLO_CHECK']
    assert [r['sequence'] for r in rows] == [1, 2]
    assert rows[0]['data']['detections'][0]['label'] == '오이'
    assert rows[0]['data']['source_stamp_ns'] == 100
    assert rows[0]['data']['ros_time_ns'] == 123
    assert rows[0]['schema_version'] == 1 and rows[0]['session_id'] == writer.path.parent.name
    assert rows[1]['monotonic_ns'] >= rows[0]['monotonic_ns'] > 0
    assert rows[0]['wall_time_ns'] > 0
    assert not writer.record('AFTER_CLOSE')
    assert writer.status()['written'] == 2


@pytest.mark.parametrize('backups', [0, 2])
def test_rotation_bounds_disk_usage_and_keeps_newest_records(tmp_path, backups):
    writer = DetectorJsonl(str(tmp_path), max_bytes=800, backup_count=backups)
    for i in range(20):
        assert writer.record('FRAME_ACCEPTED', source_stamp_ns=i, payload='x'*100)
    assert writer.close()
    paths = list(writer.path.parent.iterdir())
    assert len(paths) == backups + 1
    assert all(p.stat().st_size <= 800 for p in paths)
    rows = read_records(writer)
    assert rows[-1]['data']['source_stamp_ns'] == 19
    assert [r['sequence'] for r in rows] == sorted(r['sequence'] for r in rows)
    assert rows[0]['sequence'] > 1  # oldest data was intentionally rotated out


def test_full_queue_drops_without_waiting_for_disk_and_close_is_bounded(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = DetectorJsonl._write

    def blocked(self, row):
        entered.set()
        assert release.wait(5.)
        original(self, row)

    monkeypatch.setattr(DetectorJsonl, '_write', blocked)
    writer = DetectorJsonl(str(tmp_path), capacity=2)
    try:
        assert writer.record('FIRST') and entered.wait(1.)
        assert writer.record('SECOND') and writer.record('THIRD')
        # Producer remains responsive while the writer is stalled.
        assert not writer.record('OVERFLOW')
        assert writer.status()['dropped'] == 1 and writer.status()['queued'] == 2
        assert not writer.close(timeout=.01)
    finally:
        release.set()
        assert writer.close()
    assert writer.status()['written'] == 3
    assert len(read_records(writer)) == 3


def test_unwritable_destination_disables_only_recording(tmp_path):
    destination = tmp_path / 'regular-file'
    destination.write_text('keep')
    writer = DetectorJsonl(str(destination))
    assert not writer.status()['active'] and writer.status()['error']
    assert not writer.record('YOLO_CHECK')
    assert writer.close()
    assert destination.read_text() == 'keep'


def test_disk_error_stops_recording_without_escaping_to_producer(tmp_path, monkeypatch):
    def fail(self, row):
        raise OSError('simulated disk full')
    monkeypatch.setattr(DetectorJsonl, '_write', fail)
    writer = DetectorJsonl(str(tmp_path))
    writer.record('INFERENCE_RESULT')
    assert writer.close()
    state = writer.status()
    assert state['error'] == 'simulated disk full' and not state['active']
    assert state['dropped'] == 1 and state['written'] == 0
    assert not writer.record('YOLO_CHECK')


def test_invalid_or_oversized_record_does_not_poison_later_records(tmp_path):
    writer = DetectorJsonl(str(tmp_path), max_bytes=1024)
    writer.record('INVALID', value=float('nan'))
    writer.record('OVERSIZED', value='x'*2048)
    writer.record('VALID', value=None)
    assert writer.close()
    assert [r['event'] for r in read_records(writer)] == ['VALID']
    assert writer.status()['serialization_errors'] == 1
    assert writer.status()['dropped'] == 2


def test_concurrent_callbacks_produce_complete_ordered_lines(tmp_path):
    writer = DetectorJsonl(str(tmp_path), capacity=1024)
    def produce(index):
        for sample in range(50):
            assert writer.record('FRAME', producer=index, sample=sample)
    threads = [threading.Thread(target=produce, args=(i,)) for i in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert writer.close()
    rows = read_records(writer)
    assert [r['sequence'] for r in rows] == list(range(1, 201))
    assert len({(r['data']['producer'], r['data']['sample']) for r in rows}) == 200
    assert writer.status()['dropped'] == 0


@pytest.mark.parametrize('kwargs', [dict(max_bytes=0), dict(backup_count=-1), dict(capacity=0)])
def test_invalid_limits_rejected_before_creating_files(tmp_path, kwargs):
    with pytest.raises(ValueError):
        DetectorJsonl(str(tmp_path), **kwargs)
    assert list(tmp_path.iterdir()) == []


def test_writer_thread_start_failure_disables_only_recording(tmp_path, monkeypatch):
    def fail(self):
        raise RuntimeError('cannot start writer thread')
    monkeypatch.setattr(threading.Thread, 'start', fail)
    writer = DetectorJsonl(str(tmp_path))
    assert writer.status()['error'] == 'cannot start writer thread'
    assert not writer.record('YOLO_CHECK')
    assert writer.close()
