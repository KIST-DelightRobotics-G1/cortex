"""Run the real collector against passive fake ROS CLI processes (no robot/DDS)."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/'scripts/collect_yolo_diagnostics.py'


@pytest.fixture
def recorder(tmp_path):
    bin_dir = tmp_path/'bin'
    bin_dir.mkdir()
    cli = '#!' + sys.executable + '\n' + r"""
import json, os, signal, sys, time
from pathlib import Path
a = sys.argv[1:]
root = Path(os.environ['COLLECT_TEST_ROOT'])
mode = os.environ.get('COLLECT_TEST_MODE', '')
with (root/'calls.jsonl').open('a') as stream:
    stream.write(json.dumps([Path(sys.argv[0]).name, *a])+'\n')
if Path(sys.argv[0]).name != 'ros2':
    print('test snapshot')
elif a[:2] == ['param', 'dump']:
    node = a[2]
    if mode in ('missing', 'waiting'):
        sys.exit(1)
    if mode == 'delayed' and not (root/'attempted').exists():
        (root/'attempted').touch()
        sys.exit(1)
    values = ({'camera_topic': '/test/camera', 'diagnostics_enabled': True}
              if node == '/detector_node' else
              {'nav_cmd_topic': '/test/nav', 'transcript_topic': '/test/transcript'})
    print(json.dumps({node: {'ros__parameters': values}}))
elif a[:2] == ['bag', 'record']:
    if mode == 'crash':
        sys.exit(2)
    bag = Path(a[a.index('-o')+1])
    bag.mkdir()
    stop = [False]
    def on_signal(*_):
        stop[0] = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    (root/'ready').write_text(str(os.getpid()))
    while not stop[0]:
        time.sleep(0.02)
    entries = [] if mode == 'no_camera' else [
        {'topic_metadata': {'name': '/test/camera'}, 'message_count': 8}]
    (bag/'metadata.yaml').write_text(json.dumps({
        'rosbag2_bagfile_information': {'topics_with_message_count': entries}}))
    if mode == 'bad_metadata':
        (bag/'metadata.yaml').write_text('{}')
    (root/'flushed').touch()
else:
    print('test ROS snapshot')
"""
    for name in ('ros2', 'nvidia-smi', 'git', 'python3'):
        path = bin_dir/name
        path.write_text(cli)
        path.chmod(0o755)
    env = {**os.environ, 'PATH': str(bin_dir)+os.pathsep+os.environ['PATH'],
           'COLLECT_TEST_ROOT': str(tmp_path), 'PYTHONDONTWRITEBYTECODE': '1'}
    return tmp_path, env


def run(recorder, *args, mode=''):
    root, env = recorder
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--output', str(root/'out'), '--duration', '1', *args],
        env={**env, 'COLLECT_TEST_MODE': mode}, text=True, capture_output=True, timeout=15)
    manifest = json.loads((root/'out/manifest.json').read_text())
    return result, manifest


def wait_for(path, process):
    deadline = time.monotonic()+10
    while not path.exists() and process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.03)
    assert path.exists(), process.communicate(timeout=5)


def test_finite_recording_starts_before_environment_snapshots(recorder):
    root, _ = recorder
    result, manifest = run(recorder)
    assert result.returncode == 0, result.stdout+result.stderr
    assert manifest['status'] == 'recorded'
    assert manifest['recorded_topic_counts']['/test/camera'] == 8
    assert '/test/nav' in manifest['topics'] and '/test/transcript' in manifest['topics']
    assert '/cortex/tts/say' in manifest['missing_topics']
    assert not manifest['recorder_stop_escalated']
    calls = [json.loads(line) for line in (root/'calls.jsonl').read_text().splitlines()]
    record = next(i for i, a in enumerate(calls) if a[:3] == ['ros2', 'bag', 'record'])
    assert record < next(i for i, a in enumerate(calls) if a[0] == 'nvidia-smi')
    assert all(a[1:3] not in (['topic', 'pub'], ['service', 'call'], ['param', 'set']) for a in calls)


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_until_shutdown_flushes_owned_recorder(recorder, sig):
    root, env = recorder
    process = subprocess.Popen(
        [sys.executable, str(SCRIPT), '--output', str(root/'out'), '--duration', '0',
         '--orchestrator-node', ''], env=env, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        wait_for(root/'ready', process)
        process.send_signal(sig)
        output, _ = process.communicate(timeout=12)
        assert process.returncode == 0, output
        manifest = json.loads((root/'out/manifest.json').read_text())
        assert manifest['status'] == 'recorded' and manifest['interrupted']
        assert not manifest['orchestrator_parameters_available']
        assert (root/'flushed').exists()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        if (root/'ready').exists() and not (root/'flushed').exists():
            try:
                os.kill(int((root/'ready').read_text()), signal.SIGKILL)
            except ProcessLookupError:
                pass


def test_node_discovery_retries_during_launch(recorder):
    result, manifest = run(recorder, '--wait-for-nodes', '2', mode='delayed')
    assert result.returncode == 0, result.stdout+result.stderr
    assert manifest['status'] == 'recorded'


@pytest.mark.parametrize('mode,expected', [
    ('missing', 'failed'), ('crash', 'failed_no_bag_metadata'),
    ('no_camera', 'incomplete_no_camera'), ('bad_metadata', 'failed'),
])
def test_partial_collection_is_not_reported_as_success(recorder, mode, expected):
    result, manifest = run(recorder, mode=mode)
    assert result.returncode == 1
    assert manifest['status'] == expected
    assert 'stopped_unix_ns' in manifest


def test_shutdown_while_waiting_preserves_failure_manifest(recorder):
    root, env = recorder
    process = subprocess.Popen(
        [sys.executable, str(SCRIPT), '--output', str(root/'out'), '--duration', '0',
         '--wait-for-nodes', '30'], env={**env, 'COLLECT_TEST_MODE': 'waiting'},
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        wait_for(root/'calls.jsonl', process)
        process.terminate()
        output, _ = process.communicate(timeout=8)
        assert process.returncode == 1, output
        manifest = json.loads((root/'out/manifest.json').read_text())
        assert manifest['status'] == 'stopped_before_recording'
        assert not (root/'ready').exists()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_existing_run_is_never_overwritten(recorder):
    root, env = recorder
    out = root/'out'
    out.mkdir()
    (out/'manifest.json').write_text('keep this')
    result = subprocess.run([sys.executable, str(SCRIPT), '--output', str(out)],
                            env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert (out/'manifest.json').read_text() == 'keep this'
