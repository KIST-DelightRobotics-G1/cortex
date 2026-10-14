#!/usr/bin/env python3
"""Passively record Cortex diagnostics; never send commands or change parameters."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import threading
import time

import yaml


def command(argv, path, timeout=15):
    try:
        result = subprocess.run(argv, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=timeout)
        path.write_text(result.stdout)
        return result.returncode
    except (OSError, subprocess.TimeoutExpired) as error:
        path.write_text(str(error)+'\n')
        return -1


def parameters(path, node):
    spec = yaml.safe_load(path.read_text()) or {}
    if not isinstance(spec, dict):
        return {}
    entry = spec.get(node, spec.get(node.lstrip('/'), {}))
    return entry.get('ros__parameters', {}) if isinstance(entry, dict) else {}


def live_parameters(node, path, deadline, stopped, status):
    if not node:
        return {}
    while not stopped.is_set():
        status[path.name] = command(['ros2', 'param', 'dump', node], path, timeout=5)
        if status[path.name] == 0:
            values = parameters(path, node)
            if values:
                return values
        if time.monotonic() >= deadline:
            break
        stopped.wait(min(0.5, max(0, deadline-time.monotonic())))
    return {}


def stop_recorder(process):
    """Flush only our rosbag process group; report any forced termination."""
    forced = False
    for sig, timeout in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        if process.poll() is not None:
            break
        forced = forced or sig != signal.SIGINT
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
    return forced


def collect(args, out, manifest, stopped):
    status = manifest['command_status']
    deadline = time.monotonic()+args.wait_for_nodes
    p = live_parameters(args.detector_node, out/'detector_params.yaml', deadline, stopped, status)
    o = live_parameters(args.orchestrator_node, out/'orchestrator_params.yaml', deadline, stopped, status)
    if stopped.is_set():
        manifest['status'] = 'stopped_before_recording'
        return
    if not p:
        raise RuntimeError('Cannot read live detector parameters; see detector_params.yaml. '
                           'Source ROS and the installed workspace.')
    camera = args.camera_topic or p.get('camera_topic')
    if not camera:
        raise RuntimeError('No camera topic found; specify --camera-topic')
    topics = list(dict.fromkeys([
        camera, p.get('detections_topic', '/cortex/detections'),
        p.get('diagnostics_topic', '/cortex/detector/diagnostics'),
        o.get('detector_diagnostics_topic', '/cortex/precheck/diagnostics'),
        o.get('transcript_topic', '/cortex/stt/transcript'),
        o.get('llm_request_topic', '/cortex/llm/request'), o.get('llm_step_topic', '/cortex/llm/step'),
        o.get('say_topic', '/cortex/tts/say'), o.get('stop_topic', '/cortex/tts/stop'),
        o.get('trace_topic', '/cortex/trace'), o.get('status_topic', '/cortex/task_status'),
        o.get('vla_cmd_topic', '/cortex/vla/cmd'), o.get('vla_state_topic', '/cortex/vla/state'),
        o.get('nav_cmd_topic', '/cortex/nav/cmd'), o.get('nav_state_topic', '/cortex/nav/state'),
        '/rosout']))
    # Best-effort subscribers match reliable and best-effort publishers.
    # Recorder queues only; the detector's camera QoS remains unchanged.
    qos = {topic: dict(reliability='best_effort', history='keep_last', depth=300,
                       durability='volatile') for topic in topics}
    (out/'recorder_qos.yaml').write_text(yaml.safe_dump(qos))
    manifest.update(status='recording', topics=topics,
                    diagnostic_enabled=p.get('diagnostics_enabled', False),
                    orchestrator_parameters_available=bool(o))
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2))
    with (out/'record.log').open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', str(out/'bag'),
             '--qos-profile-overrides-path', str(out/'recorder_qos.yaml'), *topics],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        recording_start = time.monotonic()
        end = recording_start+args.duration if args.duration else float('inf')
        manifest['recorder_started_unix_ns'] = time.time_ns()
        print(f'Recorder started: {out.resolve()} (duration={args.duration}s; 0=until shutdown). '
              'Check record.log for topic subscriptions before the trial.', flush=True)
        try:
            # Start recording before these slower, nonessential environment snapshots.
            for name, argv in {
                'nodes.txt': ['ros2', 'node', 'list'],
                'topics.txt': ['ros2', 'topic', 'list', '-t'],
                'camera_qos.txt': ['ros2', 'topic', 'info', camera, '--verbose'],
                'gpu.txt': ['nvidia-smi'],
                'git.txt': ['git', 'log', '-1', '--format=%H %s'],
                'git_status.txt': ['git', 'status', '--short'],
                'git_branch.txt': ['git', 'branch', '--show-current'],
                'runtime.txt': ['python3', '-c', 'import sys, torch, ultralytics, cv2; print(sys.version); print(torch.__version__, torch.version.cuda, ultralytics.__version__, cv2.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CUDA UNAVAILABLE")'],
            }.items():
                if stopped.is_set() or process.poll() is not None or time.monotonic() >= end:
                    break
                status[name] = command(argv, out/name, timeout=min(15, max(0.1, end-time.monotonic())))
            weight = Path(p.get('model', ''))
            if weight.is_file() and not stopped.is_set() and time.monotonic() < end:
                digest = hashlib.sha256()
                with weight.open('rb') as stream:
                    for chunk in iter(lambda: stream.read(1024*1024), b''):
                        if stopped.is_set() or time.monotonic() >= end:
                            break
                        digest.update(chunk)
                    else:
                        manifest['actual_model_sha256'] = digest.hexdigest()
            if 'actual_model_sha256' not in manifest:
                manifest['model_hash_note'] = 'Model inaccessible or snapshot interrupted; inspect detector heartbeat.'
            while process.poll() is None and time.monotonic() < end and not stopped.is_set():
                stopped.wait(0.2)
        finally:
            elapsed = time.monotonic()-recording_start
            requested_stop = stopped.is_set() or elapsed >= args.duration > 0
            forced = stop_recorder(process)
            manifest.update(recorded_elapsed_s=elapsed, recorder_returncode=process.returncode,
                            interrupted=stopped.is_set(), recorder_stop_escalated=forced)
    metadata = out/'bag/metadata.yaml'
    finished = requested_stop and not forced and process.returncode in (0, -signal.SIGINT, 130)
    manifest['status'] = 'recorded' if metadata.is_file() and finished else 'partial_recorder_exit'
    if metadata.is_file():
        bag = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
        counts = {entry['topic_metadata']['name']: entry['message_count']
                  for entry in bag['topics_with_message_count']}
        manifest['recorded_topic_counts'] = counts
        manifest['missing_topics'] = [topic for topic in topics if not counts.get(topic)]
        if not counts.get(camera):
            manifest['status'] = 'incomplete_no_camera'
        status['bag_info.txt'] = command(['ros2', 'bag', 'info', str(out/'bag')], out/'bag_info.txt')
    else:
        manifest['status'] = 'failed_no_bag_metadata'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration', type=int, default=90, help='seconds; 0 records until SIGINT/SIGTERM')
    parser.add_argument('--wait-for-nodes', type=float, default=0, help='startup discovery retry window in seconds')
    parser.add_argument('--detector-node', default='/detector_node')
    parser.add_argument('--orchestrator-node', default='/orchestrator_node', help='empty skips this node')
    parser.add_argument('--camera-topic', help='override discovery from the live detector parameters')
    args = parser.parse_args()
    if args.duration < 0 or args.wait_for_nodes < 0:
        parser.error('duration and wait-for-nodes must be nonnegative')
    try:
        args.output.mkdir(parents=True)
    except FileExistsError:
        parser.error('output must be a new directory')
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.set())
    manifest = dict(
        status='starting', platform=platform.platform(), started_unix_ns=time.time_ns(),
        duration_requested_s=args.duration, command_status={},
        ros_environment={k: os.environ.get(k) for k in
                         ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY')},
        notes='Passive recording only. No .env or microphone audio copied. '
              'ROS logs and transcripts can contain speech text; review before sharing. '
              'Bag starts after node discovery; missing_topics lists topics without recorded messages.')
    path = args.output/'manifest.json'
    path.write_text(json.dumps(manifest, indent=2))
    try:
        collect(args, args.output, manifest, stopped)
    except (OSError, ValueError, TypeError, KeyError, RuntimeError, yaml.YAMLError) as error:
        manifest.update(status='failed', error=str(error))
    finally:
        manifest['stopped_unix_ns'] = time.time_ns()
        path.write_text(json.dumps(manifest, indent=2))
    print(f'COLLECTION_{manifest["status"].upper()} {args.output.resolve()}', flush=True)
    if manifest['status'] != 'recorded':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
