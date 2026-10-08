#!/usr/bin/env python3
"""Record an already running Cortex; never launch motion, call CheckTarget or change parameters."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import time


def command(argv, path, timeout=15):
    try:
        result = subprocess.run(argv, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
        path.write_text(result.stdout)
        return result.returncode
    except (OSError, subprocess.TimeoutExpired) as error:
        path.write_text(str(error)+'\n')
        return -1


def parameters(path, node):
    import yaml
    spec = yaml.safe_load(path.read_text()) or {}
    return spec.get(node, spec.get(node.lstrip('/'), {})).get('ros__parameters', {})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration', type=int, default=90)
    parser.add_argument('--detector-node', default='/detector_node')
    parser.add_argument('--orchestrator-node', default='/orchestrator_node')
    parser.add_argument('--camera-topic', help='override discovery from the live detector parameters')
    args = parser.parse_args()
    if args.duration <= 0 or args.output.exists():
        parser.error('duration must be positive and output must be a new directory')
    args.output.mkdir(parents=True)
    out = args.output
    status = {}
    for name, argv in {
        'detector_params.yaml': ['ros2', 'param', 'dump', args.detector_node],
        'orchestrator_params.yaml': ['ros2', 'param', 'dump', args.orchestrator_node],
        'nodes.txt': ['ros2', 'node', 'list'],
        'topics.txt': ['ros2', 'topic', 'list', '-t'],
        'gpu.txt': ['nvidia-smi'],
        'git.txt': ['git', 'log', '-1', '--format=%H %s'],
        'git_status.txt': ['git', 'status', '--short'],
        'git_branch.txt': ['git', 'branch', '--show-current'],
        'runtime.txt': ['python3', '-c', 'import sys, torch, ultralytics, cv2; print(sys.version); print(torch.__version__, torch.version.cuda, ultralytics.__version__, cv2.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CUDA UNAVAILABLE")'],
    }.items():
        status[name] = command(argv, out/name)
    if status['detector_params.yaml'] != 0:
        raise SystemExit('Cannot read live detector parameters. See detector_params.yaml; source ROS and the installed workspace in this terminal.')
    p = parameters(out/'detector_params.yaml', args.detector_node)
    o = parameters(out/'orchestrator_params.yaml', args.orchestrator_node) if status['orchestrator_params.yaml'] == 0 else {}
    camera = args.camera_topic or p.get('camera_topic')
    if not camera:
        raise SystemExit('No camera topic found; specify --camera-topic')
    command(['ros2', 'topic', 'info', camera, '--verbose'], out/'camera_qos.txt')
    topics = list(dict.fromkeys([camera, p.get('detections_topic', '/cortex/detections'),
        p.get('diagnostics_topic', '/cortex/detector/diagnostics'),
        o.get('detector_diagnostics_topic', '/cortex/precheck/diagnostics'),
        o.get('trace_topic', '/cortex/trace'), o.get('status_topic', '/cortex/task_status'),
        o.get('vla_cmd_topic', '/cortex/vla/cmd'), o.get('vla_state_topic', '/cortex/vla/state'),
        o.get('nav_state_topic', '/cortex/nav/state'), '/rosout']))
    import yaml
    # Best-effort subscriber matches reliable or best-effort camera publishers.
    # Deep recorder queue only; this does NOT change the detector's camera QoS.
    qos = {topic: dict(reliability='best_effort', history='keep_last', depth=300,
                       durability='volatile') for topic in (camera, p.get('diagnostics_topic', '/cortex/detector/diagnostics'),
                       o.get('detector_diagnostics_topic', '/cortex/precheck/diagnostics'))}
    (out/'recorder_qos.yaml').write_text(yaml.safe_dump(qos))
    manifest = dict(status='recording', platform=platform.platform(), started_unix_ns=time.time_ns(),
                    duration_requested_s=args.duration, command_status=status, topics=topics,
                    ros_environment={k: os.environ.get(k) for k in ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY')},
                    diagnostic_enabled=p.get('diagnostics_enabled', False),
                    notes='Passive recording only. No .env, microphone audio or credentials copied. Review ROS logs before sharing; speech text may occur in task traces.')
    weight = Path(p.get('model', ''))
    if weight.is_file():
        digest = hashlib.sha256()
        with weight.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024*1024), b''):
                digest.update(chunk)
        manifest['actual_model_sha256'] = digest.hexdigest()
    else:
        manifest['model_hash_note'] = 'Model not accessible in collector filesystem; inspect detector heartbeat or run inside its container.'
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2))
    interrupted = False
    with (out/'record.log').open('w') as log:
        process = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(out/'bag'),
                                    '--qos-profile-overrides-path', str(out/'recorder_qos.yaml'), *topics],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        recording_start = time.monotonic()
        print(f'Recording for {args.duration}s. Run the agreed scenarios now. Output: {out.resolve()}', flush=True)
        try:
            process.wait(timeout=args.duration)
        except subprocess.TimeoutExpired:
            pass
        except KeyboardInterrupt:
            interrupted = True
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
    metadata = out/'bag/metadata.yaml'
    elapsed = time.monotonic()-recording_start
    finished = process.returncode in (0, -signal.SIGINT, 130) and (interrupted or elapsed >= args.duration)
    manifest.update(recorded_elapsed_s=elapsed, stopped_unix_ns=time.time_ns(), recorder_returncode=process.returncode,
                    interrupted=interrupted, status='recorded' if metadata.is_file() and finished else 'partial_recorder_exit' if metadata.is_file() else 'failed')
    if metadata.is_file():
        bag = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
        counts = {entry['topic_metadata']['name']: entry['message_count'] for entry in bag['topics_with_message_count']}
        manifest['recorded_topic_counts'] = counts
        manifest['missing_topics'] = [topic for topic in topics if not counts.get(topic)]
        if not counts.get(camera):
            manifest['status'] = 'incomplete_no_camera'
        command(['ros2', 'bag', 'info', str(out/'bag')], out/'bag_info.txt')
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(f'COLLECTION_{manifest["status"].upper()} {out.resolve()}')
    if manifest['status'] != 'recorded':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
