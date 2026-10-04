#!/usr/bin/env python3
"""Real weight/video -> shared detector policy -> executor command sink.

ROS/DDS, generated message type support, cameras and robot motion are NOT run.
Only observed protocol fields and executor dispatch are checked here.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/cortex_perception'), str(ROOT/'src/cortex_cognition')]
from cortex_perception.detection import (PresenceWindow, YoloDetector, fill_response,
                                         target_mapping)
from cortex_cognition import executor as ex, planner


def command_sink(cfg, target, action, verdict, now, fail_open):
    commands = []
    ports = ex.Ports(now=lambda: now,
                     send_cmd=lambda module, step, pid: commands.append(
                         {'plan_id': pid, 'index': step.index, 'action': step.action,
                          'args': step.args, 'instruction': step.instruction, 'cancel': False,
                          'module': module}),
                     send_cancel=lambda *a: None,
                     check_target=lambda _: (verdict.found, verdict.detail),
                     say=lambda *a: None, stop_speech=lambda: None, trace=lambda *a: None,
                     status=lambda *a: None)
    machine = ex.Executor(cfg, ports, ex.Params(detector_fail_open=fail_open))
    machine.heard('offline-check', 'offline integration check')
    machine.on_step('offline-check', 0, 0, action, [target], '', '', '')
    assert bool(commands) == verdict.found
    return commands


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model', required=True)
    ap.add_argument('--video', action='append', required=True)
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--output', required=True)
    ap.add_argument('--profile', default=str(ROOT/'src/cortex_bringup/config/fridge_detector.yaml'))
    args = ap.parse_args()
    import cv2
    import torch
    import yaml
    torch.set_num_threads(4)
    profile = yaml.safe_load(Path(args.profile).read_text())
    p = profile['detector_node']['ros__parameters']
    fail_open = profile['orchestrator_node']['ros__parameters']['detector_fail_open']
    assert fail_open is False
    cfg = planner.load_config(str(ROOT/'src/cortex_cognition/config/actions.yaml'))
    model = YoloDetector(args.model, args.device, p['imgsz'], p['infer_conf'], p.get('model_sha256', ''))
    for name in ('refrigerator', 'cucumber'):
        assert name in model.names.values(), f'missing required class: {name}'
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    result = {'model': str(Path(args.model).resolve()),
              'weight_sha256': hashlib.sha256(Path(args.model).read_bytes()).hexdigest(),
              'classes': model.names, 'profile': profile, 'device': args.device,
              'ros_used': False,
              'scope': 'One-shot dispatch checks only; bounded precheck retries are tested by the subtask replay. Shared real detector and executor logic, no ROS/DDS/type support or robot. '
                       'One worker: sample next source frame after max(125ms, measured inference); '
                       'model warmup excluded, no transport/decode/ROS scheduling simulation.',
              'videos': []}
    from types import SimpleNamespace
    for source in args.video:
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise ValueError(f'cannot open video: {source}')
        w = PresenceWindow(target_mapping(p['target_keys'], p['target_classes']), model.names.values(),
                           p['window_s'], p['default_min_confidence'], p['min_frames'], p.get('require_latest_hit', False))
        records, counts = [], {target: {'allow': 0, 'block': 0, 'unavailable': 0}
                              for target in ('fridge_door', 'cucumber', 'user')}
        next_start, n = 0., 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            index = n; n += 1
            source_s = cap.get(cv2.CAP_PROP_POS_MSEC)/1000
            if source_s < next_start:
                continue
            if not records:
                for _ in range(3):
                    model.predict(frame)
            t0 = time.perf_counter()
            detections = model.predict(frame)
            infer_s = time.perf_counter()-t0
            now = source_s+infer_s
            stamp_ns = 1_700_000_000_000_000_000 + round(source_s*1e9)
            w.add(detections, source_s, stamp_ns, now)
            decisions = {}
            for target, action in [('fridge_door', 'open'), ('cucumber', 'pick'), ('user', None)]:
                verdict = w.check(target, now)
                response = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))
                fill_response(response, verdict)
                assert response.found == verdict.found
                assert response.stamp.sec*1_000_000_000+response.stamp.nanosec == verdict.stamp_ns
                for d in detections:
                    assert all(0 <= getattr(d, key) <= 1 for key in ('confidence', 'cx', 'cy', 'w', 'h'))
                cmds = command_sink(cfg, target, action, verdict, now, fail_open) if action else []
                counts[target]['allow' if verdict.found else 'unavailable' if verdict.detail else 'block'] += 1
                decisions[target] = {**asdict(verdict), 'commands': cmds}
            records.append({'source_frame': index, 'source_s': source_s, 'query_s': now,
                            'infer_ms': infer_s*1000, 'detections': [asdict(d) for d in detections],
                            'checks': decisions})
            next_start = max(source_s+1/p['rate_hz'], now)
        cap.release()
        assert records, 'no video frames'
        for target, action in [('fridge_door', 'open'), ('cucumber', 'pick')]:
            stale = w.check(target, records[-1]['query_s']+p['window_s']+.01)
            assert not stale.found and stale.detail == 'stale'
            assert not command_sink(cfg, target, action, stale, records[-1]['query_s']+1, fail_open)
        result['videos'].append({'source': source, 'decoded_frames': n, 'processed_frames': len(records),
                                 'counts': counts, 'stale_blocks_dispatch': True, 'records': records})
        print(Path(source).name, json.dumps(counts), flush=True)
    output.write_text(json.dumps(result, indent=2))
    print('OFFLINE_INTEGRATION_COMPLETE', output, flush=True)


if __name__ == '__main__':
    main()
