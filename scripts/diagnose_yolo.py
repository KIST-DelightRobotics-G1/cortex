#!/usr/bin/env python3
"""Saved video -> real YOLO boxes, confidence sweep and shared Cortex presence policy.

No ROS imports or robot commands. Policy queries occur at simulated inference
completion times; this does NOT simulate DDS, packet loss or executor retries.
"""
import argparse
from collections import Counter
import csv
from dataclasses import asdict
import hashlib
import html
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/cortex_perception'))
from cortex_perception.detection import PresenceWindow, YoloDetector, target_mapping


def sha256(path):
    with open(path, 'rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else _digest(stream)


def _digest(stream):
    digest = hashlib.sha256()
    for chunk in iter(lambda: stream.read(1024*1024), b''):
        digest.update(chunk)
    return digest.hexdigest()


def profile_values(path, overlay=None):
    import yaml
    values = {}
    for source in (path, overlay):
        if source:
            cfg = yaml.safe_load(Path(source).read_text())
            # Accept a full site YAML or a ros2 param dump with /detector_node.
            node = cfg.get('detector_node', cfg.get('/detector_node', {}))
            values.update(node.get('ros__parameters', {}))
    required = ('imgsz', 'infer_conf', 'rate_hz', 'window_s', 'default_min_confidence',
                'min_frames', 'require_latest_hit', 'target_keys', 'target_classes')
    if any(key not in values for key in required):
        raise ValueError('profile is missing detector parameters: use the full site YAML or detector param dump')
    return values


def source_time(index, pts_ms, fps, previous):
    """Never silently use duplicate or nonmonotonic timestamps as independent frames."""
    pts = pts_ms/1000
    if math.isfinite(pts) and pts >= 0 and (previous is None or pts > previous):
        return pts, 'pts'
    fallback = index/fps
    if previous is not None and fallback <= previous:
        raise ValueError('nonmonotonic video timestamps; remux/export a valid video before diagnosis')
    return fallback, 'fps_fallback'


def make_windows(p, names, thresholds):
    mapping = target_mapping(p['target_keys'], p['target_classes'])
    cases = {'baseline': (float(p['infer_conf']), float(p['default_min_confidence']))}
    cases.update({f'confidence_{value:g}': (min(float(p['infer_conf']), value), value) for value in thresholds})
    windows = {key: PresenceWindow(mapping, names, p['window_s'], conf, p['min_frames'],
                                  p['require_latest_hit']) for key, (_, conf) in cases.items()}
    return cases, windows


def evaluate(windows, cases, baseline, candidates, source_s, completion_s):
    rows = []
    stamp = 1_700_000_000_000_000_000 + round(source_s*1e9)
    for case, window in windows.items():
        floor, _ = cases[case]
        boxes = baseline if case == 'baseline' else tuple(d for d in candidates if d.confidence >= floor)
        accepted = window.add(boxes, source_s, stamp, completion_s)
        for target in ('fridge_door', 'cucumber'):
            evidence = {}
            window.check(target, completion_s, diagnostics=evidence)
            rows.append(dict(case=case, target=target, source_s=source_s,
                             query_s=completion_s, window_accepted=accepted, **evidence))
    return rows


def write_timeline(path, rows, duration):
    """SVG of sampled decisions, never labeled as ground-truth accuracy."""
    duration = max(duration, max((r['query_s'] for r in rows), default=0))
    groups = sorted({(r['case'], r['target']) for r in rows})
    width, left, top, row_h = 1200, 265, 60, 32
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{top+len(groups)*row_h+65}">',
           '<rect width="100%" height="100%" fill="white"/>',
           '<g font-family="sans-serif" font-size="12" fill="#172b4d">',
           '<text x="12" y="20">Sampled policy decisions (not accuracy): green=allow, red=block, gray=unavailable</text>']
    scale = (width-left-30)/max(duration, .001)
    for k in range(6):
        x = left+(width-left-30)*k/5
        out.append(f'<text x="{x:.2f}" y="43">{duration*k/5:.1f}s</text>')
    for i, (case, target) in enumerate(groups):
        y = top+i*row_h
        out.append(f'<text x="12" y="{y+14}">{html.escape(case+" / "+target)}</text>')
        out.append(f'<rect x="{left}" y="{y}" width="{width-left-30}" height="20" fill="#f4f5f7"/>')
        for r in rows:
            if (r['case'], r['target']) != (case, target):
                continue
            color = '#18845c' if r['found'] else '#89939f' if r['detail'] else '#cb3549'
            x = left+min(r['query_s'], duration)*scale
            out.append(f'<rect x="{x:.2f}" y="{y}" width="2" height="20" fill="{color}"><title>'
                       f'{r["query_s"]:.3f}s {html.escape(r["reason"])}</title></rect>')
    out.append('</g></svg>')
    path.write_text('\n'.join(out))


def synchronize(torch, device):
    if device not in ('cpu', 'mps'):
        torch.cuda.synchronize(int(device.removeprefix('cuda:')))
    elif device == 'mps':
        torch.mps.synchronize()


def inspect_video(args, p, model, source, out, torch, cv2):
    out.mkdir()
    cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise ValueError(f'cannot open video: {source}')
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError('video has no valid FPS')
    cases, windows = make_windows(p, model.names.values(), args.thresholds)
    floor = min(value[0] for value in cases.values())
    previous, origin, next_start, index = None, None, 0., 0
    rows, timings, warmups, fallbacks = [], [], [], 0
    writer = None
    try:
        with (out/'frames.jsonl').open('w') as raw:
            while args.max_frames == 0 or index < args.max_frames:
                ok, frame = cap.read()
                if not ok:
                    break
                pts, mode = source_time(index, cap.get(cv2.CAP_PROP_POS_MSEC), fps, previous)
                previous = pts
                origin = pts if origin is None else origin
                source_s = pts-origin
                fallbacks += mode == 'fps_fallback'
                if index == 0:
                    for _ in range(args.warmup):
                        synchronize(torch, args.device)
                        start = time.perf_counter()
                        model.predict(frame)
                        synchronize(torch, args.device)
                        warmups.append((time.perf_counter()-start)*1000)
                    if not args.no_video:
                        height, width = frame.shape[:2]
                        writer = cv2.VideoWriter(str(out/'annotated.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
                        if not writer.isOpened():
                            raise RuntimeError('MP4 writer unavailable; rerun with --no-video or install an OpenCV build with MP4 support')
                model.infer_conf = float(p['infer_conf'])
                synchronize(torch, args.device)
                start = time.perf_counter()
                baseline = model.predict(frame)
                synchronize(torch, args.device)
                infer_s = time.perf_counter()-start
                timings.append(infer_s*1000)
                # Baseline is measured separately at the deployed infer_conf. The
                # lower-floor diagnostic pass never contaminates its timing/boxes.
                candidates = baseline
                if floor < p['infer_conf']:
                    model.infer_conf = floor
                    candidates = model.predict(frame)
                    model.infer_conf = float(p['infer_conf'])
                selected = source_s+1e-9 >= next_start
                if selected:
                    completion = source_s+infer_s+args.source_delay_ms/1000
                    decision_rows = evaluate(windows, cases, baseline, candidates, source_s, completion)
                    for row in decision_rows:
                        row.update(frame=index, baseline_infer_ms=infer_s*1000)
                    rows.extend(decision_rows)
                    # Constant source transport age shifts arrival/completion, not source cadence.
                    next_start = source_s + max(1/p['rate_hz'], infer_s)
                raw.write(json.dumps(dict(frame=index, pts_s=pts, source_s=source_s, timestamp_mode=mode,
                                          baseline_infer_ms=infer_s*1000, policy_sample=selected,
                                          baseline_boxes=[asdict(d) for d in baseline],
                                          sweep_boxes=[asdict(d) for d in candidates]))+'\n')
                if writer is not None:
                    height, width = frame.shape[:2]
                    for d in baseline:
                        color = (40, 180, 60) if d.label == 'refrigerator' else (20, 150, 240)
                        a = (round((d.cx-d.w/2)*width), round((d.cy-d.h/2)*height))
                        b = (round((d.cx+d.w/2)*width), round((d.cy+d.h/2)*height))
                        cv2.rectangle(frame, a, b, color, 2)
                        cv2.putText(frame, f'{d.label} {d.confidence:.3f}', (a[0], max(20, a[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
                    cv2.putText(frame, f'frame={index} source={source_s:.3f}s (baseline boxes)', (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .6, (20, 20, 240), 2)
                    writer.write(frame)
                index += 1
                if index % 100 == 0:
                    print(f'{source.name}: {index} frames', flush=True)
    finally:
        cap.release()
        if writer is not None:
            writer.release()
    if not timings:
        raise ValueError(f'no decodable video frames: {source}')
    keys = sorted({key for row in rows for key in row})
    with (out/'policy.csv').open('w', newline='') as stream:
        writer_csv = csv.DictWriter(stream, fieldnames=keys)
        writer_csv.writeheader(); writer_csv.writerows(rows)
    counts = {}
    for row in rows:
        key = row['case']+'/'+row['target']
        counts.setdefault(key, Counter())[row['reason']] += 1
    ordered = sorted(timings)
    duration = previous-origin
    write_timeline(out/'timeline.svg', rows, duration)
    summary = dict(video=str(source), video_sha256=sha256(source), decoded_frames=index,
                   source_duration_s=duration, source_fps=fps, timestamp_fallback_frames=fallbacks,
                   warmup_ms=warmups, baseline_infer_ms=dict(mean=sum(timings)/len(timings),
                       p50=ordered[len(ordered)//2], p95=ordered[math.ceil(len(ordered)*.95)-1]),
                   policy_counts=counts, sweep_inference_floor=floor, cases=cases,
                   output=str(out), annotated_video_timing='constant FPS preview; CSV/JSONL timestamps are authoritative')
    (out/'summary.json').write_text(json.dumps(summary, indent=2))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True, type=Path)
    parser.add_argument('--video', required=True, action='append', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--device', default='0', help='CUDA index (default 0); cpu/mps only for explicit local checks')
    parser.add_argument('--profile', type=Path, default=ROOT/'src/cortex_bringup/config/cortex_params.yaml')
    parser.add_argument('--detector-profile', type=Path)
    parser.add_argument('--expected-sha256', help='intentional replacement weight hash; default is the profile hash')
    parser.add_argument('--thresholds', default='0.15,0.20,0.25')
    parser.add_argument('--max-frames', type=int, default=0, help='0=whole video; nonzero is a truncated diagnostic run')
    parser.add_argument('--warmup', type=int, default=3)
    parser.add_argument('--source-delay-ms', type=float, default=0, help='hypothetical source delay for policy replay ONLY; not measured network latency')
    parser.add_argument('--no-video', action='store_true')
    args = parser.parse_args()
    args.thresholds = sorted(set(float(t) for t in args.thresholds.split(',')))
    if any(not 0 < t <= 1 for t in args.thresholds) or args.max_frames < 0 or args.warmup < 0 or not math.isfinite(args.source_delay_ms) or args.source_delay_ms < 0:
        parser.error('invalid threshold, frame limit, warmup or source delay')
    if not args.model.is_file() or any(not p.is_file() for p in args.video):
        parser.error('model and videos must be existing local files; no downloads are performed')
    if args.output.exists():
        parser.error('output already exists; choose a new directory')
    import cv2
    import torch
    if args.device not in ('cpu', 'mps'):
        device_index = args.device.removeprefix('cuda:')
        if not device_index.isdigit() or not torch.cuda.is_available() or int(device_index) >= torch.cuda.device_count():
            parser.error('CUDA device unavailable. Check nvidia-smi, Docker --gpus and CUDA PyTorch; no CPU fallback is used')
        args.device = device_index
    p = profile_values(args.profile, args.detector_profile)
    if not math.isfinite(float(p['rate_hz'])) or p['rate_hz'] <= 0:
        parser.error('rate_hz must be positive')
    expected = args.expected_sha256 if args.expected_sha256 is not None else p.get('model_sha256', '')
    model = YoloDetector(args.model, args.device, p['imgsz'], p['infer_conf'], expected)
    if not {'refrigerator', 'cucumber'}.issubset(model.names.values()):
        parser.error('this diagnostic requires the refrigerator/cucumber model; check class names')
    args.output.mkdir(parents=True)
    try:
        revision = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
        manifest = dict(schema=1, status='running', git_base_revision=revision,
                        script_sha256=sha256(__file__), policy_code_sha256=sha256(ROOT/'src/cortex_perception/cortex_perception/detection.py'),
                        python=sys.version, platform=platform.platform(), device=args.device,
                        cuda_available=torch.cuda.is_available(), torch_cuda=torch.version.cuda,
                        gpu=torch.cuda.get_device_name(int(args.device)) if args.device.isdigit() else None,
                        libraries={name: importlib.metadata.version(name) for name in ('torch', 'torchvision', 'ultralytics', 'opencv-python', 'PyYAML')},
                        model=str(args.model.resolve()), model_sha256=sha256(args.model), classes=model.names,
                        detector_parameters=p, max_frames=args.max_frames, source_delay_ms=args.source_delay_ms,
                        scope='Exploratory saved-video analysis, no labels/accuracy metrics. Baseline inference measured separately after warmup. '
                              'Policy queries at simulated completion, target rate and measured inference; no DDS/clock skew/decoder queue or executor deadline replay. '
                              'Sweep uses one lower-floor candidate pass; it is not a deployment performance benchmark.', videos=[])
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
        for i, source in enumerate(args.video, 1):
            manifest['videos'].append(inspect_video(args, p, model, source, args.output/f'{i:02d}_{source.stem}', torch, cv2))
        manifest['status'] = 'complete'
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
        print(f'DIAGNOSIS_COMPLETE {args.output.resolve()}', flush=True)
    except Exception as error:
        (args.output/'ERROR.txt').write_text(f'{type(error).__name__}: {error}\nIncomplete output; do not treat as a completed test.\n')
        raise


if __name__ == '__main__':
    main()
