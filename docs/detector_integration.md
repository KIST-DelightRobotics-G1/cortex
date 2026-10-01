# Local refrigerator / cucumber detector [SYS-REQ-44]

The real detector profile loads an explicit local YOLO detection weight and uses
its class names, including custom `refrigerator` and `cucumber` classes. Existing
`DetectionArray` and `CheckTarget` fields are unchanged. Default demo launches
still use the explicit `always` stub; select the real profile below.

For the selected 3rd continued model, use [the v3 profile](detector_v3.md).
The settings and results below describe the earlier profile.
For adaptation to the latest main's H.264 input and DONE/IDLE lifecycle, see
[main integration verification](detector_main_verification.md).

## Install and launch

On the supported Ubuntu 22.04 / ROS 2 Humble system Python 3.10 environment:

```bash
python3 -m pip install -r requirements.txt -r requirements-detector.txt
colcon build --symlink-install
source install/setup.bash
ros2 launch cortex_bringup cortex_yolo.launch.py profile:=fridge_detector.yaml model:=/absolute/path/best.pt device:=0 detector_only:=true
```

Use `device:=cpu` without CUDA. MPS was used only for local macOS verification.
The local model file is required; this profile never downloads a missing weight.
Install the appropriate platform Torch build. Omit `detector_only:=true` to start
the normal Cortex node graph with this detector and fail-closed prechecks.
Camera topic/transport are inherited from `cortex_params.yaml`; adjust those to
the actual bridge publisher. Weights are external assets, not committed binaries.

## Contract and decision policy

- A single inference worker samples the latest camera frame, targeting 8 Hz.
  Actual throughput depends on inference latency; frames can be skipped.
- Camera subscription uses best-effort QoS, depth 1. H.264 from ext-sensor-io is
  now the default, decoded serially while retaining each decoded frame's PTS.
  Raw `rgb8`/`bgr8` images support row padding; compressed images decode to BGR.
- Published boxes are normalized `cx, cy, w, h` in [0, 1], with the weight's
  class label and confidence. Image source time/frame ID are retained.
- Inference retains confidence >= 0.25. Presence defaults to confidence >= 0.4
  and a strict majority over a 0.6-second window containing at least 3 distinct
  fresh source frames. Duplicate boxes do not create extra votes.
- Source age includes arrival/decode/inference latency. Source and receiver
  clocks must be synchronized, or share ROS `/clock`. Stale frames, frames
  >0.1 seconds in the future and zero timestamps are rejected by default.
  `allow_unstamped: true` explicitly substitutes receipt ROS time for zero stamps.
  Restart the node after source clock resets: stamps must increase monotonically.
- `CheckTarget.stamp` is the source time of the returned highest-confidence box,
  not the request time. A negative result has empty box/label and zero stamp.
- No frame, insufficient frames, stale data, model/inference errors and unsupported
  classes return `found=false` with a detail string. Inference failure clears
  prior evidence; recovery requires fresh warmup. The real profile sets
  `detector_fail_open: false` so unavailable checks cannot dispatch a VLA step.
- These two-class weights cannot detect `person`: the mapped `user` target returns
  `unsupported_class`, so actions requiring user presence are blocked. Use a model
  containing the required class or a separate verified detector before handover.
- `fridge_door` maps to `refrigerator` for **object presence only**. A positive
  result does not establish door open/closed state, reachability or graspability.
  No tracking or box smoothing is added. Majority voting can delay disappearance
  decisions and does not guarantee removal of the observed long detection gaps.

The pure detector policy lives in `detection.py` and is shared by the ROS node,
offline verifier and tests. Inference callbacks are mutually exclusive; camera
and check callbacks can run while a large model is busy.

## Verification completed locally (2026-09-27)

macOS Apple M1, Python 3.12, Ultralytics 8.4.154, Torch 2.14.0, MPS. ROS was not
installed. `python -m pytest -q`: **55 passed, 1 skipped**. The skipped test uses
real generated ROS messages and a fake inference adapter when Humble is available;
CI now runs the root tests after building the interfaces. CI itself has not been
run for this local branch.

Tests cover class mapping, normalized boxes, confidence gates, strict majority,
duplicate frames, expiry, source timestamps, unavailable/fault recovery, and the
real pure executor's dispatch/block behavior. The cancel adapter was also fixed
to avoid writing a nonexistent `SubtaskCmd.header` field and regression tested.

Real weights and both new KIST source videos were replayed through the shared
YOLO adapter -> presence window -> response fields -> real executor command sink.
No ROS/DDS or physical command publisher was involved. Every decision was checked
against dispatch, and stale evidence blocked dispatch at each video's end.

| Model | Video | Processed frames | Fridge allow / block / unavailable | Cucumber allow / block / unavailable | Mean inference |
|---|---|---:|---|---|---:|
| YOLO26s | 1.nav.mp4 | 172 | 86 / 84 / 2 | 0 / 170 / 2 | 27.7 ms |
| YOLO26s | 2.3 open_fridge.mp4 | 204 | 197 / 5 / 2 | 84 / 118 / 2 | 25.2 ms |
| YOLO26x | 1.nav.mp4 | 131 | 53 / 72 / 6 | 0 / 125 / 6 | 166.3 ms |
| YOLO26x | 2.3 open_fridge.mp4 | 160 | 153 / 5 / 2 | 68 / 90 / 2 | 161.6 ms |

These are decision counts, **not accuracy metrics**. Each query starts an isolated
executor to verify its gate. The approximate single-worker replay advances to the
next source frame after max(125 ms, measured inference duration); it excludes model
warmup and does not simulate network, ROS scheduling or camera delivery. YOLO26x
exceeded the 125 ms inference budget on this Mac. No action completion was tested.

Verified weights (both class 0 refrigerator / class 1 cucumber):

- YOLO26s: `s_newgeometry_fridge_cucumber/weights/best.pt`, SHA-256
  `a631ca0ade9725070222ac283fce2816c8a43c0b54c0ff4d6dda47963d805350`
- YOLO26x: `x_newgeometry_fridge_cucumber/weights/best.pt`, SHA-256
  `d0b7f9f0be036c8fe2406149b30f27c22b5dc9fd4cc30a026291f5ccff566fe5`

Weight root: `/Volumes/T7/g1_experiments/fridge_new_geometry_20260924/runs`.
Source video root: `/Volumes/T7/g1_data`.
Local raw results: `../output/cortex_yolo_integration_20260927/{yolo26s,yolo26x}.json`
(relative to this repository root). Result JSON retains full weight/video paths,
hashes, per-frame boxes, verdicts and simulated command records.

Reproduce (output path must not exist):

```bash
python3 scripts/verify_detector_offline.py \
  --model /absolute/path/best.pt \
  --video '/absolute/path/1.nav.mp4' \
  --video '/absolute/path/2.3 open_fridge.mp4' \
  --device cpu --output /tmp/detector-verification.json
```

Remaining verification: Humble build/generated-message test, actual camera QoS
and clock sync, DDS/service transport, target-PC end-to-end latency and real
nav/VLA interaction. Local success establishes software gate compatibility;
it does not establish readiness for autonomous robot operation.
