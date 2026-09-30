# Selected YOLO26s model and bounded prechecks [SYS-REQ-44]

This profile connects **YOLO26s 3차 9/30 영상 기반 파인튜닝 (2차 모델 연속 학습)**
to the existing LLM-mode object-presence precheck. It does not verify door state,
reachability, graspability or task completion. Subtask/CheckTarget message fields
are unchanged. The VLA runner still owns DONE/FAILED.

## Run on the workstation

Use the deployment Python/Torch/ROS environment described in `detector_integration.md`.
Copy the selected weight to the workstation; the repository contains no model binary.
The selected file is `yolo26s_fridge_cucumber_continued.pt`, with SHA-256:

`3d275b84378da06dc9202b4d40962f49f8d88d18062357d1d167abab10d6f920`

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch cortex_bringup cortex_yolo.launch.py \
  model:=/absolute/path/yolo26s_fridge_cucumber_continued.pt \
  profile:=fridge_detector_v3.yaml device:=0 detector_only:=true
```

Omit `detector_only:=true` to start the Cortex node graph. Motion modules are
external; this launch does not provide the camera bridge or VLA runner. Configure
camera topic/transport and actual LLM backend in `cortex_params.yaml` for the site.
The ordinary `cortex.launch.py`/demo profiles still use the existing stub settings;
the dedicated YOLO launch now defaults to this versioned v3 profile. Use
`profile:=fridge_detector.yaml` explicitly to reproduce the earlier detector profile.
A wrong weight fails the hash check, yields model_not_loaded, and cannot dispatch
under the v3 profile. `device:=cpu` is available without CUDA.

## Decision sequence

1. A single detector worker processes recent camera frames at a target 8 Hz.
2. Candidate boxes use confidence >= 0.25. Each frame contributes at most one vote
   per target. The recent 0.6-second window needs at least 3 distinct frames.
3. An object passes only if a strict majority has confidence >= 0.25 **and the
   latest processed frame also has a qualifying detection**. This prevents a prior
   positive majority from overriding an already processed negative frame.
4. Before dispatching a subtask, Cortex queries `CheckTarget`. If it cannot confirm
   the target, it enters PRECHECK and retries at intervals of at least 0.1 seconds,
   with a 1.0-second deadline. No VLA command is sent while waiting.
5. A positive result before the deadline dispatches once. Persistent absence or
   unavailable input aborts the plan. Unknown/unsupported targets and invalid
   requests fail immediately. Fail-open is disabled.
6. A user stop or replacement plan can end PRECHECK without waiting for a module
   that has received no command. DONE received before dispatch cannot advance it.

The deadline uses the executor clock; timer granularity and a service call already
in progress can delay the failure notification. Responses arriving after the
deadline cannot authorize dispatch. Existing service timeout is 0.3 seconds.
This is a start-condition check, not a continuous stop controller during VLA motion.

`fridge_door` and `fridge` map to `refrigerator`; `cucumber` maps to `cucumber`.
`user` maps to unsupported `person`, so handover requiring a person is blocked.
A positive refrigerator result does not distinguish open from closed doors.

## Comparison and reproducibility

Confidence 0.25 is retained from the report as a provisional baseline, not selected
by optimizing the reused test set. Compare 0.25/0.4 and majority-only/latest-hit
policies using the same saved predictions. This is exploratory analysis; validate
final settings on separate KIST execution records.

```bash
python3 -m pytest -q
python3 scripts/verify_detector_offline.py \
  --model /absolute/path/yolo26s_fridge_cucumber_continued.pt \
  --profile src/cortex_bringup/config/fridge_detector_v3.yaml \
  --video /absolute/path/test_video.mp4 \
  --device cpu --output /tmp/v3-inference.json
python3 scripts/replay_preconditions.py \
  --evidence /tmp/v3-inference.json \
  --profile src/cortex_bringup/config/fridge_detector_v3.yaml \
  --near-timing /absolute/path/continued_v3.json \
  --visibility /absolute/path/temporal_annotations.json \
  --output /tmp/v3-replay.json
```

Outputs must not already exist. The second script uses the reviewed near test
video: `near-timing` supplies original PTS and `visibility` the existing frame
intervals. It validates the selected weight hash and inference configuration.

The subtask replay uses real Cortex planner validation, presence policy and executor.
It injects the plan `open(fridge_door) -> pick(cucumber) -> close(fridge_door)`.
VLA heartbeats and completion times are **mocked**, with manual scenario boundaries:
open DONE at 10.6 s, pick DONE at 23.8 s, close DONE at 25.2 s. Separate scenarios
request pick too early, omit completion, or stop during PRECHECK. These are software
integration checks, not successful robot trials or estimated action durations.

The presence comparison evaluates queries at completed inference timestamps,
roughly 8 Hz, against visibility at those timestamps. Counts differ from the
report's 105 images and 815 dense frames. An unavailable response cannot authorize.
Ambiguous visibility frames are excluded. The latest-frame requirement can increase
misses while reducing delayed disappearance; retries may recover, but do not
eliminate every missed opportunity. Between inference updates an earlier verdict
may remain available, and camera/ROS transport adds unmeasured delay.

ROS Humble build/generated-message tests, actual camera timing/QoS, live VLA
integration and RTX 4090 end-to-end latency remain to be measured.
