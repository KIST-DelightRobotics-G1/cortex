# Standard Cortex YOLO deployment

Use `cortex.launch.py` for the robot. Its default `cortex_params.yaml` now enables
real YOLO and fail-closed object checks. `llm_demo.launch.py` explicitly selects
the stub detector, dummy LLM and mock motion modules. A missing model stops the
standard launch with an actionable error; it is never silently replaced by a stub.

## Configuration and model

The default model path is `/models/cortex/yolo26s_fridge_cucumber_continued.pt`.
The selected SHA-256 and v3 presence policy live in
`src/cortex_bringup/config/cortex_params.yaml`. The model stays outside Git/images.
If replacing the weight, update both the path and its `model_sha256`.

```bash
ros2 launch cortex_bringup cortex.launch.py
# Host path / device override:
ros2 launch cortex_bringup cortex.launch.py \
  model:=/absolute/path/yolo26s_fridge_cucumber_continued.pt device:=0
# Camera + detector only, no orchestration or speech:
ros2 launch cortex_bringup cortex.launch.py \
  model:=/absolute/path/yolo26s_fridge_cucumber_continued.pt device:=0 detector_only:=true
# A full site YAML can contain model/device and the other node settings:
ros2 launch cortex_bringup cortex.launch.py params_file:=/absolute/path/site.yaml
```

Copy the full default YAML when creating a site configuration. Empty model/device
launch arguments preserve YAML values. Precedence is: full params_file, optional
detector_profile YAML overlay, explicit model/device launch overrides. Custom
filenames are accepted; `detector_profile` takes a file path.

`cortex_yolo.launch.py` remains a compatibility wrapper around the same standard
launch. Existing `profile:=fridge_detector_v3.yaml` commands still work. It has no
separate node graph. `fridge_detector.yaml` explicitly retains the older policy.

The default device is the string `'0'`. Use `device:=cpu` or `device:=mps` when
appropriate. The default LLM is still `dummy`: configure the real backend,
credentials, VLA exact training sentences and external camera/nav/VLA processes
before full robot tests. Door state and post-condition checking are not included.

## Host installation

Ubuntu 22.04 / ROS 2 Humble, Python 3.10, compatible NVIDIA driver:

```bash
git submodule update --init --recursive
source env.sh
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r requirements-torch-cu126.txt
python3 -m pip install -r requirements.txt -r requirements-detector.txt
colcon build --symlink-install
source install/setup.bash
python3 -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"
```

The GPU wheel pair is Torch 2.7.1 / torchvision 0.22.1, CUDA 12.6, from the
[official PyTorch index](https://pytorch.org/get-started/previous-versions/).
Ultralytics is pinned to 8.4.154; OpenCV 4.11 retains NumPy 1.26 compatibility.
For CPU/macOS, install the corresponding Torch pair instead of the CUDA requirements.
Real selected-weight compatibility and GPU latency must be checked on the target PC.

## Docker

The standard image installs the above CUDA PyTorch pair and detector requirements.
Building it does not require a GPU. Running GPU inference requires an NVIDIA driver
and [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
on the host. `--gpus` exposes the host GPU; it does not install the driver.

```bash
# Put the separately supplied model here first.
mkdir -p "$HOME/models/cortex"
docker/build.sh
docker/run.sh ros2 launch cortex_bringup cortex.launch.py detector_only:=true
```

Defaults: all GPUs exposed, host `~/models/cortex` mounted read-only at
`/models/cortex`, with host networking and existing DDS/credential/cache options.
The model must exist under the expected filename or be overridden in YAML/launch.

```bash
# Choose a host GPU and another model directory at container creation:
CORTEX_GPUS=0 CORTEX_MODEL_DIR=/absolute/path/models \
  docker/run.sh ros2 launch cortex_bringup cortex.launch.py detector_only:=true
# CPU host (still needs the model):
CORTEX_GPUS=none docker/run.sh ros2 launch cortex_bringup cortex.launch.py device:=cpu
# Explicit demo, no model needed (an empty model directory is OK):
CORTEX_GPUS=none docker/run.sh ros2 launch cortex_bringup llm_demo.launch.py
```

Supported CORTEX_GPUS values are `all`, `none`, or a single host GPU index.
With one GPU exposed, YOLO uses container device 0. The script creates the model
directory if missing, but never downloads or creates a model.

A persistent container with old/different GPU, model directory or image settings
is rejected rather than reused. Preserve any needed container-only files and
explicitly recreate it. The script never removes a container automatically.
Credentials, DDS variables and cache mounts still use creation-time settings.

## Checks and limits

In the same ROS environment, query:

```bash
ros2 service call /cortex/detector/check cortex_msgs/srv/CheckTarget "{target: fridge}"
ros2 service call /cortex/detector/check cortex_msgs/srv/CheckTarget "{target: cucumber}"
```

Default checks use confidence 0.25, a 0.6 s / 3-frame window, strict majority and
a hit in the latest frame. PRECHECK retries for at most 1 s after module readiness.
`take_out` requires fridge AND cucumber; `pick` requires only cucumber.

Regression tests cover YAML precedence and missing weights, the standard/compat/demo
launches, GPU/mount arguments and refusal to reuse incompatible containers.
Docker CI runs actual YOLO CPU inference with temporary random weights through
the standard launch, raw ROS input, DetectionArray and CheckTarget, then the existing
mock plan smoke test. This does not benchmark or validate the supplied model's
accuracy, H.264 transport on the robot, or GPU execution.

Known main-derived limitations remain: initial/old IDLE freshness is not fully
validated and module source stamps are not used for stale checking. Those changes
are outside this deployment update; see the handoff notes before robot operation.

## Console diagnostics for field trials

Console messages are chosen per node in `src/cortex_bringup/config/logging.yaml`
(launch argument `logging_file:=/absolute/path.yaml` to use another file). The
shipped file turns on detector `config`, `check` and orchestrator `session`, `plan`,
`precheck`, `subtask`. Remove a topic from a node's `events` list to silence it.
No separate collector, bag, video writer, new topic, or message/service definition
is added. Presence thresholds,
retries and executor transitions are unchanged, including the **1.0 s** precheck timeout.

```yaml
nodes:
  orchestrator_node: {events: [session, plan, precheck, subtask]}
  detector_node:     {events: [config, check]}
```

The file is read at launch; edit it and relaunch cortex. With `docker/run.sh` the
host's `src/cortex_bringup/config` is mounted read-only and launch reads its
`logging.yaml` (`CORTEX_LOGGING_FILE`), so no image rebuild is needed. Nothing is
printed periodically: the detector prints once at start and then only when it
answers a CheckTarget. The orchestrator re-asks every 0.1 s while a target is not
visible; the same answer repeated is printed once (`YOLO_CHECK` within 2 s,
`PRECHECK_RESULT` per step), and `PRECHECK_STATE` carries the final verdict.
Orchestrator prechecks operate in `planner_mode: llm`.

Messages to inspect, in order:

- `YOLO_CONFIG` / `PRECHECK_CONFIG`: model path, configured device and expected hash,
  loaded classes and target mapping, camera transport, inference/check thresholds,
  window/minimum frames/latest-hit policy, and service/precheck timeouts. The device
  field is the requested setting, not a GPU benchmark or proof of actual utilization.
- `YOLO_CHECK`: target, actual effective check threshold/window, samples, hits,
  latest-frame hit, maximum candidate confidence and the decision reason. Evidence
  is captured from the same locked window used for the actual response. The same
  line carries the pipeline state at that moment: `camera_fps` and `infer_hz` over
  the last 2 s, `infer_ms` and `best_confidence` of the **last completed
  inference** (not a new presence decision), and `dropped` — frames rejected since
  start (`h264_no_output` while waiting for a keyframe, `stale_input`,
  `out_of_order`, …). `camera_fps: 0` means the camera stopped, not that the
  object is absent.
- `PRECHECK_RESULT`: plan_id, step index/action, target, service result and
  `service_ms` (one RPC, not the total retry duration). A positive RPC is not final
  subtask permission: the executor still applies the shared precheck deadline.
- `PLAN_STEP` / `PLAN_END`, `PRECHECK_STATE` / `PLAN_CANCEL`, `SUBTASK_SENT`:
  accepted plan traces (including existing rewrites), waiting/final outcome and
  the actual command published. Compare these to distinguish a missing open step
  in the plan from a detector-blocked step. They mirror existing events only.

The following is an illustrative, abbreviated example, **not a measured trial**:

```text
[detector_node] YOLO_CHECK {"target":"cucumber","found":false,"reason":"latest_miss","samples":5,"hits":4,"latest_hit":false}
[orchestrator_node] PRECHECK_RESULT {"plan_id":"p-example","index":2,"action":"pick","target":"cucumber","found":false,"detail":"","service_ms":5.2}
[orchestrator_node] PRECHECK_STATE {"plan_id":"p-example","index":2,"title":"오이 확인 대기","detail":"not_visible"}
```

Here the window contains enough positive frames, but the required newest-frame
hit is missing. The existing wire response keeps `detail=""` for ordinary absence;
the finer `latest_miss` reason exists only in the log. It must not turn absence
into an unavailable response or change fail-open behavior.

Other reasons and limits:

- `no_hits`: no frame passes the effective check threshold; `below_majority`:
  positives are half or less of the usable frames; `latest_miss`: the majority
  passes but the required last-frame hit does not.
- `insufficient_frames`: too few fresh frames; `no_frame` / `stale`: no usable
  inference evidence yet / previously observed evidence has expired. `hits` and
  `latest_hit` are null when those stages of the decision were not evaluated.
- `unsupported_class`, `unknown_target`, `model_not_loaded`, `inference_error`:
  inspect classes, mapping, configuration or model errors.
- `dropped` counters `stale_input`, `future_stamp`, `unstamped`, `out_of_order`,
  `stale_before_inference` and `window_add_rejected` locate rejected frames.
  Source age assumes synchronized camera/receiver clocks.
- `h264_no_output` includes keyframe waiting, normal codec buffering and decoder
  recovery; it alone does not prove corruption or dropped network packets.
- Candidate confidences include only boxes retained by `infer_conf`.
  Null means no retained candidate, not a measured confidence of zero.
- Match detector/orchestrator lines by target and timestamps. CheckTarget has no
  request ID, so simultaneous clients are not guaranteed a unique pairing.
  Logs cannot establish whether an object was truly visible; that needs the video.

To share a field run, capture the existing console in the same ROS environment:

```bash
set -o pipefail
ros2 launch cortex_bringup cortex.launch.py 2>&1 | tee -i yolo-console.log
```

Keep the actual site/model/device launch arguments if used. Pass the complete
file (startup through shutdown) and trial conditions, including the time of the
failed instruction. Use a new filename for each run. The file can contain
transcribed speech. Docker users should copy it out before discarding the container.

## Wrist evidence before carrying subtasks (opt-in)

The standard launch keeps the ego detector and can start a second instance named
`wrist_detector_node`. Its source, `DetectionArray` topic, `CheckTarget` service,
model and frame window are independent. `SubtaskCmd` / `SubtaskState` are unchanged.
The existing perception executable is reused; no camera topics are hard-coded.

This check is added in `planner_mode: llm` before `step_back(fridge)`,
`close(fridge_door)` and `move_to(table)` when the orchestrator's expected
`holding` state is `cucumber`. Each step requires its own fresh wrist check;
a successful check before stepping back does not authorize closing or moving.
The state comes from a matching completed pick/take_out/receive; it survives
step_back, close, navigation, plan completion and a later command. Completed
place/put_in/handover clears it. It is metadata,
not visual proof. A detector miss never clears the requirement. After a Cortex
restart the state is unknown to this mechanism (`none` initially): a standalone
carrying command while already holding an object is not covered by this conditional check.

After the executing module (VLA or nav) becomes IDLE, each PRECHECK attempt
requires only `held_cucumber` on the wrist service for these guarded steps.
This replaces the usual ego fridge requirement for close; it adds a wrist check
for step_back and move_to(table). Open, pick, take_out, other destinations and
steps without expected cucumber holding retain their existing checks. In particular,
ordinary table navigation before picking an object still works without wrist evidence.
Only wrist source frames captured at or after entry to this PRECHECK can
contribute. The window still requires
enough distinct frames, a strict majority and a latest-frame hit. Old positive
answers are not latched across retries. Missing or unavailable wrist evidence
blocks dispatch even if `detector_fail_open` is enabled elsewhere. The existing
precheck timeout bounds waiting; failure reports the wrist target and ends the plan.

The ego detector keeps running. When precheck logging is enabled, the orchestrator
requests one ego `cucumber` observation per PRECHECK asynchronously for reference.
It does not wait for that answer. An ego miss, stale frame, missing service or
service timeout cannot block or authorize a guarded step, or consume its
waiting budget. The auxiliary request is bounded by `detector_call_timeout_s`;
late results keep the original plan/step context and never change execution.
Ego evidence cannot substitute for missing wrist evidence.

### Site configuration and limits

The feature is off by default. Configure these sections in a copy of
`cortex_params.yaml` (or a `detector_profile` overlay):

- `orchestrator_node.ros__parameters.holding_precheck_enabled: true`
- `holding_detector_service`: match the wrist detector's service (default
  `/cortex/wrist_detector/check`); `holding_max_age_s` bounds source age on return.
- Keep a positive `precheck_timeout_s` (allow time for the new frame window).
- `wrist_detector_node.ros__parameters.camera_topic` defaults to
  `/kist/camera/right_wrist/color/h264`, with `camera_transport: h264`, matching
  ext-sensor-io's `right_wrist` camera name. The ego detector keeps
  `/kist/camera/head/color/h264`. Confirm stream reception, timestamp clock and
  image orientation; override the topic if the site renames the camera.
- `model` and `model_sha256` default to the same YOLO26s 3rd continued weight as
  the ego detector: `/models/cortex/yolo26s_fridge_cucumber_continued.pt`. The
  existing Docker model mount serves both nodes; no second copy of the file is
  needed. Each node loads its own model instance and retains separate camera,
  confidence and window settings. If using a different weight, update that node's
  path and hash together. The global launch `model:=` override remains **ego-only**;
  `device:=` applies to both detectors. Models are not stored in the repository
  or downloaded at launch.
- `check_roi`: optional `[left, top, right, bottom]` in normalized full-image
  coordinates. `roi_min_overlap: 0.0` disables the spatial filter. To enable it,
  calibrate the region and set a positive overlap: it is intersection area
  divided by cucumber box area. A candidate is filtered **before** choosing the
  best box of its class; rejected candidates still count as missed frames. The
  published DetectionArray retains all boxes for debugging.
- Calibrate `default_min_confidence`, `infer_conf`, `min_frames` and `window_s` on
  separately recorded positive and empty-gripper videos. `infer_conf` must not
  exceed the desired check threshold. Keep `require_latest_hit: true` and
  `allow_unstamped: false` for this branch's wrist check.

Launch rejects missing weights, missing/shared camera topics or services, a demo
stub, inconsistent target mapping, or invalid ROI coordinates/overlap. With the
standard model file already mounted, enable the guard through
`holding_precheck_enabled`; the shipped camera and model paths need no edits
unless the deployment uses different locations.
The wrist default check threshold is 0.70, with `infer_conf: 0.25` and no ROI filter;
the ego detector keeps its separate 0.25 check threshold. The wrist setting is an
offline-video candidate, not a validated grasp detector for all camera conditions.
Confirm it on original site recordings before robot use.

`held_cucumber` currently means **temporal cucumber evidence in the configured
wrist view (or ROI when enabled)**, not a measured gripping force or a trained
held/not-held state.
The existing model produced persistent empty-gripper false positives in the
supplied composite clip. ROI and voting cannot prove holding or eliminate those
errors. The two-camera connection can be tested now; enabling it for robot motion
requires validation with original wrist recordings (held, empty, nearby but not
held, and release) and, if needed, a fine-tuned model. No automatic ego fallback
substitutes for missing wrist evidence.

First verify inputs without the orchestrator:

```bash
ros2 launch cortex_bringup cortex.launch.py params_file:=/path/to/site.yaml detector_only:=true
ros2 service call /cortex/wrist_detector/check cortex_msgs/srv/CheckTarget "{target: held_cucumber}"
```

A direct service call reads the rolling window; the orchestrator additionally
restricts evidence to the current PRECHECK. After calibration, run the same
standard launch without `detector_only:=true` to test the whole path. Two model
instances use additional GPU memory and inference time; measure on the target
RTX 4090 alongside VLA rather than assuming the earlier single-camera latency.

Logs use the existing `logging.yaml` / `/cortex/events` / session JSONL recorder.
`wrist_detector_node` emits `YOLO_CONFIG` and `YOLO_CHECK`, including ROI settings;
`PRECHECK_RESULT` identifies `source: right_wrist` and `target: held_cucumber`.
The advisory ego result uses `PRECHECK_AUXILIARY`, `source: ego` and
`required: false`; it may be logged after `SUBTASK_SENT`. Only the wrist result
and the executor's `PRECHECK_STATE` authorize these guarded steps. Ego checks for
other subtasks still use `PRECHECK_RESULT` and remain required as before.
