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

The `SYS-REQ-44-yolo-console-diagnostics` branch adds INFO messages to the existing
node loggers. Use the standard launch command after building this branch (rebuild
the image when using Docker). No separate collector, bag, video writer, new topic,
or message/service definition is added. Optional detailed JSONL files are described below. Presence thresholds, retries and executor
transitions are unchanged from main, including its **1.0 s** precheck timeout.

The standard YAML enables the logs. Add the following keys to a custom site YAML
if it predates this branch; parameters are read at node startup:

```yaml
detector_node:
  ros__parameters:
    diagnostics_log_enabled: true
    diagnostics_log_period_s: 2.0
orchestrator_node:
  ros__parameters:
    diagnostics_log_enabled: true
```

Merge these keys into the existing node sections rather than replacing the full
site configuration. Set both enabled flags to false to disable the additional
messages. The summary period must be at least 0.5 s. Normal per-frame processing
does not print a line; the summary is periodic, and each actual CheckTarget request
prints one result. Orchestrator prechecks operate in `planner_mode: llm`.

Messages to inspect, in order:

- `YOLO_CONFIG` / `PRECHECK_CONFIG`: model path, configured device and expected hash,
  loaded classes and target mapping, camera transport, inference/check thresholds,
  window/minimum frames/latest-hit policy, and service/precheck timeouts. The device
  field is the requested setting, not a GPU benchmark or proof of actual utilization.
- `YOLO_STATUS`: receive/accepted-frame/completed-inference rates over the last
  interval, lifetime counters, time since each last event, current age of the
  latest accepted source frame, last inference time and source age at completion.
  `best_confidence` describes the **last completed inference**, not a new presence
  decision; check `inference_completed_age_ms` to avoid reading old scores as fresh.
- `YOLO_CHECK`: target, actual effective check threshold/window, samples, hits,
  latest-frame hit, maximum candidate confidence and the decision reason. Evidence
  is captured from the same locked window used for the actual response.
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
- Status counters `stale_input`, `future_stamp`, `unstamped`, `out_of_order`,
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

## Optional detailed JSONL recording

Keep console diagnostics enabled as above. To additionally record detailed YOLO
evidence, merge these parameters into the existing detector section and restart
the rebuilt node/image:

```yaml
detector_node:
  ros__parameters:
    diagnostics_jsonl_enabled: true
    diagnostics_jsonl_dir: ''              # resolve from ROS log directory
    diagnostics_jsonl_max_bytes: 10485760   # 10 MiB per file
    diagnostics_jsonl_backup_count: 4      # current file plus four backups
```

Recording is **off by default** and independent of `diagnostics_log_enabled`.
With recording off, no recording directory, file or writer thread is created.
Parameters are read at startup; changing a running parameter is not a toggle.

An empty directory uses the first available location:

1. `$ROS_LOG_DIR/yolo_diagnostics`
2. `$ROS_HOME/log/yolo_diagnostics`
3. `~/.ros/log/yolo_diagnostics`

Each node start creates a unique UTC-date/PID/random-suffix subdirectory containing
`events.jsonl`. For example, a root Docker process without overrides writes under
`/root/.ros/log/yolo_diagnostics/<run>/events.jsonl`. The exact path appears once
in the console as `YOLO_JSONL path=...`. Set `diagnostics_jsonl_dir` to an absolute
directory such as `/logs/yolo_diagnostics` to select another location. In Docker,
that path is **inside the container**: bind-mount the host destination there, or
copy out the complete run directory before removing the container.

One UTF-8 JSON object is written per line. Each has `schema_version: 1`,
`session_id`, a per-process `sequence`, Unix `wall_time_ns`, `monotonic_ns`,
`dropped_total`, `event` and `data`. Node events put `ros_time_ns` in `data`.
Use source stamps to join image evidence, ROS timestamps to compare node logs,
and monotonic time for elapsed durations on the same host.

Recorded events:

- `YOLO_CONFIG`, `YOLO_STATUS`, `YOLO_CHECK`: existing detector diagnostics.
  Each check additionally records `frames`, with source stamp, age, maximum
  retained candidate confidence and threshold hit for every frame actually
  considered by the locked presence window. An empty list means no frame
  evidence was available; an early failure such as insufficient frames still
  retains available per-frame evidence without claiming a valid verdict.
- `FRAME_ACCEPTED`: effective/original source stamp, camera frame ID, dimensions
  and source age. Acceptance does not guarantee inference: the existing detector
  samples the latest image at its configured rate.
- `FRAME_REJECTED`, `INPUT_NO_FRAME`: reason and source/packet stamp for stale,
  invalid or out-of-order input and decoder cases. An H.264 packet with no output
  may be normal codec buffering; it is not by itself a bad frame.
- `INFERENCE_RESULT`: source stamp, dimensions, inference duration, source age at
  completion, `window_added` and every retained detection. Box fields
  `cx/cy/w/h` are normalized image coordinates, not pixels.
- `INFERENCE_SKIPPED`, `INFERENCE_ERROR`: source stamp and skip/error detail.

Example shape (illustrative, abbreviated):

```json
{"schema_version":1,"event":"YOLO_CHECK","data":{"ros_time_ns":1791940957000000000,"target":"cucumber","found":false,"reason":"latest_miss","samples":3,"hits":2,"latest_hit":false,"frames":[{"source_stamp_ns":1791940956700000000,"age_ms":300.0,"max_candidate_confidence":0.8,"hit":true},{"source_stamp_ns":1791940956825000000,"age_ms":175.0,"max_candidate_confidence":0.9,"hit":true},{"source_stamp_ns":1791940956950000000,"age_ms":50.0,"max_candidate_confidence":null,"hit":false}]}}
```

The writer uses a bounded queue of 256 events and performs serialization, flushes
and rotation on a background thread. A full queue drops new records instead of
waiting for disk; `YOLO_STATUS.jsonl` and `YOLO_JSONL_CLOSED` expose written,
dropped, queued and error counts. Oversized or non-JSON-compatible records are
also counted as dropped. A file error disables recording and prints a warning;
detection and CheckTarget continue. Shutdown drains for at most two seconds;
forced shutdown or a blocked disk can lose queued records. This is diagnostic
recording, not a lossless recorder, and its runtime overhead still needs checking
on the robot.

At default limits, a run retains at most five 10 MiB files (50 MiB total).
`events.jsonl` is newest; `.1` is the preceding file, then `.2` through `.4`.
Old data, including the initial config record, can rotate out. Collect all files
and the console log from startup through shutdown. Limits apply **per run**;
earlier run directories are retained and must be archived/removed separately.

This file does not record images, audio, the orchestrator's plan events or
candidates discarded by `infer_conf`. Share the existing console log and matching
camera recording with it. CheckTarget has no end-to-end request/plan ID; the JSONL
sequence is only a local record number. Correlate with `PRECHECK_RESULT` by target
and timestamp, with the existing limitation for simultaneous service clients.
