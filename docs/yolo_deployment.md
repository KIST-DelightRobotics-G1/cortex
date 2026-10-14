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
or message/service definition is added. Presence thresholds, retries and executor
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
