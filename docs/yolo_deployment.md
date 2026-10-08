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

For field failure diagnosis on RTX 4090 / Ubuntu / ROS 2 Humble, see
[yolo_diagnostics.md](yolo_diagnostics.md) for opt-in logs, passive recording and standalone video analysis.
