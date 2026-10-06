# Workstation container for the cortex cognitive plane (ROS 2 Humble).
#
# Self-contained, same pattern as kist-gearsonic-inference and
# kist-vla-inference: dependencies are installed INSIDE the image, the repo
# source is baked in via COPY, and the workspace is built and tested at
# image-build time — `docker run` gives a ready runtime; no bind mount, no
# in-container setup.
#
# Includes YOLO and CUDA 12.6 PyTorch. Runtime GPU access is set by run.sh.
# Weights are mounted externally; no model or credential is baked into the image.
# cortex-gui ships the display renderer separately.
#
# DDS: Humble's rmw_cyclonedds_cpp is CycloneDDS 0.10.x, the same generation as
# the robot bus (unitree_sdk2 / gearsonic / vla-inference pin 0.10.2).
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ─── system deps ──────────────────────────────────────────────────────────────
# ament_lint_*: cortex_msgs' CMakeLists finds them when BUILD_TESTING is on.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    python3-colcon-common-extensions \
    python3-pip \
    libgl1 libglib2.0-0 \
    nano vim \
    && rm -rf /var/lib/apt/lists/*

# ─── python deps ──────────────────────────────────────────────────────────────
# Before the source COPY so code edits never re-download. pytest>=7 for the
# same reason as CI (the LLM SDKs pull anyio, whose plugin needs pytest 7).
WORKDIR /workspace/cortex
COPY requirements.txt requirements-detector.txt requirements-torch-cu126.txt ./
RUN pip3 install --no-cache-dir -r requirements-torch-cu126.txt && \
    pip3 install --no-cache-dir -r requirements.txt -r requirements-detector.txt "pytest>=7,<9" && \
    python3 -c "import torch, torchvision, ultralytics; assert torch.version.cuda == '12.6'; print('YOLO CUDA runtime installed')"

# ─── Unitree SDK (speaker_node -> the robot speaker via AudioClient) ──────────
# unitree_sdk2py is pure Python on the cyclonedds binding. That binding is built
# against ROS's OWN CycloneDDS so rclpy and the SDK share one libddsc in the
# speaker_node process (two copies would each run their own domain 0). ROS puts
# the headers and library one level deeper than the binding's build looks, hence
# the /opt/ros-cyclonedds shim; CYCLONEDDS_HOME points the binding at it at run
# time too. --no-deps: the SDK would re-pin cyclonedds and pull opencv.
ARG UNITREE_SDK2PY_REF=master
RUN apt-get update && apt-get install -y --no-install-recommends git && rm -rf /var/lib/apt/lists/* && \
    source /opt/ros/humble/setup.bash && \
    V=$(dpkg-query -W -f='${Version}' ros-humble-cyclonedds | cut -d- -f1) && \
    INC=$(dirname "$(dirname "$(find /opt/ros/humble/include -path '*/dds/dds.h' | head -1)")") && \
    LIB=$(dirname "$(find /opt/ros/humble/lib -name 'libddsc.so' | head -1)") && \
    mkdir -p /opt/ros-cyclonedds && ln -s "$INC" /opt/ros-cyclonedds/include && \
    ln -s "$LIB" /opt/ros-cyclonedds/lib && ln -s /opt/ros/humble/bin /opt/ros-cyclonedds/bin && \
    CYCLONEDDS_HOME=/opt/ros-cyclonedds pip3 install --no-cache-dir --no-binary cyclonedds "cyclonedds==${V}" && \
    git clone --depth 1 --branch "${UNITREE_SDK2PY_REF}" \
        https://github.com/unitreerobotics/unitree_sdk2_python.git /tmp/unitree_sdk2_python && \
    pip3 install --no-cache-dir --no-deps /tmp/unitree_sdk2_python && rm -rf /tmp/unitree_sdk2_python && \
    CYCLONEDDS_HOME=/opt/ros-cyclonedds python3 -c \
        "from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient; print('unitree_sdk2py OK')"
ENV CYCLONEDDS_HOME=/opt/ros-cyclonedds

# ─── source + build + test ────────────────────────────────────────────────────
# .dockerignore keeps build/ install/ log/ .git out of the context. The
# g1_onboard_msgs submodule must be checked out on the host (build.sh does it).
COPY . .
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    colcon test && colcon test-result --verbose && \
    rm -rf build log

# ─── runtime ──────────────────────────────────────────────────────────────────
# Same values env.sh sets on a bare host; override with `-e` (run.sh passes
# ROS_DOMAIN_ID / DDS_PEER_IP through).
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///workspace/cortex/config/cyclonedds.xml \
    ROS_DOMAIN_ID=0 \
    DDS_PEER_IP=192.168.123.164 \
    DDS_ROBOT_IP=192.168.123.161

# `docker exec` skips the entrypoint, so interactive shells source it too.
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash && source /workspace/cortex/install/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
