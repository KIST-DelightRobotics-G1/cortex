# Workstation container for the cortex cognitive plane (ROS 2 Humble).
#
# Self-contained, same pattern as kist-gearsonic-inference and
# kist-vla-inference: dependencies are installed INSIDE the image, the repo
# source is baked in via COPY, and the workspace is built and tested at
# image-build time — `docker run` gives a ready runtime; no bind mount, no
# in-container setup.
#
# Deliberately left out (first cut):
#   detector_node's YOLO backend — no torch/ultralytics, no GPU. The node still
#     runs with `backend: always` and the orchestrator's precheck passes.
#   the display renderer — cortex-gui ships its own container; this one only
#     serves the websocket (gui_bridge_node, :8081).
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
    && rm -rf /var/lib/apt/lists/*

# ─── python deps ──────────────────────────────────────────────────────────────
# Before the source COPY so code edits never re-download. pytest>=7 for the
# same reason as CI (the LLM SDKs pull anyio, whose plugin needs pytest 7).
WORKDIR /workspace/cortex
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt "pytest>=7,<9"

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
    DDS_PEER_IP=192.168.123.164

# `docker exec` skips the entrypoint, so interactive shells source it too.
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash && source /workspace/cortex/install/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
