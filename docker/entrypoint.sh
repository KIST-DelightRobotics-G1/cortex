#!/usr/bin/env bash
# ROS + the built workspace, then whatever was asked for (default: a shell).
set -e
source /opt/ros/humble/setup.bash
source /workspace/cortex/install/setup.bash
exec "$@"
