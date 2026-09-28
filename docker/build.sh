#!/usr/bin/env bash
# Build the cortex image (run from anywhere).
#
# The source is baked in, so the g1_onboard_msgs submodule has to be checked
# out first — .git is not in the build context.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

git submodule update --init --recursive
exec docker build -f docker/Dockerfile -t kist-cortex "$@" .
