#!/usr/bin/env bash
# Launch (or re-attach to) a persistent named container — same convention as
# kist-gearsonic-inference and kist-vla-inference: reuse it across sessions
# until you `docker rm kist-cortex`.
#
#   docker/run.sh                          a shell inside (ROS already sourced)
#   docker/run.sh ros2 launch cortex_bringup cortex.launch.py
#   docker/run.sh ros2 launch cortex_bringup llm_demo.launch.py backend:=gemini speech:=true
#
#   --network host    DDS discovery toward the NX, nav and VLA; gui_bridge :8081
#   no --gpus         detector_node runs without YOLO in this image
#
# Credentials (STT / TTS / LLM keys) are never baked into the image. They come
# from an env file on the host, read at container creation:
#   CORTEX_ENV_FILE   default <repo>/.env (template: .env.example)
# Use GOOGLE_APPLICATION_CREDENTIALS_B64 there — a host file path means nothing
# inside the container.
#
# Mounts:
#   TTS_CACHE_DIR     default ~/.cache/cortex_tts -> /root/.cache/cortex_tts
#                     the synthesized-sentence cache survives image rebuilds
#
# Iterative dev: add  -v "$(pwd)":/workspace/cortex  to shadow the baked
# source with your working copy, then `colcon build` inside.
set -euo pipefail

CONTAINER=kist-cortex
IMAGE=kist-cortex
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${CORTEX_ENV_FILE:-${REPO_ROOT}/.env}"
TTS_CACHE_DIR_GIVEN="${TTS_CACHE_DIR:-}"
TTS_CACHE_DIR="${TTS_CACHE_DIR:-${HOME}/.cache/cortex_tts}"
CMD=("$@")
[ ${#CMD[@]} -eq 0 ] && CMD=(/bin/bash)

# The env file, mounts and -e values apply only at container CREATION.
warn_reuse() {
    if [ -n "${CORTEX_ENV_FILE:-}${TTS_CACHE_DIR_GIVEN}${ROS_DOMAIN_ID:-}${DDS_PEER_IP:-}" ]; then
        echo "WARNING: reusing the existing '${CONTAINER}' container — its env file," >&2
        echo "         mounts and DDS settings were fixed at creation and are IGNORED now." >&2
        echo "         To apply them: docker rm -f ${CONTAINER}  # then re-run this script" >&2
    fi
}

if [ "$(docker ps -q -f name=^${CONTAINER}$)" ]; then
    warn_reuse
    exec docker exec -it "${CONTAINER}" /entrypoint.sh "${CMD[@]}"
elif [ "$(docker ps -aq -f name=^${CONTAINER}$)" ]; then
    warn_reuse
    docker start "${CONTAINER}" >/dev/null
    exec docker exec -it "${CONTAINER}" /entrypoint.sh "${CMD[@]}"
fi

ENV_ARGS=()
if [ -f "${ENV_FILE}" ]; then
    ENV_ARGS=(--env-file "${ENV_FILE}")
else
    echo "WARNING: ${ENV_FILE} not found — STT/TTS/LLM will run without credentials" >&2
    echo "         (template: .env.example)" >&2
fi

mkdir -p "${TTS_CACHE_DIR}"
exec docker run -it --name "${CONTAINER}" \
    --network host \
    "${ENV_ARGS[@]}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e DDS_PEER_IP="${DDS_PEER_IP:-192.168.123.164}" \
    -v "${TTS_CACHE_DIR}":/root/.cache/cortex_tts \
    "${IMAGE}" "${CMD[@]}"
