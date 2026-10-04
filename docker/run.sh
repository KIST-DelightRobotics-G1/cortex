#!/usr/bin/env bash
# Standard GPU runtime. CORTEX_GPUS=none selects CPU/demo (launch device:=cpu).
# CORTEX_MODEL_DIR (default ~/models/cortex) mounts read-only at /models/cortex.
# Env, mounts and GPU assignment are fixed at container creation.
set -euo pipefail

CONTAINER=kist-cortex
IMAGE=kist-cortex
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${CORTEX_ENV_FILE:-${REPO_ROOT}/.env}"
TTS_CACHE_DIR="${TTS_CACHE_DIR:-${HOME}/.cache/cortex_tts}"
MODEL_DIR="${CORTEX_MODEL_DIR:-${HOME}/models/cortex}"
GPUS="${CORTEX_GPUS:-all}"
CMD=("$@")
[ ${#CMD[@]} -eq 0 ] && CMD=(/bin/bash)

if [[ ! "$GPUS" =~ ^(all|none|[0-9]+)$ ]]; then
    echo "CORTEX_GPUS must be all, none, or one host GPU index (e.g. 0)." >&2
    exit 1
fi
mkdir -p "$MODEL_DIR" "$TTS_CACHE_DIR"
MODEL_DIR="$(cd "$MODEL_DIR" && pwd)"
# Docker --mount uses comma-separated fields.
if [[ "$MODEL_DIR" == *,* ]]; then
    echo "CORTEX_MODEL_DIR cannot contain a comma." >&2
    exit 1
fi

existing="$(docker ps -aq -f name=^${CONTAINER}$)"
if [ -n "$existing" ]; then
    old_gpus="$(docker inspect -f '{{ index .Config.Labels "cortex.runtime.gpus" }}' "$CONTAINER")"
    old_model="$(docker inspect -f '{{ index .Config.Labels "cortex.runtime.models" }}' "$CONTAINER")"
    old_image="$(docker inspect -f '{{.Image}}' "$CONTAINER")"
    image_id="$(docker image inspect -f '{{.Id}}' "$IMAGE")"
    if [ "$old_gpus" != "$GPUS" ] || [ "$old_model" != "$MODEL_DIR" ] || [ "$old_image" != "$image_id" ]; then
        echo "Existing $CONTAINER has different/legacy GPU, model mount or image settings." >&2
        echo "Preserve needed container files, then recreate it; it has NOT been removed." >&2
        exit 1
    fi
    echo "Reusing $CONTAINER: env file, DDS and cache settings remain those from creation." >&2
    if [ -z "$(docker ps -q -f name=^${CONTAINER}$)" ]; then
        docker start "$CONTAINER" >/dev/null
    fi
    exec docker exec -it "$CONTAINER" /entrypoint.sh "${CMD[@]}"
fi

GPU_ARGS=()
if [ "$GPUS" == all ]; then
    GPU_ARGS=(--gpus all)
elif [ "$GPUS" != none ]; then
    GPU_ARGS=(--gpus "device=$GPUS")
fi
ENV_ARGS=()
if [ -f "$ENV_FILE" ]; then
    ENV_ARGS=(--env-file "$ENV_FILE")
else
    echo "WARNING: $ENV_FILE not found — configure STT/TTS/LLM credentials separately." >&2
fi
exec docker run -it --name "$CONTAINER" \
    --network host \
    "${GPU_ARGS[@]}" \
    "${ENV_ARGS[@]}" \
    --label "cortex.runtime.gpus=$GPUS" \
    --label "cortex.runtime.models=$MODEL_DIR" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e DDS_PEER_IP="${DDS_PEER_IP:-192.168.123.164}" \
    -e DDS_ROBOT_IP="${DDS_ROBOT_IP:-192.168.123.161}" \
    --mount "type=bind,src=$MODEL_DIR,dst=/models/cortex,readonly" \
    -v "${TTS_CACHE_DIR}":/root/.cache/cortex_tts \
    "$IMAGE" "${CMD[@]}"
