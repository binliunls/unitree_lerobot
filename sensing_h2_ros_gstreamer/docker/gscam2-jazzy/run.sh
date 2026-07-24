#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

IMAGE_NAME="${IMAGE_NAME:-sensing-gscam2-jazzy:latest}"
CONTAINER_NAME="${CONTAINER_NAME:-sensing-gscam2}"
LEFT_DEVICE="${LEFT_DEVICE:-/dev/video2}"
RIGHT_DEVICE="${RIGHT_DEVICE:-/dev/video3}"
NVIDIA_RUNTIME="${NVIDIA_RUNTIME:-nvidia}"

DOCKER_ARGS=(
  --rm
  --name "${CONTAINER_NAME}"
  --network host
  --ipc host
  --privileged
  -e NVIDIA_VISIBLE_DEVICES=all
  -e NVIDIA_DRIVER_CAPABILITIES=all
  -v "${REPO_ROOT}:/workspace/sensing_wrist_gstreamer:ro"
)

if [[ -t 0 && -t 1 ]]; then
  DOCKER_ARGS+=(-it)
fi

if [[ "${NVIDIA_RUNTIME}" != "none" ]]; then
  DOCKER_ARGS+=(--runtime "${NVIDIA_RUNTIME}")
fi

for device in "${LEFT_DEVICE}" "${RIGHT_DEVICE}"; do
  if [[ -e "${device}" ]]; then
    DOCKER_ARGS+=(--device "${device}:${device}")
  else
    echo "Warning: ${device} does not exist on the host; container will still start." >&2
  fi
done

if [[ -S /tmp/argus_socket ]]; then
  DOCKER_ARGS+=(-v /tmp/argus_socket:/tmp/argus_socket)
fi

if [[ -f /etc/nv_tegra_release ]]; then
  DOCKER_ARGS+=(-v /etc/nv_tegra_release:/etc/nv_tegra_release:ro)
fi

for env_name in ROS_DOMAIN_ID RMW_IMPLEMENTATION GSCAM_CONFIG GST_DEBUG; do
  if [[ -n "${!env_name:-}" ]]; then
    DOCKER_ARGS+=(-e "${env_name}=${!env_name}")
  fi
done

if [[ $# -eq 0 ]]; then
  set -- bash
fi

exec docker run "${DOCKER_ARGS[@]}" "${IMAGE_NAME}" "$@"
