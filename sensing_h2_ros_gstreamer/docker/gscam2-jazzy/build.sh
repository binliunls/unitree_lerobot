#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE_NAME="${IMAGE_NAME:-sensing-gscam2-jazzy:latest}"
GSCAM2_SRC="${GSCAM2_SRC:-/home/unitree/dev/gscam2}"
BASE_IMAGE="${BASE_IMAGE:-arm64v8/ubuntu:24.04}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DOCKER_PLATFORM="${DOCKER_PLATFORM:-linux/arm64}"
BUILD_CONTEXT="${BUILD_CONTEXT:-${SCRIPT_DIR}/.build-context}"

if [[ ! -d "${GSCAM2_SRC}" ]]; then
  echo "gscam2 source directory not found: ${GSCAM2_SRC}" >&2
  echo "Set GSCAM2_SRC=/path/to/gscam2 or clone it at /home/unitree/dev/gscam2." >&2
  exit 1
fi

if [[ ! -f "${GSCAM2_SRC}/package.xml" ]]; then
  echo "Expected ${GSCAM2_SRC}/package.xml; ${GSCAM2_SRC} does not look like a gscam2 ROS package." >&2
  exit 1
fi

mkdir -p "${BUILD_CONTEXT}/gscam2"
rm -rf "${BUILD_CONTEXT}/gscam2"
mkdir -p "${BUILD_CONTEXT}/gscam2"

tar -C "${GSCAM2_SRC}" \
  --exclude='.git' \
  --exclude='build' \
  --exclude='install' \
  --exclude='log' \
  -cf - . | tar -C "${BUILD_CONTEXT}/gscam2" -xf -

export DOCKER_BUILDKIT="${DOCKER_BUILDKIT:-1}"

docker build \
  --platform "${DOCKER_PLATFORM}" \
  --build-arg "BASE_IMAGE=${BASE_IMAGE}" \
  --build-arg "ROS_DISTRO=${ROS_DISTRO}" \
  -t "${IMAGE_NAME}" \
  -f "${SCRIPT_DIR}/Dockerfile" \
  "${BUILD_CONTEXT}"

cat <<EOF

Built image: ${IMAGE_NAME}

Run an interactive container:
  IMAGE_NAME=${IMAGE_NAME} ${SCRIPT_DIR}/run.sh
EOF
