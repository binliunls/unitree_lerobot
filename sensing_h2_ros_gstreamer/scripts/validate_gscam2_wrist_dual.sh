#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cyclonedds_config="${repo_root}/config/cyclonedds/image_streams.xml"

set +u
source /opt/ros/jazzy/setup.bash
source /home/unitree/dev/shf3l_ros_ws/install/setup.bash
set -u

# Use the same default RMW as the camera launch scripts so ros2 CLI tools can discover them.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ "${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp" ]]; then
  export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${cyclonedds_config}}"
fi

setsid "${repo_root}/scripts/run_gscam2_wrist_dual.sh" &
dual_pid=$!

cleanup() {
  kill -- "-${dual_pid}" 2>/dev/null || true
  wait "${dual_pid}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

sleep "${WRIST_STARTUP_SECONDS:-6}"

echo_field() {
  local topic="$1"
  local field="$2"
  local attempts="${WRIST_ECHO_ATTEMPTS:-3}"

  for ((attempt = 1; attempt <= attempts; attempt++)); do
    if timeout 10 ros2 topic echo "${topic}" --once --field "${field}"; then
      return 0
    fi
    echo "Retrying ${topic} field ${field} (${attempt}/${attempts})" >&2
  done

  return 1
}

echo "--- topics ---"
ros2 topic list | sort | grep -E '/wrist/(left|right)/(image_raw|camera_info)' || true

for side in left right; do
  topic_prefix="/wrist/${side}"
  echo "--- ${side} image ---"
  echo_field "${topic_prefix}/image_raw" encoding
  echo_field "${topic_prefix}/image_raw" width
  echo_field "${topic_prefix}/image_raw" height
  echo_field "${topic_prefix}/image_raw" step

  echo "--- ${side} camera_info ---"
  echo_field "${topic_prefix}/camera_info" width
  echo_field "${topic_prefix}/camera_info" height
  echo_field "${topic_prefix}/camera_info" header.frame_id
done
