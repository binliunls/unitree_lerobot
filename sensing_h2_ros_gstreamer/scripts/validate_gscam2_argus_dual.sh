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

setsid "${repo_root}/scripts/run_gscam2_argus_dual.sh" &
dual_pid=$!

cleanup() {
  kill -- "-${dual_pid}" 2>/dev/null || true
  pkill -TERM -f "gscam_head_left" 2>/dev/null || true
  pkill -TERM -f "gscam_head_right" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "gscam_head_left" 2>/dev/null || true
  pkill -KILL -f "gscam_head_right" 2>/dev/null || true
  wait "${dual_pid}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

sleep "${HEAD_STARTUP_SECONDS:-6}"

echo "--- topics ---"
ros2 topic list | sort | grep -E '/head/(left|right)/(image_raw|camera_info)' || true

for side in left right; do
  topic_prefix="/head/${side}"
  echo "--- ${side} image ---"
  timeout 10 ros2 topic echo "${topic_prefix}/image_raw" --once --field encoding
  timeout 10 ros2 topic echo "${topic_prefix}/image_raw" --once --field width
  timeout 10 ros2 topic echo "${topic_prefix}/image_raw" --once --field height
  timeout 10 ros2 topic echo "${topic_prefix}/image_raw" --once --field step

  echo "--- ${side} camera_info ---"
  timeout 10 ros2 topic echo "${topic_prefix}/camera_info" --once --field width
  timeout 10 ros2 topic echo "${topic_prefix}/camera_info" --once --field height
  timeout 10 ros2 topic echo "${topic_prefix}/camera_info" --once --field header.frame_id
done
