#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cyclonedds_config="${repo_root}/config/cyclonedds/image_streams.xml"

head_log="${HEAD_DUAL_LOG:-/tmp/head_gscam2_dual.log}"
wrist_log="${WRIST_DUAL_LOG:-/tmp/wrist_gscam2_dual.log}"

# Use the same default RMW settings as the individual camera launch scripts.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ "${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp" ]]; then
  export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${cyclonedds_config}}"
fi

setsid "${repo_root}/scripts/run_gscam2_argus_dual.sh" >"${head_log}" 2>&1 &
head_pid=$!
sleep 2
setsid "${repo_root}/scripts/run_gscam2_wrist_dual.sh" >"${wrist_log}" 2>&1 &
wrist_pid=$!

cleanup() {
  kill -- "-${head_pid}" "-${wrist_pid}" 2>/dev/null || true
  pkill -TERM -f "gscam_head_left|gscam_head_right|gscam_wrist_left|gscam_wrist_right" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "gscam_head_left|gscam_head_right|gscam_wrist_left|gscam_wrist_right" 2>/dev/null || true
  wait "${head_pid}" "${wrist_pid}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "Started head camera launcher pid=${head_pid}, log=${head_log}"
echo "Started wrist camera launcher pid=${wrist_pid}, log=${wrist_log}"
echo "Per-camera logs default to /tmp/head_gscam2_{left,right}.log and /tmp/wrist_gscam2_{left,right}.log"

wait "${head_pid}" "${wrist_pid}"
