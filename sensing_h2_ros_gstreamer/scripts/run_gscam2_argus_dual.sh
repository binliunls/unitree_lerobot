#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

left_log="${HEAD_LEFT_LOG:-/tmp/head_gscam2_left.log}"
right_log="${HEAD_RIGHT_LOG:-/tmp/head_gscam2_right.log}"

setsid "${repo_root}/scripts/run_gscam2_argus_left.sh" >"${left_log}" 2>&1 &
left_pid=$!

sleep "${HEAD_RIGHT_START_DELAY_SECONDS:-3}"

setsid "${repo_root}/scripts/run_gscam2_argus_right.sh" >"${right_log}" 2>&1 &
right_pid=$!

cleanup() {
  kill -- "-${left_pid}" "-${right_pid}" 2>/dev/null || true
  pkill -TERM -f "gscam_head_left" 2>/dev/null || true
  pkill -TERM -f "gscam_head_right" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "gscam_head_left" 2>/dev/null || true
  pkill -KILL -f "gscam_head_right" 2>/dev/null || true
  wait "${left_pid}" "${right_pid}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "Started left gscam2 Argus process pid=${left_pid}, log=${left_log}"
echo "Started right gscam2 Argus process pid=${right_pid}, log=${right_log}"
wait "${left_pid}" "${right_pid}"
