#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

left_log="${WRIST_LEFT_LOG:-/tmp/wrist_gscam2_left.log}"
right_log="${WRIST_RIGHT_LOG:-/tmp/wrist_gscam2_right.log}"

"${repo_root}/scripts/run_gscam2_wrist_left.sh" >"${left_log}" 2>&1 &
left_pid=$!

"${repo_root}/scripts/run_gscam2_wrist_right.sh" >"${right_log}" 2>&1 &
right_pid=$!

cleanup() {
  kill "${left_pid}" "${right_pid}" 2>/dev/null || true
  wait "${left_pid}" "${right_pid}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "Started left gscam2 wrist process pid=${left_pid}, log=${left_log}"
echo "Started right gscam2 wrist process pid=${right_pid}, log=${right_log}"
wait "${left_pid}" "${right_pid}"
