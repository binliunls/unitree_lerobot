#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
wrist_output_width="${WRIST_OUTPUT_WIDTH:-640}"
wrist_output_height="${WRIST_OUTPUT_HEIGHT:-480}"
camera_info_yaml="${WRIST_LEFT_CAMERA_INFO_YAML:-${repo_root}/config/calibration/wrist_left_${wrist_output_width}x${wrist_output_height}.yaml}"

exec "${repo_root}/scripts/run_gscam2_wrist_camera.sh" \
  "${WRIST_LEFT_DEVICE:-/dev/video2}" \
  "gscam_wrist_left" \
  "/wrist/left" \
  "wrist_left" \
  "wrist_left_optical_frame" \
  "${camera_info_yaml}" \
  "${WRIST_CAPTURE_WIDTH:-1920}" \
  "${WRIST_CAPTURE_HEIGHT:-1536}" \
  "${WRIST_FPS:-30}" \
  "${wrist_output_width}" \
  "${wrist_output_height}"
