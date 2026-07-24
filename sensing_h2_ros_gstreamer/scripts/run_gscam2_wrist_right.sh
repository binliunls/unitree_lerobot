#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
wrist_output_width="${WRIST_OUTPUT_WIDTH:-640}"
wrist_output_height="${WRIST_OUTPUT_HEIGHT:-480}"
camera_info_yaml="${WRIST_RIGHT_CAMERA_INFO_YAML:-${repo_root}/config/calibration/wrist_right_${wrist_output_width}x${wrist_output_height}.yaml}"

exec "${repo_root}/scripts/run_gscam2_wrist_camera.sh" \
  "${WRIST_RIGHT_DEVICE:-/dev/video3}" \
  "gscam_wrist_right" \
  "/wrist/right" \
  "wrist_right" \
  "wrist_right_optical_frame" \
  "${camera_info_yaml}" \
  "${WRIST_CAPTURE_WIDTH:-1920}" \
  "${WRIST_CAPTURE_HEIGHT:-1536}" \
  "${WRIST_FPS:-30}" \
  "${wrist_output_width}" \
  "${wrist_output_height}"
