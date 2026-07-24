#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
head_output_width="${HEAD_OUTPUT_WIDTH:-640}"
head_output_height="${HEAD_OUTPUT_HEIGHT:-480}"
camera_info_yaml="${HEAD_LEFT_CAMERA_INFO_YAML:-${repo_root}/config/calibration/head_left_${head_output_width}x${head_output_height}.yaml}"

exec "${repo_root}/scripts/run_gscam2_argus_camera.sh" \
  "${HEAD_LEFT_SENSOR_ID:-0}" \
  "gscam_head_left" \
  "/head/left" \
  "head_left" \
  "head_left_optical_frame" \
  "${camera_info_yaml}" \
  "${HEAD_CAPTURE_WIDTH:-2560}" \
  "${HEAD_CAPTURE_HEIGHT:-1984}" \
  "${HEAD_FPS:-30}" \
  "${head_output_width}" \
  "${head_output_height}"
