#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cyclonedds_config="${repo_root}/config/cyclonedds/image_streams.xml"

if [[ $# -lt 6 ]]; then
  echo "Usage: $0 <device> <node-name> <topic-prefix> <camera-name> <frame-id> <camera-info-yaml> [capture-width] [capture-height] [fps] [output-width] [output-height]" >&2
  exit 2
fi

device="$1"
node_name="$2"
topic_prefix="$3"
camera_name="$4"
frame_id="$5"
camera_info_yaml="$6"
capture_width="${7:-1920}"
capture_height="${8:-1536}"
fps="${9:-30}"
output_width="${10:-${capture_width}}"
output_height="${11:-${capture_height}}"

if [[ ! -e "${device}" ]]; then
  echo "Video device not found: ${device}" >&2
  exit 2
fi

if [[ ! -f "${camera_info_yaml}" ]]; then
  echo "Camera info YAML not found: ${camera_info_yaml}" >&2
  exit 2
fi

camera_info_width="$(awk -F': *' '$1 == "image_width" {print $2; exit}' "${camera_info_yaml}")"
camera_info_height="$(awk -F': *' '$1 == "image_height" {print $2; exit}' "${camera_info_yaml}")"
if [[ -z "${camera_info_width}" || -z "${camera_info_height}" ]]; then
  echo "Camera info YAML missing image_width/image_height: ${camera_info_yaml}" >&2
  exit 2
fi
if [[ "${camera_info_width}" != "${output_width}" || "${camera_info_height}" != "${output_height}" ]]; then
  echo "Camera info YAML size ${camera_info_width}x${camera_info_height} does not match output ${output_width}x${output_height}: ${camera_info_yaml}" >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source /home/unitree/dev/shf3l_ros_ws/install/setup.bash
set -u

# Prefer CycloneDDS for high-rate image topics; callers can override this env.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ "${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp" ]]; then
  export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${cyclonedds_config}}"
fi

existing_pids="$(pgrep -f "gscam_main.*__node:=${node_name}" || true)"
if [[ -n "${existing_pids}" ]]; then
  echo "Stopping existing gscam2 process named ${node_name}: ${existing_pids}" >&2
  kill ${existing_pids}
  for _ in {1..50}; do
    if ! pgrep -f "gscam_main.*__node:=${node_name}" >/dev/null; then
      break
    fi
    sleep 0.1
  done
fi

if [[ "${DEBUG_LOG:-0}" != "0" ]]; then
  trace_dir="${GSCAM2_TRACE_DIR:-/tmp/gscam2-traces}"
  mkdir -p "${trace_dir}"

  export GST_DEBUG_NO_COLOR="${GST_DEBUG_NO_COLOR:-1}"
  export GST_TRACERS="${GST_TRACERS:-latency}"
  export GST_DEBUG="${GST_DEBUG:-2,GST_TRACER:7}"
  export GST_DEBUG_FILE="${GST_DEBUG_FILE:-${trace_dir}/${node_name}.gst.log}"
fi

# Timestamp buffers at capture time so GStreamer tracing/latency data is meaningful.
export GSCAM_CONFIG="v4l2src device=${device} do-timestamp=true ! \
video/x-raw,format=UYVY,width=${capture_width},height=${capture_height},framerate=${fps}/1 ! \
queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! \
videoscale ! video/x-raw,format=UYVY,width=${output_width},height=${output_height},framerate=${fps}/1 ! \
videoconvert n-threads=2 ! video/x-raw,format=RGB,width=${output_width},height=${output_height},framerate=${fps}/1"

# sync_sink=true is for timestamp-faithful recording/playback; false is for
# realtime robotics streams where appsink backpressure can cause frame drops.
exec ros2 run gscam2 gscam_main --ros-args \
  -r "__node:=${node_name}" \
  -r "/image_raw:=${topic_prefix}/image_raw" \
  -r "/camera_info:=${topic_prefix}/camera_info" \
  -p "camera_name:=${camera_name}" \
  -p "frame_id:=${frame_id}" \
  -p "camera_info_url:=file://${camera_info_yaml}" \
  -p "image_encoding:=rgb8" \
  -p "sync_sink:=false" \
  -p "use_gst_timestamps:=false"
