# Lightweight gscam2 Jazzy Docker Method

This builds a portable ROS 2 Jazzy container for wrist camera validation on Jetson Thor / JetPack 7.x / Jetson Linux R38.4.0. It does not install ROS camera packages on the host and does not touch the local ROS host build.

The image builds `gscam2` from local source and relies on the Jetson NVIDIA container runtime to provide `nvvidconv` and matching Jetson multimedia libraries from the host. Do not add a new GStreamer ROS sink plugin for this path; `gscam2` owns the GStreamer pipeline and publishes standard ROS camera topics.

## Files

- `Dockerfile`: Ubuntu 24.04 arm64 + ROS 2 Jazzy + the minimal GStreamer build/runtime dependencies for `gscam2`, then source-builds `gscam2`.
- `build.sh`: stages `/home/unitree/dev/gscam2` into a local Docker build context and builds the image.
- `run.sh`: starts the image on Jetson with NVIDIA runtime, host network/ipc, `/dev/video2`, `/dev/video3`, and broad Jetson device access for `nvvidconv`.

## Build

Run from the repo root:

```bash
cd /home/unitree/dev/sensing_wrist_gstreamer
GSCAM2_SRC=/home/unitree/dev/gscam2 \
IMAGE_NAME=sensing-gscam2-jazzy:latest \
docker/gscam2-jazzy/build.sh
```

The script copies the local `gscam2` source into `docker/gscam2-jazzy/.build-context/gscam2` before calling `docker build`. Override `GSCAM2_SRC` if the source checkout lives elsewhere.

## Run

Start an interactive shell:

```bash
cd /home/unitree/dev/sensing_wrist_gstreamer
IMAGE_NAME=sensing-gscam2-jazzy:latest docker/gscam2-jazzy/run.sh
```

Inside the container, check that ROS, `gscam2`, and host-provided `nvvidconv` are visible:

```bash
ros2 pkg executables gscam2
gst-inspect-1.0 nvvidconv
gst-inspect-1.0 v4l2src
```

## Wrist gscam2 Command

Validated left-camera pipeline:

```bash
export WRIST_LEFT_GSCAM_CONFIG='v4l2src device=/dev/video2 ! video/x-raw,format=UYVY,width=1280,height=720,framerate=30/1 ! queue leaky=downstream max-size-buffers=1 ! nvvidconv ! video/x-raw,format=RGBA ! videoconvert ! video/x-raw,format=RGB'
```

Run the left camera as final `rgb8`:

```bash
cd /home/unitree/dev/sensing_wrist_gstreamer
CONTAINER_NAME=wrist-gscam2-left \
GSCAM_CONFIG="${WRIST_LEFT_GSCAM_CONFIG}" \
docker/gscam2-jazzy/run.sh \
ros2 run gscam2 gscam_main --ros-args \
  -r /image_raw:=/wrist/left/image_raw \
  -r /camera_info:=/wrist/left/camera_info \
  -p camera_name:=wrist_left \
  -p frame_id:=wrist_left_optical_frame \
  -p image_encoding:=rgb8 \
  -p use_gst_timestamps:=false
```

Validated right-camera pipeline:

```bash
export WRIST_RIGHT_GSCAM_CONFIG='v4l2src device=/dev/video3 ! video/x-raw,format=UYVY,width=1280,height=720,framerate=30/1 ! queue leaky=downstream max-size-buffers=1 ! nvvidconv ! video/x-raw,format=RGBA ! videoconvert ! video/x-raw,format=RGB'
```

Run the right camera in a second terminal:

```bash
cd /home/unitree/dev/sensing_wrist_gstreamer
CONTAINER_NAME=wrist-gscam2-right \
GSCAM_CONFIG="${WRIST_RIGHT_GSCAM_CONFIG}" \
docker/gscam2-jazzy/run.sh \
ros2 run gscam2 gscam_main --ros-args \
  -r /image_raw:=/wrist/right/image_raw \
  -r /camera_info:=/wrist/right/camera_info \
  -p camera_name:=wrist_right \
  -p frame_id:=wrist_right_optical_frame \
  -p image_encoding:=rgb8 \
  -p use_gst_timestamps:=false
```

When calibration YAML files exist, add `camera_info_url` to each command, for example:

```bash
-p camera_info_url:=file:///workspace/sensing_wrist_gstreamer/config/calibration/wrist_left_1280x720.yaml
```

## Validation

From another ROS 2 environment on the same host network, verify final encoding and CameraInfo:

```bash
ros2 topic echo --once /wrist/left/image_raw --field encoding
ros2 topic echo --once /wrist/left/camera_info
ros2 topic hz /wrist/left/image_raw
```

Expected image encoding is:

```text
rgb8
```

Repeat for `/wrist/right/image_raw` and `/wrist/right/camera_info`.

## Jetson Notes

The run script uses `--runtime nvidia`, `--network host`, `--ipc host`, `--privileged`, and explicit `/dev/video2` plus `/dev/video3` device mappings. `--privileged` is intentionally broad for bring-up because `nvvidconv` may need Jetson `/dev/nvhost*` and related multimedia devices. After validation, narrow this if local deployment policy requires a smaller device set.

If Docker reports that the `nvidia` runtime is unknown, install/configure NVIDIA Container Runtime for Jetson first, or run with `NVIDIA_RUNTIME=none` only for non-NVIDIA syntax checks where `nvvidconv` is not needed.
