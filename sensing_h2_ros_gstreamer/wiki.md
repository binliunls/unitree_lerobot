# Wrist And Head ROS/GStreamer Handoff Wiki

For current user-facing bring-up instructions, use `docs/camera-bringup.md`.
This wiki is a historical handoff log and may preserve older validation notes.

## Current Goal

Publish the wrist and head cameras into ROS 2 with:

- `sensor_msgs/msg/Image` final encoding: `rgb8`
- `sensor_msgs/msg/CameraInfo`
- ROS 2 launch/orchestration as the primary runtime model
- GStreamer as the internal camera capture/conversion backend

Do not build a new GStreamer ROS sink plugin for v1. The preferred shape is `gscam2`: a ROS 2 camera node that owns a GStreamer pipeline internally and publishes standard camera topics.

## Target Host

- SSH target: `unitree@192.168.31.225`
- Hostname: `sh-01`
- Sudo password provided by user: `123`
- Platform: Jetson Thor / JetPack 7.x
- `/etc/nv_tegra_release`: `R38 (release), REVISION: 4.0`
- GStreamer: `1.24.2`

`sh-01` did not resolve by hostname from the local environment; `192.168.31.225` worked.

## Non-ROS / Non-Generic-GStreamer Dependencies

These are the host-specific dependencies beyond ROS 2 packages and generic
GStreamer packages such as `videoconvert`, `v4l2src`, and base plugin libraries.

Runtime platform dependencies:

- Jetson Thor / JetPack 7.x with Jetson Linux `R38.4.0`.
- NVIDIA Jetson camera and multimedia stack from JetPack.
- NVIDIA Argus camera service: `nvargus-daemon`.
- NVIDIA-specific GStreamer elements from JetPack:
  - `nvarguscamerasrc` for the head cameras.
  - `nvvidconv` for hardware YUV/NVMM conversion.
- Unitree camera bring-up scripts that must run after boot:
  - `~/.Unitree/set_i2c3.sh`
  - `~/.Unitree/YUSHU_4A_AGTH_G2Y_7.1/load_modules.sh`
- Argus service environment for the current head-camera setup:
  - `NVCAMERA_NITO_PATH=CONFIG`
  - `enableCamInfiniteTimeout=1`
- Camera device availability:
  - Argus `sensor-id=0` and `sensor-id=1` for `/head/left` and `/head/right`.
  - V4L2 `/dev/video2` and `/dev/video3` for `/wrist/left` and `/wrist/right`.

Useful host diagnostics and setup tools:

- `v4l-utils` for `v4l2-ctl`.
- `systemd` / `systemctl` for managing `nvargus-daemon`.
- `git-lfs` for Isaac ROS repositories.
- Debian packaging tools used during local setup or packaging:
  - `dpkg-deb`
  - `debhelper`
  - `dh-python`

Python/system helper packages installed during Isaac ROS CLI setup:

- `python3-pydantic`
- `python3-termcolor`
- `python3-venv`

For containerized runs, the Docker path also depends on NVIDIA Container
Runtime and host device access to Jetson multimedia devices such as
`/dev/video*`, `/dev/nvhost*`, and `/tmp/argus_socket`.

## Camera Nodes And Namespaces

Backend camera mapping:

- Head stereo camera: model S56, `nvarguscamerasrc sensor-id=0` and `sensor-id=1`, publish under `/head/left/*` and `/head/right/*`.
- Wrist cameras: model SHF3L, V4L2 `/dev/video2` and `/dev/video3`, publish under `/wrist/left/*` and `/wrist/right/*`.
- These identifiers are not interchangeable: do not use Argus `sensor-id=2`/`3`, and do not use `/dev/video0`/`1` for the V4L2 wrist pipeline.

`/dev/video2` and `/dev/video3` exist and advertise the same formats:

- `UYVY`
- `NV16`

Advertised resolutions at 30 fps:

- `1280x720`
- `1920x1536`
- `2880x1860`
- `3840x2160`
- `1600x1300`

Validated smoke tests:

- `/dev/video2` `UYVY 1280x720@30` through `nvvidconv` to `RGBA`: passed.
- `/dev/video3` `UYVY 1280x720@30` through `nvvidconv` to `RGBA`: passed.
- Dual concurrent `/dev/video2` and `/dev/video3` `UYVY 1280x720@30` through `nvvidconv` to `RGBA`: passed.
- `NV16 1280x720@30` through `nvvidconv` to `RGBA`: passed.

## NVIDIA GStreamer Conversion Findings

Installed:

- `nvvidconv`
- `videoconvert`

Not installed:

- `nvvideoconvert`

`nvvidconv` supports SHF3L raw `UYVY` and `NV16` input. On this target, tested output caps:

- Works: `RGBA`
- Works: `BGRx`
- Works: `UYVY`
- Works: `NV16`
- Fails: `RGB`
- Fails: `BGR`
- Fails: `RGBx`
- Fails: `xRGB`
- Fails: `xBGR`
- Fails: `BGRA`
- Fails: `ARGB`
- Fails: `ABGR`

Important design consequence:

- `nvvidconv` cannot directly produce final packed `RGB` on this target.
- Final ROS output must still be `rgb8`.
- The wrist accelerated path is not the active path right now. It is faster in
  single-camera userspace CPU measurements, but showed top-left crop /
  horizontal tearing under `gscam2`.
- The active wrist path is CPU `videoconvert` after V4L2 UYVY capture:

```bash
v4l2src device=/dev/video2 !
  'video/x-raw,format=UYVY,width=1280,height=720,framerate=30/1' !
  queue leaky=downstream max-size-buffers=1 !
  videoconvert ! 'video/x-raw,format=RGB'
```

For the Argus head path, pure CPU `videoconvert` cannot consume
`memory:NVMM` directly. The active head path captures `1920x1080`, uses
`nvvidconv` to downscale to `640x480` and convert Argus NVMM output to
CPU-readable `NV12`, then uses `videoconvert` for final packed `RGB`:

```bash
nvarguscamerasrc sensor-id=0 !
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' !
  queue leaky=downstream max-size-buffers=1 !
  nvvidconv ! 'video/x-raw,format=NV12,width=640,height=480' !
  videoconvert ! 'video/x-raw,format=RGB,width=640,height=480'
```

## Why Not Direct RGB?

This appears to be a hardware/video-surface alignment choice rather than a simple bug. NVIDIA accelerated paths favor formats that map cleanly to NVMM/CUDA/VIC memory surfaces. Packed 3-byte `RGB` has awkward alignment and stride behavior. Four-byte formats like `RGBA` and `BGRx` are more accelerator-friendly.

## ROS / Isaac ROS Environment Status

Host-level ROS was not initially present:

- No `/opt/ros/*`
- `ros2` not initially on `PATH`

Isaac docs live under:

- `~/dev/isaac/docs/isaac_ros_docs/src/index.rst`
- `~/dev/isaac/docs/isaac_ros_docs/src/getting_started/index.rst`

Important docs finding:

- Docker is the recommended Isaac ROS environment isolation mode.
- CLI flow is `sudo isaac-ros init docker` then `isaac-ros activate`.

Work performed:

- Built `isaac-ros-cli` from `~/dev/isaac/scripts/isaac-ros-cli`.
- Installed missing build/runtime dependencies:
  - `debhelper`
  - `dh-python`
  - `python3-pydantic`
  - `python3-termcolor`
  - `python3-venv`
- Installed `isaac-ros-cli_2.3.0-1_all.deb`.
- Ran `sudo isaac-ros init docker --yes`.
- Current known status after init / retry:

```json
{"activation": "inactive", "mode": "docker"}
```

Activation note:

- In the Cursor/non-interactive shell, `isaac-ros activate --build-local` failed until `ISAAC_ROS_WS=/home/unitree/dev/isaac` was provided.
- With `ISAAC_ROS_WS` set, activation reached prerequisite checks. `git-lfs` is installed now.

Host-side ROS 2 note:

- Ubuntu 24.04 / Noble matches ROS 2 Jazzy.
- The ROS 2 apt repository was added and `ros-jazzy-ros-base`, `ros-jazzy-camera-info-manager`, `ros-jazzy-image-transport`, and `ros-jazzy-gscam` are available for arm64.
- Do not install binary `ros-jazzy-gscam` or `ros-jazzy-usb-cam` on the host as-is: apt simulation showed they would install Ubuntu OpenCV dev packages, downgrade NVIDIA `libopencv-dev` from `4.8.0-3-g6ef37b4` to Ubuntu `4.6.0`, and remove `nvidia-jetpack`, `nvidia-jetpack-dev`, and `nvidia-opencv-dev`.
- A JetPack-safe local ROS install was completed with `ros-jazzy-ros-base`, `ros-jazzy-ament-cmake`, `ros-jazzy-camera-info-manager`, `ros-jazzy-camera-calibration-parsers`, `ros-jazzy-class-loader`, `ros-jazzy-rclcpp-components`, `ros-jazzy-sensor-msgs`, `ros-jazzy-image-transport`, `python3-colcon-common-extensions`, `libgstreamer1.0-dev`, and `libgstreamer-plugins-base1.0-dev`.
- `gscam2` was built locally from `/home/unitree/dev/gscam2` in `/home/unitree/dev/shf3l_ros_ws` with `colcon build --symlink-install --packages-select gscam2`.
- The built local workspace exposes `ros2 run gscam2 gscam_main` and `ros2 run gscam2 ipc_test_main`.
- `gscam2` `package.xml` lists `cv_bridge`, but the current `CMakeLists.txt` does not use it. Do not run `rosdep install` blindly for local `gscam2`, because binary `cv_bridge` can pull OpenCV packages that conflict with JetPack.
- Local `gscam2` validation with `videotestsrc` passed: test `image_raw` published `rgb8`, `1280x720`, `step=3840`.
- Local `gscam2` publishes `CameraInfo`, but width/height are `0` without a calibration file. Use calibration YAMLs under `config/calibration/` for nonzero metadata.
- Real `/dev/video2` validation through local `gscam2` passed with the V4L2 SHF3L wrist pipeline: image published `rgb8`, `1280x720`, `step=3840`, and CameraInfo published `1280x720`.
- `nvarguscamerasrc` separate-process support was added with `scripts/run_gscam2_argus_camera.sh`, `scripts/run_gscam2_argus_left.sh`, `scripts/run_gscam2_argus_right.sh`, `scripts/run_gscam2_argus_dual.sh`, and `scripts/validate_gscam2_argus_dual.sh`.
- Head Argus `gscam2` validation passed with two separate processes using supported `sensor-id=0` and `sensor-id=1`. The current launch default publishes downsampled `rgb8` `640x480`, `step=1920` images with matching `640x480` CameraInfo.
- Head Argus only supports `sensor-id=0` and `sensor-id=1` for this setup. The wrist V4L2 path only supports `/dev/video2` and `/dev/video3`.
- RViz2 is installed at `/opt/ros/jazzy/bin/rviz2`.

Next agent should run/continue:

```bash
source /opt/ros/jazzy/setup.bash
source /home/unitree/dev/shf3l_ros_ws/install/setup.bash
ros2 pkg executables gscam2
ls -l /dev/video*
v4l2-ctl --list-devices
```

When `/dev/video2` and `/dev/video3` reappear, rerun real-camera `gscam2` validation.

For the separate-process Argus path:

```bash
cd /home/unitree/dev/sensing_wrist_gstreamer
./scripts/validate_gscam2_argus_dual.sh
```

The validated and supported head Argus pair is `sensor-id=0` and `sensor-id=1`.

## ROS Bridge Recommendation

Primary direction:

- Use `clydemcqueen/gscam2` for v1.
- `gscam2` is a ROS 2 node, not a GStreamer sink plugin. It owns an internal `appsink`, publishes `image_raw` and `camera_info`, supports ROS 2 composition/intra-process examples, and exposes parameters including `gscam_config`, `camera_info_url`, `camera_name`, `frame_id`, `use_gst_timestamps`, and `image_encoding`.
- The documented executable is `ros2 run gscam2 gscam_main`. The documented topics are `image_raw` and `camera_info`.

Do not use as v1 primary path:

- Released binary `ros-jazzy-gscam` on the host if it requires downgrading NVIDIA OpenCV or removing JetPack packages.
- `gstreamer_ros_babel_fish` / `rbfimagesink`: good true GStreamer sink plugin, but the user clarified ROS should orchestrate the cameras.
- `BrettRD/ros-gst-bridge`: true sink/source plugin design, but docs say `rosimagesink` still needs CameraInfo publisher.

## Next Validation Steps

1. Use the local ROS workspace first:

```bash
source /opt/ros/jazzy/setup.bash
source /home/unitree/dev/shf3l_ros_ws/install/setup.bash
command -v ros2
ros2 pkg list | grep -E '(^gscam$|gscam2|usb_cam|camera_info_manager|image_transport)'
ros2 pkg executables gscam2 || true
```

2. Confirm `/dev/video2` and `/dev/video3` exist again.
3. Validate one real SHF3L wrist camera:

```bash
export GSCAM_CONFIG="v4l2src device=/dev/video2 ! \
  video/x-raw,format=UYVY,width=1280,height=720,framerate=30/1 ! \
  queue leaky=downstream max-size-buffers=1 ! \
  nvvidconv ! video/x-raw,format=RGBA ! \
  videoconvert ! video/x-raw,format=RGB"

ros2 run gscam2 gscam_main --ros-args \
  -r /image_raw:=/wrist/left/image_raw \
  -r /camera_info:=/wrist/left/camera_info \
  -p camera_name:=wrist_left \
  -p frame_id:=wrist_left_optical_frame \
  -p camera_info_url:=file:///home/unitree/dev/sensing_wrist_gstreamer/config/calibration/wrist_left_1280x720.yaml \
  -p image_encoding:=rgb8
```

4. Validate `image_raw` publishes as `rgb8`.
5. Validate `camera_info` timestamps, frame IDs, width, and height.
6. For live FPS/drop monitoring across all known camera topics, run:

```bash
cd /home/unitree/dev/sensing_wrist_gstreamer
source /opt/ros/jazzy/setup.bash
source /home/unitree/dev/shf3l_ros_ws/install/setup.bash
./scripts/monitor_image_topics.py --expected-fps 30
```

The monitor depends on `rclpy` and `sensor_msgs`, provided by
`ros-jazzy-ros-base` and `ros-jazzy-sensor-msgs`. It does not require
`cv_bridge` or OpenCV. See `docs/image-topic-monitor.md` for both the legacy
CLI monitor and the `scripts/ros_image_monitor.py` Python API for applications
that need to consume image and CameraInfo data directly. The API exposes a
background `ImageMonitor` wrapper for non-ROS app code and an
`ImageTopicSubscriber` component for applications that already own an `rclpy`
node and executor.

7. Run dual-camera ROS launch.
8. Measure CPU load from the final `videoconvert` tail.

## Known Annoyance

Remote sudo prints:

```text
sudo: unable to resolve host sh-01: Temporary failure in name resolution
```

This did not block apt/dpkg commands. It likely means `/etc/hosts` does not map `sh-01`; fixing it is optional and unrelated to camera validation.
