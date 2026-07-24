# Camera Bring-Up Quick Start

This is the main user-facing reference for publishing the head and wrist cameras
through `gscam2`.

Use `docs/camera-pipeline-notes.md` for implementation details and
`docs/image-topic-monitor.md` for monitor options and the Python embedding API.

## TL;DR

- From the repo root, run the boot setup once after each boot:
  `./scripts/setup_sensing_cameras_boot.sh`.
- Start the head cameras with `./scripts/run_gscam2_argus_dual.sh`.
- Start the wrist cameras with `./scripts/run_gscam2_wrist_dual.sh`.
- Check metadata with `./scripts/validate_gscam2_argus_dual.sh` and
  `./scripts/validate_gscam2_wrist_dual.sh`.
- Monitor stream health with `./scripts/monitor_image_topics.py --expected-fps 30`.
- Consume the same ROS image and CameraInfo topics from Python applications with
  `scripts/ros_image_monitor.py`.
- For direct `ros2`, RViz, or monitor commands, source `/opt/ros/jazzy/setup.bash`
  and `/home/unitree/dev/shf3l_ros_ws/install/setup.bash` first.

The dual-camera scripts stay in the foreground. Run head and wrist in separate
terminals when you want all four streams at once.

## Output Contract

Every camera publishes:

- `sensor_msgs/msg/Image` on `image_raw`
- `sensor_msgs/msg/CameraInfo` on `camera_info`
- final image encoding `rgb8`
- published image size `640x480`
- image step `1920` bytes
- expected rate about `30 Hz`
- ROS publish-time headers because `use_gst_timestamps:=false`

`RGBA` may appear inside GStreamer conversion experiments, but it is not the ROS
output format. The ROS image output is `rgb8`.

## Camera Mapping

Head cameras use the Jetson Argus backend:

- `/head/left`: `nvarguscamerasrc sensor-id=0`
- `/head/right`: `nvarguscamerasrc sensor-id=1`
- capture size: `2560x1984`
- published size: `640x480`

Wrist cameras use the V4L2 backend:

- `/wrist/left`: `/dev/video2`
- `/wrist/right`: `/dev/video3`
- capture size: `1920x1536`
- published size: `640x480`

Do not mix these identifiers. Argus IDs `0` and `1` are for the head path.
V4L2 devices `/dev/video2` and `/dev/video3` are for the wrist path.

## Setup Each Boot

Run `./scripts/setup_sensing_cameras_boot.sh` after each boot.

The setup script keeps the boot-sensitive pieces in one place:

- Ensures the CycloneDDS ROS 2 RMW package is installed.
- Applies the receive-buffer sysctl values used for high-rate image streams.
- Verifies that Unitree's module loader enables trigger mode for the head
  cameras.
- Runs the Unitree I2C and camera module setup scripts.
- Offers to apply the persistent `nvargus-daemon` environment override used by
  the Argus head cameras.

If the head trigger devices ever move away from `/dev/video0` and `/dev/video1`,
set `HEAD_TRIGGER_CAMERAS` before running the setup script.

The launch and validation scripts default to CycloneDDS and the repo's
`config/cyclonedds/image_streams.xml` config. Override `RMW_IMPLEMENTATION` only
when comparing middleware behavior.

## Set Up Another Jetson

For a matching Ubuntu 24.04 arm64 / ROS 2 Jazzy Jetson, copy this repository and
the locally built `gscam2` Debian package:

```bash
rsync -a /home/unitree/dev/sensing_wrist_gstreamer/ unitree@TARGET:/home/unitree/dev/sensing_wrist_gstreamer/
scp /home/unitree/dev/sensing_wrist_gstreamer/dist/ros-jazzy-gscam2_0.0.2-1_arm64.deb unitree@TARGET:/home/unitree/dev/sensing_wrist_gstreamer/
```

On the target Jetson:

```bash
cd /home/unitree/dev/sensing_wrist_gstreamer
sudo apt install ./ros-jazzy-gscam2_0.0.2-1_arm64.deb
./scripts/setup_sensing_cameras_boot.sh
source /opt/ros/jazzy/setup.bash
ros2 pkg executables gscam2
```

Expected `gscam2` output includes `gscam2 gscam_main`. The package installs the
camera node into `/opt/ros/jazzy`; the copied repository provides the camera
launch scripts, calibration files, CycloneDDS config, and validation tools. See
`docs/gscam2-packaging.md` for rebuild and rosdep details.

## Run

Start both head cameras with `./scripts/run_gscam2_argus_dual.sh`.

Start both wrist cameras with `./scripts/run_gscam2_wrist_dual.sh`.

Single-camera helpers are also available:

- `./scripts/run_gscam2_argus_left.sh`
- `./scripts/run_gscam2_argus_right.sh`
- `./scripts/run_gscam2_wrist_left.sh`
- `./scripts/run_gscam2_wrist_right.sh`

The default output size for both camera groups is `640x480`. Only override output
size when you also provide matching calibration YAMLs.

## Topics

Head topics:

- `/head/left/image_raw`
- `/head/left/camera_info`
- `/head/right/image_raw`
- `/head/right/camera_info`

Wrist topics:

- `/wrist/left/image_raw`
- `/wrist/left/camera_info`
- `/wrist/right/image_raw`
- `/wrist/right/camera_info`

Frame IDs:

- `head_left_optical_frame`
- `head_right_optical_frame`
- `wrist_left_optical_frame`
- `wrist_right_optical_frame`

## Calibration Status

The launch helpers default to the scaled `640x480` calibration files under
`config/calibration/`. These files use the real native calibration data copied
from the camera calibration YAMLs, with focal lengths and principal points
scaled to the published image size. Distortion coefficients are unchanged.

The wrappers capture at the calibrated native sizes (`2560x1984` for head and
`1920x1536` for wrist) before resizing to the smaller published image. If you
override output size, provide a matching `*_CAMERA_INFO_YAML`; the launcher
fails when the YAML dimensions do not match the output dimensions.

The frame IDs are stable ROS message labels, not a complete TF tree. Publish
calibrated extrinsics separately if consumers need transforms between the robot,
head, wrist, and optical frames.

## Validate

Use `./scripts/validate_gscam2_argus_dual.sh` for the head cameras and
`./scripts/validate_gscam2_wrist_dual.sh` for the wrist cameras.

Expected validation fields for all four streams:

- image `encoding: rgb8`
- image `width: 640`, `height: 480`, `step: 1920`
- matching CameraInfo `width: 640`, `height: 480`
- CameraInfo frame ID matches the camera's optical frame

For live FPS and drop visibility, run
`./scripts/monitor_image_topics.py --expected-fps 30`. Applications that need
to consume images directly can use `scripts/ros_image_monitor.py`; see
`docs/image-topic-monitor.md` for dependencies, options, and API examples.

`ros2 topic hz` can under-report large image topics on a busy host. The matching
`camera_info` topic is often a better quick check for publisher rate.

## Consume In Applications

Use `scripts/ros_image_monitor.py` when application code needs the latest image,
matching CameraInfo, and stream health in-process. The simplest integration lets
the helper own a background ROS executor:

```python
from scripts.ros_image_monitor import ImageMonitor

with ImageMonitor(
    image_topics=["/wrist/left/image_raw"],
    camera_info_topics=["/wrist/left/camera_info"],
    expected_fps=30.0,
) as monitor:
    image = monitor.wait_for_image("/wrist/left/image_raw", timeout=2.0)
    if image is not None:
        handle_rgb8(image.data, image.width, image.height, image.step)
```

If the application already owns an `rclpy` node and executor, attach
`ImageTopicSubscriber` to that node instead. See `docs/image-topic-monitor.md`
for callback, snapshot, FPS, drop, and staleness APIs.

## View In RViz

Source ROS, run `rviz2`, then add Image displays for the `image_raw` topics.

## Receive On An x86 Host

The publisher runs on the Jetson. Any ROS 2 Jazzy host on the same network can
subscribe — including an x86 workstation. The repo already ships an x86-friendly
receiver at `scripts/run_ros_image_monitor.py` (FPS / drop monitor) and
`scripts/visualize_camera_streams.py` (live image display).

### One-time x86 setup

1. Install ROS 2 Jazzy and the matching CycloneDDS RMW:

   ```bash
   sudo apt install ros-jazzy-ros-base ros-jazzy-rmw-cyclonedds-cpp
   ```

2. Raise the UDP receive buffer to match the Jetson side
   (`config/cyclonedds/image_streams.xml` requests 16 MB):

   ```bash
   sudo sysctl -w net.core.rmem_max=16777216
   sudo sysctl -w net.core.rmem_default=16777216
   ```

   Persist by adding the same lines to `/etc/sysctl.d/60-ros-image-streams.conf`.

3. For the visualizer only, install OpenCV and NumPy:

   ```bash
   sudo apt install python3-opencv python3-numpy
   ```

### Each shell

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0   # match the Jetson; default is 0 if neither sets it
```

Both `run_ros_image_monitor.py` and `visualize_camera_streams.py` auto-set
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and point `CYCLONEDDS_URI` at this
repo's `config/cyclonedds/image_streams.xml`, so no extra env is needed as long
as the Jetson and x86 share an L2/multicast-capable network.

### Sanity checks

```bash
ros2 topic list | grep -E '/(head|wrist)/(left|right)/(image_raw|camera_info)'
ros2 topic hz /head/left/camera_info       # cheaper than topic hz on image_raw
./scripts/run_ros_image_monitor.py --expected-fps 30
```

### Visualize the streams

```bash
./scripts/visualize_camera_streams.py                           # all four cameras tiled
./scripts/visualize_camera_streams.py --topics /wrist/left/image_raw /wrist/right/image_raw
./scripts/visualize_camera_streams.py --separate                # one window per camera
```

Press `q` or `Esc` in any window to quit.

## More Details

- `docs/camera-pipeline-notes.md`: GStreamer pipeline shape, timestamp choices,
  tracing, and accelerated wrist status.
- `docs/image-topic-monitor.md`: live monitor behavior, options, and the Python
  embedding API for consuming image and CameraInfo data.
- `docs/gscam2-packaging.md`: packaging `gscam2` for another matching host.
