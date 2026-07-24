# Agent Handoff: SHF3L ROS/GStreamer Bridge

## Mission

Continue validating and implementing a ROS 2-orchestrated dual-camera bridge for SHF3L cameras on Jetson Thor.

Final user requirement:

- Publish final ROS images as `sensor_msgs/msg/Image` with `rgb8` encoding.
- Publish matching `sensor_msgs/msg/CameraInfo`.
- Run/orchestrate from ROS 2, not from a GStreamer-first product.

## Start Here

Read these files first:

- `wiki.md`
- `docs/plans/shf3l_ros_bridge_298c981d.plan.md`

Target host:

```bash
ssh unitree@192.168.31.225
```

Sudo password on target is known to the user and was provided in the previous session.

## Current State

Validated:

- `/dev/video2` and `/dev/video3` both stream SHF3L `UYVY`/`NV16`.
- `nvvidconv` can convert `UYVY`/`NV16` to `RGBA` and `BGRx`.
- `nvvidconv` cannot output packed `RGB` on this target.
- `UYVY -> nvvidconv -> RGBA -> videoconvert -> RGB` works.
- Dual concurrent camera GStreamer smoke test works.
- `isaac-ros-cli` was built and installed.
- `isaac-ros init docker --yes` was run successfully.
- Isaac ROS status was `mode: docker`, `activation: inactive`.
- `isaac-ros activate` must be run with `ISAAC_ROS_WS=/home/unitree/dev/isaac` in this non-interactive shell.
- Host-side ROS 2 Jazzy base and `gscam2` dependencies were installed without downgrading NVIDIA OpenCV or removing JetPack metapackages.
- `gscam2` was built locally from `/home/unitree/dev/gscam2` in `/home/unitree/dev/shf3l_ros_ws`.
- Local `gscam2` publishes `rgb8` Image and CameraInfo with a calibration file using `videotestsrc`.
- Real `/dev/video2` through local `gscam2` publishes `rgb8` Image and nonzero CameraInfo using the validated V4L2 wrist pipeline.
- `nvarguscamerasrc` dual-process `gscam2` support was added under `scripts/run_gscam2_argus_*.sh` and validated with Argus `sensor-id=0` plus `sensor-id=1`.
- Camera mapping is fixed by backend and namespace: the S56 head stereo path uses Argus `sensor-id=0/1` and publishes under `/head/left/*` and `/head/right/*`; the SHF3L wrist path uses `/dev/video2` and `/dev/video3` and publishes under `/wrist/left/*` and `/wrist/right/*`.
- Do not try to use Argus `sensor-id=2`/`3` for these cameras, and do not try to use `/dev/video0`/`1` for the V4L2 wrist pipeline.
- RViz2 is installed at `/opt/ros/jazzy/bin/rviz2`.

Do not repeat already completed hardware smoke tests unless you need fresh evidence after changing something.

## Next Best Step

Use the local ROS 2 workspace first:

```bash
source /opt/ros/jazzy/setup.bash
source /home/unitree/dev/shf3l_ros_ws/install/setup.bash
ros2 pkg executables gscam2
```

Wait for the SHF3L V4L2 nodes to reappear, then rerun:

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

Then validate real `/dev/video2` through local `gscam2` with the candidate wrist pipeline below and a camera calibration URL. Without a calibration file, `gscam2` still publishes `CameraInfo`, but width/height remain `0`.

```bash
ros2 run gscam2 gscam_main --ros-args \
  -r /image_raw:=/wrist/left/image_raw \
  -r /camera_info:=/wrist/left/camera_info \
  -p camera_name:=wrist_left \
  -p frame_id:=wrist_left_optical_frame \
  -p camera_info_url:=file:///home/unitree/dev/sensing_wrist_gstreamer/config/calibration/wrist_left_1280x720.yaml \
  -p image_encoding:=rgb8 \
  -p use_gst_timestamps:=false
```

For the separate-process Argus path, run:

```bash
cd /home/unitree/dev/sensing_wrist_gstreamer
./scripts/validate_gscam2_argus_dual.sh
```

This publishes the S56 head stereo topics under `/head/left/*` and `/head/right/*`, using the supported Argus IDs `HEAD_LEFT_SENSOR_ID=0` and `HEAD_RIGHT_SENSOR_ID=1`.

## Design Constraints

- Use `gscam2` as the v1 ROS camera node target.
- Do not install binary `ros-jazzy-gscam` on the host if apt wants to downgrade NVIDIA OpenCV or remove JetPack packages.
- Do not run `rosdep install` blindly for local `gscam2`; its manifest lists `cv_bridge`, but the current `CMakeLists.txt` does not use it, and avoiding binary `cv_bridge` avoids OpenCV/JetPack conflicts.
- Do not make `rbfimagesink` / GStreamer sink plugin the primary v1 route.
- Do not publish final `rgba8`; final output must be `rgb8`.
- `RGBA` and `BGRx` are acceptable intermediate formats only.
- Keep the final `videoconvert ! video/x-raw,format=RGB` tail until performance measurements prove it is unacceptable.

## Candidate Pipeline

Use this as the first real-camera GStreamer config for `gscam2`:

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

Mirror it for `/dev/video3` as `/wrist/right/*`.

## Avoid These Pitfalls

- Do not assume `nvvidconv` supports `RGB` output; it was tested and fails caps negotiation.
- Do not assume `/dev/video2` and `/dev/video3` currently exist; they were validated earlier but were absent during the latest ROS validation attempt.
- Do not mix backend camera identifiers or namespaces: S56 head is `nvarguscamerasrc sensor-id=0/1` under `/head`; SHF3L wrist is V4L2 `/dev/video2` and `/dev/video3` under `/wrist`.
- Do not use `sh-01` hostname unless DNS has been fixed; use `192.168.31.225`.
- Do not spend time creating a new GStreamer sink plugin for v1.
- Do not treat `rgba8` as acceptable final output; the user explicitly clarified final output must be `rgb8`.

## Plan Maintenance

When new validation is completed, update:

- `wiki.md`
- `docs/plans/shf3l_ros_bridge_298c981d.plan.md`
- this `AGENT.md` if the next-agent instructions change
