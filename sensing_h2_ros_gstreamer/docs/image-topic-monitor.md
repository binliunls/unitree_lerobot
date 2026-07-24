# ROS 2 Image Monitor And API

This repo provides two ways to consume the ROS camera topics:

- `scripts/monitor_image_topics.py`: the original bring-up CLI. Use this when
  you want a terminal view of image metadata, FPS, and estimated drops.
- `scripts/ros_image_monitor.py`: the reusable Python API. Use this when an
  application needs to subscribe to ROS `Image` and `CameraInfo` topics, retain
  the latest samples, and inspect stream health without parsing CLI output.

Both paths avoid `cv_bridge` and OpenCV. Image payloads are read directly from
`sensor_msgs/msg/Image`, and the monitor keeps only the latest sample per topic.

## Getting Started

Start the camera publishers first:

```bash
cd /home/unitree/dev/sensing_wrist_gstreamer
./scripts/run_gscam2_argus_dual.sh
./scripts/run_gscam2_wrist_dual.sh
```

Use separate terminals for the head and wrist launchers when running all four
streams at once. In any terminal that runs `ros2`, the monitor CLI, or an
application using the Python API, source ROS and the local `gscam2` workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /home/unitree/dev/shf3l_ros_ws/install/setup.bash
```

Runtime dependencies:

- ROS 2 Jazzy Python client library: `rclpy`
- ROS 2 sensor message definitions: `sensor_msgs`
- A sourced ROS 2 environment that can see the camera topics
- Python 3 from the host OS

On this Jetson host, those dependencies are provided by:

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-sensor-msgs
```

The monitor defaults to CycloneDDS via
`config/cyclonedds/image_streams.xml` unless `RMW_IMPLEMENTATION` is already set.

## Topic Defaults

By default, both the legacy CLI and the reusable API subscribe to:

- `/head/left/image_raw`
- `/head/right/image_raw`
- `/wrist/left/image_raw`
- `/wrist/right/image_raw`
- matching `/camera_info` topics for each camera

Pass explicit topic lists when an application only needs a subset.

## CLI Usage

Use the legacy monitor for bring-up and quick health checks:

```bash
cd /home/unitree/dev/sensing_wrist_gstreamer
source /opt/ros/jazzy/setup.bash
source /home/unitree/dev/shf3l_ros_ws/install/setup.bash
./scripts/monitor_image_topics.py --expected-fps 30
```

It prints one-time metadata for the first image and CameraInfo sample on each
topic, then reports FPS and estimated drops once per second.

Monitor only one camera pair:

```bash
./scripts/monitor_image_topics.py \
  --topics /head/left/image_raw \
  --camera-info-topics /head/left/camera_info \
  --expected-fps 30
```

Avoid retaining image byte payloads in memory:

```bash
./scripts/monitor_image_topics.py --no-store-pixels
```

Tune retained payload buffers per image topic:

```bash
./scripts/monitor_image_topics.py --payload-pool-size 3
```

Increase drop tolerance for a busy host:

```bash
./scripts/monitor_image_topics.py --drop-gap-ratio 2.0
```

The newer API also has a separately named example CLI:

```bash
./scripts/run_ros_image_monitor.py --expected-fps 30
```

Keep using `scripts/monitor_image_topics.py` for existing bring-up workflows.
Use `scripts/run_ros_image_monitor.py` when you want to exercise the same API
that application code will import.

## API Integration

### Background Runner

Use `ImageMonitor` when the client application does not already manage `rclpy`.
It owns a ROS node, starts an executor thread, and exposes thread-safe accessors:

```python
from scripts.ros_image_monitor import ImageMonitor

with ImageMonitor(
    image_topics=["/head/left/image_raw"],
    camera_info_topics=["/head/left/camera_info"],
    expected_fps=30.0,
    stale_after=1.0,
) as monitor:
    image = monitor.wait_for_image("/head/left/image_raw", timeout=2.0)
    if image is not None:
        print(image.width, image.height, image.encoding, image.data_size)

    snapshot = monitor.snapshot()
    status = snapshot.statuses["/head/left/image_raw"]
    print(status.frames, status.estimated_drops, status.is_stale)
```

Useful `ImageMonitor` methods:

- `start()`, `stop()`, `close()`: control lifecycle manually.
- `wait_for_image(topic, timeout=None)`: block until a topic has an image.
- `get_latest_image(topic)`: get the latest retained `LatestImage`, or `None`.
- `get_latest_camera_info(topic)`: get the latest retained `LatestCameraInfo`,
  or `None`.
- `snapshot()`: get a consistent view of latest samples, FPS reports, and topic
  status.
- `add_image_callback(callback)`: receive `(topic, LatestImage)` on each image.
- `add_camera_info_callback(callback)`: receive `(topic, LatestCameraInfo)` on
  each CameraInfo.
- `add_drop_callback(callback)`: receive `(topic, FpsReport)` when estimated
  drops increase.

### Existing ROS Node

Use `ImageTopicSubscriber` when the client already owns an `rclpy` node and
executor:

```python
from scripts.ros_image_monitor import ImageTopicSubscriber

subscriber = ImageTopicSubscriber(
    image_topics=["/wrist/left/image_raw"],
    camera_info_topics=["/wrist/left/camera_info"],
)
subscriber.attach_to_node(existing_node)
subscriber.add_image_callback(lambda topic, image: consume(topic, image))
```

The caller remains responsible for `rclpy.init()`, adding the node to an
executor, spinning, and shutdown.

### Data Types

`LatestImage` contains:

- `received_monotonic`: local receive time from `time.monotonic()`
- `sample_time`, `stamp_sec`, `stamp_nanosec`: ROS header time
- `frame_id`
- `width`, `height`, `encoding`, `step`
- `data_size`
- `data`: retained payload as a `memoryview`, or empty bytes when
  `store_pixels=False`

`LatestCameraInfo` contains the CameraInfo header, dimensions, distortion model,
calibration arrays, and the original message object.

`FpsReport` contains receive-side FPS, publisher-header FPS, total frames,
estimated drops, and publish gap timings.

`TopicStatus` contains a compact health view: frame count, receive age, estimated
drops, and `is_stale`.

`MonitorSnapshot` groups latest images, latest camera info, stats, and statuses
so client code can make decisions from one consistent read.

### Import Path

Run application scripts from the repo root, or ensure the repo root is on
`PYTHONPATH`, so `from scripts.ros_image_monitor import ImageMonitor` resolves.
The module keeps ROS imports lazy, so pure Python tests can import the module
without sourcing ROS.

## Output

On the first image from each topic, it prints metadata similar to:

```text
[image-meta] /head/left/image_raw frame_id=head_left_optical_frame stamp=... encoding=rgb8 size=640x480 step=1920 data_bytes=921600
```

On the first CameraInfo from each topic, it prints metadata similar to:

```text
[camera-info-meta] /head/left/camera_info frame_id=head_left_optical_frame size=640x480 distortion_model=plumb_bob d_len=5 k0=1 k4=1
```

Every second, it prints FPS and drop estimates:

```text
[fps] /head/left/image_raw receive_window=29.98 receive_avg=29.99 publish_window=30.00 publish_avg=30.00 frames=300 drops~=0 publish_last_gap_ms=33.4 publish_max_gap_ms=38.2 last_receive_age_ms=4.1
```

Fields:

- `receive_window`: observed receive FPS in the most recent report window
- `receive_avg`: observed receive FPS since the first received frame
- `publish_window`: publisher header-stamp FPS in the most recent report window
- `publish_avg`: publisher header-stamp FPS since the first received frame
- `frames`: total frames received by this monitor
- `drops~`: estimated missing frames based on ROS header timestamp gaps
- `publish_last_gap_ms`: header timestamp gap between the last two frames
- `publish_max_gap_ms`: largest header timestamp gap seen by the monitor
- `last_receive_age_ms`: time since this monitor received the latest retained image sample

Drop detection is approximate. The script compares ROS header timestamp gaps
against `1 / expected_fps`. With the default `--drop-gap-ratio 1.5`, a 30 FPS
stream is considered suspicious when a header gap exceeds about `50 ms`.

## Notes

- The monitor stores only the latest image per topic, not a full history.
- By default, each image topic gets two reusable payload buffers sized to the
  largest frame seen on that topic. This reduces large allocation churn while
  keeping memory bounded.
- If client code keeps `LatestImage.data` for longer than the payload pool can
  protect, raise `payload_pool_size` or copy the bytes in the callback.
- Frame drops are estimated from ROS header timestamps. That is useful for
  bring-up, but it is not a hardware timestamp audit unless the publishers use
  hardware-synchronized stamps.
- Use the existing validation scripts when you only need a one-shot check of
  metadata fields; use the monitor API when application code needs live image
  data plus stream health.
