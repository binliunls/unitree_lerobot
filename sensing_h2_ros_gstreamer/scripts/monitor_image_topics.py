#!/usr/bin/env python3
"""Monitor ROS 2 image topics helper

It keeps the latest image payload for each topic in memory, prints one-time
metadata, estimates frame drops from message header timestamps, and reports
observed FPS once per second by default.
"""

import argparse
import os
import signal
import threading
import time
from dataclasses import dataclass
from pathlib import Path

DEFAULT_RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
DEFAULT_CYCLONEDDS_CONFIG = (
    Path(__file__).resolve().parents[1] / "config" / "cyclonedds" / "image_streams.xml"
)


DEFAULT_IMAGE_TOPICS = (
    "/head/left/image_raw",
    "/head/right/image_raw",
    "/wrist/left/image_raw",
    "/wrist/right/image_raw",
)

DEFAULT_CAMERA_INFO_TOPICS = (
    "/head/left/camera_info",
    "/head/right/camera_info",
    "/wrist/left/camera_info",
    "/wrist/right/camera_info",
)


def configure_default_rmw():
    os.environ.setdefault("RMW_IMPLEMENTATION", DEFAULT_RMW_IMPLEMENTATION)
    if os.environ["RMW_IMPLEMENTATION"] == DEFAULT_RMW_IMPLEMENTATION:
        os.environ.setdefault("CYCLONEDDS_URI", f"file://{DEFAULT_CYCLONEDDS_CONFIG}")


@dataclass
class FpsReport:
    frames: int
    receive_window_fps: float
    receive_average_fps: float
    publish_window_fps: float
    publish_average_fps: float
    estimated_drops: int
    last_publish_gap_seconds: float
    max_publish_gap_seconds: float


@dataclass
class LatestImage:
    """The latest image sample retained in memory for a topic."""

    received_monotonic: float
    sample_time: float
    frame_id: str
    stamp_sec: int
    stamp_nanosec: int
    width: int
    height: int
    encoding: str
    step: int
    data_size: int
    data: object


@dataclass
class ParsedImage:
    stamp_sec: int
    stamp_nanosec: int
    frame_id: str
    width: int
    height: int
    encoding: str
    step: int
    data_size: int
    data: memoryview


def stamp_to_seconds(sec, nanosec):
    return float(sec) + float(nanosec) / 1_000_000_000.0


def align4(offset):
    return (offset + 3) & ~3


def _read_u32(raw, offset):
    return int.from_bytes(raw[offset : offset + 4], "little"), offset + 4


def _read_i32(raw, offset):
    return int.from_bytes(raw[offset : offset + 4], "little", signed=True), offset + 4


def _read_string(raw, offset):
    length, offset = _read_u32(raw, offset)
    value_bytes = bytes(raw[offset : offset + length])
    offset = offset + length
    if value_bytes.endswith(b"\x00"):
        value_bytes = value_bytes[:-1]
    return value_bytes.decode("utf-8"), offset


def parse_serialized_image(raw_data):
    # ROS 2 delivers raw=True subscriptions as CDR bytes. CDR is the DDS wire
    # serialization format for messages; parsing it here avoids constructing a
    # full Python sensor_msgs/Image object and copying the large pixel array.
    raw = memoryview(raw_data)
    offset = 4  # CDR encapsulation header.
    stamp_sec, offset = _read_i32(raw, offset)
    stamp_nanosec, offset = _read_u32(raw, offset)
    frame_id, offset = _read_string(raw, offset)
    offset = align4(offset)
    height, offset = _read_u32(raw, offset)
    width, offset = _read_u32(raw, offset)
    encoding, offset = _read_string(raw, offset)
    offset += 1  # is_bigendian
    offset = align4(offset)
    step, offset = _read_u32(raw, offset)
    data_size, offset = _read_u32(raw, offset)
    data = raw[offset : offset + data_size]
    return ParsedImage(
        stamp_sec=stamp_sec,
        stamp_nanosec=stamp_nanosec,
        frame_id=frame_id,
        width=width,
        height=height,
        encoding=encoding,
        step=step,
        data_size=data_size,
        data=data,
    )


class ImagePayloadPool:
    """Per-topic reusable payload buffers for latest-image retention."""

    def __init__(self, slots_per_topic):
        self.slots_per_topic = int(slots_per_topic)
        if self.slots_per_topic < 1:
            raise ValueError("slots_per_topic must be >= 1")
        self.buffers_by_topic = {}
        self.next_index_by_topic = {}

    def store(self, topic, data):
        data_size = len(data)
        buffers = self.buffers_by_topic.get(topic)
        if buffers is None or len(buffers[0]) < data_size:
            buffers = [bytearray(data_size) for _ in range(self.slots_per_topic)]
            self.buffers_by_topic[topic] = buffers
            self.next_index_by_topic[topic] = 0

        index = self.next_index_by_topic[topic]
        buffer = buffers[index]
        buffer[:data_size] = data
        self.next_index_by_topic[topic] = (index + 1) % self.slots_per_topic
        return memoryview(buffer)[:data_size]


def payload_for_storage(data, store_pixels, pool=None, topic=None):
    if not store_pixels:
        return b""
    if pool is None:
        return memoryview(data)
    return pool.store(topic, data)


class TopicFpsStats:
    """Track observed receive FPS separately from publisher header timing."""

    def __init__(self, expected_fps, drop_gap_ratio):
        self.expected_fps = float(expected_fps)
        self.drop_gap_ratio = float(drop_gap_ratio)
        self.expected_period = 1.0 / self.expected_fps if self.expected_fps > 0 else None

        self.frames = 0
        self.estimated_drops = 0
        self.receive_start_time = None
        self.last_receive_time = None
        self.receive_window_start_time = None
        self.receive_window_frames = 0
        self.publish_start_time = None
        self.last_publish_time = None
        self.publish_window_first_time = None
        self.publish_window_frames = 0
        self.last_publish_gap_seconds = 0.0
        self.max_publish_gap_seconds = 0.0

    def record(self, publish_time, receive_time):
        if self.frames == 0:
            self.receive_start_time = receive_time
            self.receive_window_start_time = receive_time
            self.publish_start_time = publish_time
        else:
            gap = publish_time - self.last_publish_time
            self.last_publish_gap_seconds = gap
            self.max_publish_gap_seconds = max(self.max_publish_gap_seconds, gap)

            if self.expected_period and gap > self.expected_period * self.drop_gap_ratio:
                missed = max(1, round(gap / self.expected_period) - 1)
                self.estimated_drops += missed

        if self.publish_window_first_time is None:
            self.publish_window_first_time = publish_time

        self.frames += 1
        self.receive_window_frames += 1
        self.publish_window_frames += 1
        self.last_receive_time = receive_time
        self.last_publish_time = publish_time

    @staticmethod
    def _elapsed_rate(count, elapsed):
        if count <= 0 or elapsed <= 0:
            return 0.0
        return count / elapsed

    @staticmethod
    def _interval_rate(count, elapsed):
        if count <= 1 or elapsed <= 0:
            return 0.0
        return (count - 1) / elapsed

    def report(self, receive_now):
        receive_total_elapsed = (
            receive_now - self.receive_start_time if self.receive_start_time is not None else 0.0
        )
        receive_window_elapsed = (
            receive_now - self.receive_window_start_time
            if self.receive_window_start_time is not None
            else 0.0
        )
        publish_total_elapsed = (
            self.last_publish_time - self.publish_start_time
            if self.last_publish_time is not None and self.publish_start_time is not None
            else 0.0
        )
        publish_window_elapsed = (
            self.last_publish_time - self.publish_window_first_time
            if self.last_publish_time is not None and self.publish_window_first_time is not None
            else 0.0
        )
        return FpsReport(
            frames=self.frames,
            receive_window_fps=self._elapsed_rate(
                self.receive_window_frames, receive_window_elapsed
            ),
            receive_average_fps=self._elapsed_rate(self.frames, receive_total_elapsed),
            publish_window_fps=self._interval_rate(
                self.publish_window_frames, publish_window_elapsed
            ),
            publish_average_fps=self._interval_rate(self.frames, publish_total_elapsed),
            estimated_drops=self.estimated_drops,
            last_publish_gap_seconds=self.last_publish_gap_seconds,
            max_publish_gap_seconds=self.max_publish_gap_seconds,
        )

    def reset_report_window(self, receive_now):
        self.receive_window_start_time = receive_now
        self.receive_window_frames = 0
        self.publish_window_first_time = None
        self.publish_window_frames = 0


def format_fps_report_line(prefix, topic, report, last_receive_age_ms=None):
    line = (
        f"{prefix} "
        f"{topic} "
        f"receive_window={report.receive_window_fps:.2f} "
        f"receive_avg={report.receive_average_fps:.2f} "
        f"publish_window={report.publish_window_fps:.2f} "
        f"publish_avg={report.publish_average_fps:.2f} "
        f"frames={report.frames} drops~={report.estimated_drops} "
        f"publish_last_gap_ms={report.last_publish_gap_seconds * 1000.0:.1f} "
        f"publish_max_gap_ms={report.max_publish_gap_seconds * 1000.0:.1f}"
    )
    if last_receive_age_ms is not None:
        line = f"{line} last_receive_age_ms={last_receive_age_ms:.1f}"
    return line


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Subscribe to ROS 2 Image/CameraInfo topics and report metadata, FPS, and drop estimates."
    )
    parser.add_argument(
        "--expected-fps",
        type=float,
        default=30.0,
        help="Expected image FPS used to estimate dropped frames from receive-time gaps.",
    )
    parser.add_argument(
        "--drop-gap-ratio",
        type=float,
        default=1.5,
        help="Treat a receive gap larger than expected_period * ratio as a likely drop.",
    )
    parser.add_argument(
        "--report-period",
        type=float,
        default=1.0,
        help="Seconds between FPS reports.",
    )
    parser.add_argument(
        "--drop-log-period",
        type=float,
        default=1.0,
        help="Minimum seconds between per-topic drop log lines; set 0 to log every detected gap.",
    )
    parser.add_argument(
        "--topics",
        nargs="*",
        default=[],
        help="Extra image topics to subscribe to immediately.",
    )
    parser.add_argument(
        "--camera-info-topics",
        nargs="*",
        default=[],
        help="Extra CameraInfo topics to subscribe to immediately.",
    )
    parser.add_argument(
        "--no-metadata-once",
        dest="metadata_once",
        action="store_false",
        default=True,
        help="Do not print one-time metadata for each first Image/CameraInfo sample.",
    )
    parser.add_argument(
        "--no-store-pixels",
        dest="store_pixels",
        action="store_false",
        default=True,
        help="Keep metadata only; do not retain the latest image byte payload.",
    )
    parser.add_argument(
        "--payload-pool-size",
        type=int,
        default=2,
        help="Reusable image payload buffers per topic when storing pixels.",
    )
    return parser.parse_args(argv)


def _unique(items):
    seen = set()
    result = []
    for item in items:
        if item not in seen:
            result.append(item)
            seen.add(item)
    return result


def run_monitor(args):
    configure_default_rmw()

    # Import ROS only at runtime so unit tests for the pure-Python accounting can
    # run without sourcing /opt/ros/jazzy/setup.bash.
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    class ImageTopicMonitor(Node):
        def __init__(self):
            super().__init__("image_topic_monitor")
            self.latest_images = {}
            self.image_stats = {}
            self.camera_info_stats = {}
            self.image_subscriptions = {}
            self.camera_info_subscriptions = {}
            self.printed_image_metadata = set()
            self.printed_camera_info_metadata = set()
            self.callback_group = ReentrantCallbackGroup()
            self.lock = threading.Lock()
            self.last_drop_log_time = {}
            self.payload_pool = (
                ImagePayloadPool(args.payload_pool_size) if args.store_pixels else None
            )

            image_topics = list(args.topics)
            if not image_topics:
                image_topics = DEFAULT_IMAGE_TOPICS
            camera_info_topics = list(args.camera_info_topics)
            if not camera_info_topics:
                camera_info_topics = DEFAULT_CAMERA_INFO_TOPICS

            for topic in _unique(image_topics):
                self.subscribe_image(topic)
            for topic in _unique(camera_info_topics):
                self.subscribe_camera_info(topic)

            self.create_timer(args.report_period, self.print_fps_report)
            print(
                "[monitor] image topic monitor started "
                f"expected_fps={args.expected_fps:.3f} report_period={args.report_period:.3f}s",
                flush=True,
            )

        def subscribe_image(self, topic):
            if topic in self.image_subscriptions:
                return
            self.image_stats[topic] = TopicFpsStats(args.expected_fps, args.drop_gap_ratio)
            self.image_subscriptions[topic] = self.create_subscription(
                Image,
                topic,
                lambda msg, topic=topic: self.on_image(topic, msg),
                qos_profile_sensor_data,
                callback_group=self.callback_group,
                # Keep image samples serialized so the monitor can read header
                # fields and retain the payload slice without full deserialization.
                raw=True,
            )
            print(f"[monitor] subscribed image {topic}", flush=True)

        def subscribe_camera_info(self, topic):
            if topic in self.camera_info_subscriptions:
                return
            self.camera_info_stats[topic] = TopicFpsStats(args.expected_fps, args.drop_gap_ratio)
            self.camera_info_subscriptions[topic] = self.create_subscription(
                CameraInfo,
                topic,
                lambda msg, topic=topic: self.on_camera_info(topic, msg),
                qos_profile_sensor_data,
                callback_group=self.callback_group,
            )
            print(f"[monitor] subscribed camera_info {topic}", flush=True)

        def on_image(self, topic, msg):
            image = parse_serialized_image(msg)
            receive_now = time.monotonic()
            sample_time = stamp_to_seconds(image.stamp_sec, image.stamp_nanosec)

            with self.lock:
                stats = self.image_stats[topic]
                previous_drops = stats.estimated_drops
                stats.record(publish_time=sample_time, receive_time=receive_now)

                data = payload_for_storage(
                    image.data,
                    args.store_pixels,
                    pool=self.payload_pool,
                    topic=topic,
                )
                self.latest_images[topic] = LatestImage(
                    received_monotonic=receive_now,
                    sample_time=sample_time,
                    frame_id=image.frame_id,
                    stamp_sec=image.stamp_sec,
                    stamp_nanosec=image.stamp_nanosec,
                    width=image.width,
                    height=image.height,
                    encoding=image.encoding,
                    step=image.step,
                    data_size=image.data_size,
                    data=data,
                )

                should_log_drop = stats.estimated_drops > previous_drops
                if should_log_drop and args.drop_log_period > 0:
                    last_log = self.last_drop_log_time.get(topic, 0.0)
                    should_log_drop = receive_now - last_log >= args.drop_log_period
                if should_log_drop:
                    self.last_drop_log_time[topic] = receive_now
                    print(
                        "[drop] "
                        f"{topic} publish_gap_ms={stats.last_publish_gap_seconds * 1000.0:.1f} "
                        f"estimated_total_drops={stats.estimated_drops}",
                        flush=True,
                    )

                if args.metadata_once and topic not in self.printed_image_metadata:
                    self.printed_image_metadata.add(topic)
                    print(
                        "[image-meta] "
                        f"{topic} frame_id={image.frame_id or '<empty>'} "
                        f"stamp={image.stamp_sec}.{image.stamp_nanosec:09d} "
                        f"encoding={image.encoding} size={image.width}x{image.height} "
                        f"step={image.step} data_bytes={image.data_size}",
                        flush=True,
                    )

        def on_camera_info(self, topic, msg):
            receive_now = time.monotonic()
            sample_time = stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)

            with self.lock:
                self.camera_info_stats[topic].record(
                    publish_time=sample_time,
                    receive_time=receive_now,
                )

                if not args.metadata_once or topic in self.printed_camera_info_metadata:
                    return
                self.printed_camera_info_metadata.add(topic)
                print(
                    "[camera-info-meta] "
                    f"{topic} frame_id={msg.header.frame_id or '<empty>'} "
                    f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
                    f"size={msg.width}x{msg.height} distortion_model={msg.distortion_model or '<empty>'} "
                    f"d_len={len(msg.d)} k0={msg.k[0]:.6g} k4={msg.k[4]:.6g}",
                    flush=True,
                )

        def print_fps_report(self):
            with self.lock:
                receive_now = time.monotonic()
                for topic in sorted(self.image_stats):
                    stats = self.image_stats[topic]
                    if stats.frames == 0:
                        print(f"[fps] {topic} waiting_for_frames", flush=True)
                        continue

                    latest = self.latest_images.get(topic)
                    report = stats.report(receive_now=receive_now)
                    last_receive_age_ms = (
                        (receive_now - latest.received_monotonic) * 1000.0
                        if latest
                        else 0.0
                    )
                    print(
                        format_fps_report_line(
                            "[fps]",
                            topic,
                            report,
                            last_receive_age_ms=last_receive_age_ms,
                        ),
                        flush=True,
                    )
                    stats.reset_report_window(receive_now=receive_now)

                for topic in sorted(self.camera_info_stats):
                    stats = self.camera_info_stats[topic]
                    if stats.frames == 0:
                        print(f"[camera-info-fps] {topic} waiting_for_frames", flush=True)
                        continue

                    report = stats.report(receive_now=receive_now)
                    print(format_fps_report_line("[camera-info-fps]", topic, report), flush=True)
                    stats.reset_report_window(receive_now=receive_now)

    rclpy.init()
    node = ImageTopicMonitor()

    def stop(_signum, _frame):
        raise SystemExit(0)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


def main(argv=None):
    args = parse_args(argv)
    try:
        run_monitor(args)
    except ImportError as exc:
        raise SystemExit(
            "Failed to import ROS 2 Python modules. Source ROS first, for example:\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  source /home/unitree/dev/shf3l_ros_ws/install/setup.bash\n"
            f"Original import error: {exc}"
        )


if __name__ == "__main__":
    main()
