#!/usr/bin/env python3
"""Reusable ROS 2 image and CameraInfo monitor helpers.

This module is meant for application code. It keeps ROS imports lazy so the
pure-Python API can be imported and lightly tested without sourcing ROS first.
"""

import os
import threading
import time
from dataclasses import dataclass, field
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
class LatestCameraInfo:
    received_monotonic: float
    sample_time: float
    frame_id: str
    stamp_sec: int
    stamp_nanosec: int
    width: int
    height: int
    distortion_model: str
    d: tuple = field(default_factory=tuple)
    k: tuple = field(default_factory=tuple)
    r: tuple = field(default_factory=tuple)
    p: tuple = field(default_factory=tuple)
    message: object = None


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


@dataclass
class TopicStatus:
    topic: str
    frames: int
    last_receive_age_seconds: float
    estimated_drops: int
    is_stale: bool


@dataclass
class MonitorSnapshot:
    images: dict
    camera_infos: dict
    image_stats: dict
    camera_info_stats: dict
    statuses: dict


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
    raw = memoryview(raw_data)
    offset = 4
    stamp_sec, offset = _read_i32(raw, offset)
    stamp_nanosec, offset = _read_u32(raw, offset)
    frame_id, offset = _read_string(raw, offset)
    offset = align4(offset)
    height, offset = _read_u32(raw, offset)
    width, offset = _read_u32(raw, offset)
    encoding, offset = _read_string(raw, offset)
    offset += 1
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


class TopicFpsStats:
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
                self.estimated_drops += max(1, round(gap / self.expected_period) - 1)

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
            receive_window_fps=self._elapsed_rate(self.receive_window_frames, receive_window_elapsed),
            receive_average_fps=self._elapsed_rate(self.frames, receive_total_elapsed),
            publish_window_fps=self._interval_rate(self.publish_window_frames, publish_window_elapsed),
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


class ImageTopicSubscriber:
    """Reusable image/CameraInfo subscriber state and callbacks.

    Pass an existing `rclpy` node to `attach_to_node()` when the client owns ROS.
    Use `record_serialized_image()` and `record_camera_info()` directly in tests
    or custom subscription setups.
    """

    def __init__(
        self,
        image_topics=None,
        camera_info_topics=None,
        expected_fps=30.0,
        drop_gap_ratio=1.5,
        store_pixels=True,
        payload_pool_size=2,
        stale_after=None,
        node=None,
        qos_profile=None,
    ):
        self.image_topics = list(DEFAULT_IMAGE_TOPICS if image_topics is None else image_topics)
        self.camera_info_topics = list(
            DEFAULT_CAMERA_INFO_TOPICS if camera_info_topics is None else camera_info_topics
        )
        self.expected_fps = float(expected_fps)
        self.drop_gap_ratio = float(drop_gap_ratio)
        self.store_pixels = bool(store_pixels)
        self.stale_after = stale_after
        self.qos_profile = qos_profile
        self.latest_images = {}
        self.latest_camera_infos = {}
        self.image_stats = {
            topic: TopicFpsStats(self.expected_fps, self.drop_gap_ratio)
            for topic in _unique(self.image_topics)
        }
        self.camera_info_stats = {
            topic: TopicFpsStats(self.expected_fps, self.drop_gap_ratio)
            for topic in _unique(self.camera_info_topics)
        }
        self.image_subscriptions = {}
        self.camera_info_subscriptions = {}
        self.image_callbacks = []
        self.camera_info_callbacks = []
        self.drop_callbacks = []
        self.stats_callbacks = []
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.payload_pool = ImagePayloadPool(payload_pool_size) if self.store_pixels else None
        if node is not None:
            self.attach_to_node(node)

    def add_image_callback(self, callback):
        self.image_callbacks.append(callback)

    def add_camera_info_callback(self, callback):
        self.camera_info_callbacks.append(callback)

    def add_drop_callback(self, callback):
        self.drop_callbacks.append(callback)

    def add_stats_callback(self, callback):
        self.stats_callbacks.append(callback)

    def subscribe_image(self, topic):
        with self.lock:
            self.image_stats.setdefault(topic, TopicFpsStats(self.expected_fps, self.drop_gap_ratio))
            if topic not in self.image_topics:
                self.image_topics.append(topic)

    def subscribe_camera_info(self, topic):
        with self.lock:
            self.camera_info_stats.setdefault(
                topic, TopicFpsStats(self.expected_fps, self.drop_gap_ratio)
            )
            if topic not in self.camera_info_topics:
                self.camera_info_topics.append(topic)

    def attach_to_node(self, node):
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image

        callback_group = ReentrantCallbackGroup()
        qos_profile = self.qos_profile or qos_profile_sensor_data
        for topic in _unique(self.image_topics):
            if topic in self.image_subscriptions:
                continue
            self.image_subscriptions[topic] = node.create_subscription(
                Image,
                topic,
                lambda msg, topic=topic: self.record_serialized_image(topic, msg),
                qos_profile,
                callback_group=callback_group,
                raw=True,
            )
        for topic in _unique(self.camera_info_topics):
            if topic in self.camera_info_subscriptions:
                continue
            self.camera_info_subscriptions[topic] = node.create_subscription(
                CameraInfo,
                topic,
                lambda msg, topic=topic: self.record_camera_info(topic, msg),
                qos_profile,
                callback_group=callback_group,
            )

    def record_serialized_image(self, topic, raw_msg, receive_now=None):
        image = parse_serialized_image(raw_msg)
        receive_now = time.monotonic() if receive_now is None else receive_now
        sample_time = stamp_to_seconds(image.stamp_sec, image.stamp_nanosec)
        callbacks = []
        drop_callbacks = []

        with self.condition:
            stats = self.image_stats.setdefault(
                topic, TopicFpsStats(self.expected_fps, self.drop_gap_ratio)
            )
            previous_drops = stats.estimated_drops
            stats.record(publish_time=sample_time, receive_time=receive_now)
            data = payload_for_storage(
                image.data,
                self.store_pixels,
                pool=self.payload_pool,
                topic=topic,
            )
            latest = LatestImage(
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
            self.latest_images[topic] = latest
            callbacks = list(self.image_callbacks)
            if stats.estimated_drops > previous_drops:
                report = stats.report(receive_now=receive_now)
                drop_callbacks = list(self.drop_callbacks)
            else:
                report = None
            self.condition.notify_all()

        for callback in callbacks:
            callback(topic, latest)
        for callback in drop_callbacks:
            callback(topic, report)
        return latest

    def record_camera_info(self, topic, msg, receive_now=None):
        receive_now = time.monotonic() if receive_now is None else receive_now
        stamp = msg.header.stamp
        sample_time = stamp_to_seconds(stamp.sec, stamp.nanosec)
        latest = LatestCameraInfo(
            received_monotonic=receive_now,
            sample_time=sample_time,
            frame_id=msg.header.frame_id,
            stamp_sec=stamp.sec,
            stamp_nanosec=stamp.nanosec,
            width=msg.width,
            height=msg.height,
            distortion_model=msg.distortion_model,
            d=tuple(msg.d),
            k=tuple(msg.k),
            r=tuple(msg.r),
            p=tuple(msg.p),
            message=msg,
        )
        callbacks = []
        with self.condition:
            stats = self.camera_info_stats.setdefault(
                topic, TopicFpsStats(self.expected_fps, self.drop_gap_ratio)
            )
            stats.record(publish_time=sample_time, receive_time=receive_now)
            self.latest_camera_infos[topic] = latest
            callbacks = list(self.camera_info_callbacks)
            self.condition.notify_all()

        for callback in callbacks:
            callback(topic, latest)
        return latest

    def get_latest_image(self, topic):
        with self.lock:
            return self.latest_images.get(topic)

    def get_latest_camera_info(self, topic):
        with self.lock:
            return self.latest_camera_infos.get(topic)

    def wait_for_image(self, topic, timeout=None):
        deadline = None if timeout is None else time.monotonic() + timeout
        with self.condition:
            while topic not in self.latest_images:
                if deadline is None:
                    self.condition.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.condition.wait(remaining)
            return self.latest_images[topic]

    def get_image_stats(self, topic, receive_now=None):
        with self.lock:
            stats = self.image_stats.get(topic)
            if stats is None:
                return None
            return stats.report(time.monotonic() if receive_now is None else receive_now)

    def get_camera_info_stats(self, topic, receive_now=None):
        with self.lock:
            stats = self.camera_info_stats.get(topic)
            if stats is None:
                return None
            return stats.report(time.monotonic() if receive_now is None else receive_now)

    def get_topic_status(self, topic, receive_now=None):
        receive_now = time.monotonic() if receive_now is None else receive_now
        with self.lock:
            latest = self.latest_images.get(topic) or self.latest_camera_infos.get(topic)
            stats = self.image_stats.get(topic) or self.camera_info_stats.get(topic)
            frames = stats.frames if stats else 0
            drops = stats.estimated_drops if stats else 0
            age = receive_now - latest.received_monotonic if latest else float("inf")
            is_stale = self.stale_after is not None and age > self.stale_after
            return TopicStatus(topic, frames, age, drops, is_stale)

    def snapshot(self, receive_now=None):
        receive_now = time.monotonic() if receive_now is None else receive_now
        with self.lock:
            image_stats = {
                topic: stats.report(receive_now) for topic, stats in self.image_stats.items()
            }
            camera_info_stats = {
                topic: stats.report(receive_now)
                for topic, stats in self.camera_info_stats.items()
            }
            statuses = {
                topic: self.get_topic_status(topic, receive_now=receive_now)
                for topic in set(image_stats) | set(camera_info_stats)
            }
            return MonitorSnapshot(
                images=dict(self.latest_images),
                camera_infos=dict(self.latest_camera_infos),
                image_stats=image_stats,
                camera_info_stats=camera_info_stats,
                statuses=statuses,
            )


class ImageMonitor:
    """Convenience wrapper that can own a ROS node and executor thread."""

    def __init__(
        self,
        image_topics=None,
        camera_info_topics=None,
        expected_fps=30.0,
        drop_gap_ratio=1.5,
        store_pixels=True,
        payload_pool_size=2,
        stale_after=None,
        node_name="ros_image_monitor",
        auto_start_ros=True,
        subscriber_factory=None,
    ):
        self.options = {
            "image_topics": image_topics,
            "camera_info_topics": camera_info_topics,
            "expected_fps": expected_fps,
            "drop_gap_ratio": drop_gap_ratio,
            "store_pixels": store_pixels,
            "payload_pool_size": payload_pool_size,
            "stale_after": stale_after,
        }
        self.node_name = node_name
        self.auto_start_ros = auto_start_ros
        self.subscriber_factory = subscriber_factory or ImageTopicSubscriber
        self.subscriber = None
        self.node = None
        self.executor = None
        self.thread = None
        self._started_rclpy = False
        self.is_running = False

    def start(self):
        if self.is_running:
            return self
        self.subscriber = self.subscriber_factory(**self.options)
        if self.auto_start_ros:
            configure_default_rmw()
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node

            if not rclpy.ok():
                rclpy.init()
                self._started_rclpy = True
            self.node = Node(self.node_name)
            self.subscriber.attach_to_node(self.node)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.thread.start()
        self.is_running = True
        return self

    def stop(self):
        if not self.is_running:
            return
        if self.executor is not None:
            self.executor.shutdown()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        if self.node is not None:
            self.node.destroy_node()
        if self._started_rclpy:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        self.is_running = False

    close = stop

    def __enter__(self):
        return self.start()

    def __exit__(self, _exc_type, _exc, _tb):
        self.stop()

    def __getattr__(self, name):
        if self.subscriber is not None and hasattr(self.subscriber, name):
            return getattr(self.subscriber, name)
        raise AttributeError(name)


def _unique(items):
    seen = set()
    result = []
    for item in items:
        if item not in seen:
            result.append(item)
            seen.add(item)
    return result
