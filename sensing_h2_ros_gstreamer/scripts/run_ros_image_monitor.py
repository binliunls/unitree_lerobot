#!/usr/bin/env python3
"""Example CLI for the reusable ROS image monitor API."""

import argparse
import signal
import time

from ros_image_monitor import (
    DEFAULT_CAMERA_INFO_TOPICS,
    DEFAULT_IMAGE_TOPICS,
    ImageMonitor,
    format_fps_report_line,
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Run the reusable ROS Image/CameraInfo monitor API as a CLI."
    )
    parser.add_argument("--expected-fps", type=float, default=30.0)
    parser.add_argument("--drop-gap-ratio", type=float, default=1.5)
    parser.add_argument("--report-period", type=float, default=1.0)
    parser.add_argument("--topics", nargs="*", default=[])
    parser.add_argument("--camera-info-topics", nargs="*", default=[])
    parser.add_argument("--no-store-pixels", dest="store_pixels", action="store_false", default=True)
    parser.add_argument("--payload-pool-size", type=int, default=2)
    parser.add_argument("--stale-after", type=float, default=None)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    image_topics = args.topics or list(DEFAULT_IMAGE_TOPICS)
    camera_info_topics = args.camera_info_topics or list(DEFAULT_CAMERA_INFO_TOPICS)
    monitor = ImageMonitor(
        image_topics=image_topics,
        camera_info_topics=camera_info_topics,
        expected_fps=args.expected_fps,
        drop_gap_ratio=args.drop_gap_ratio,
        store_pixels=args.store_pixels,
        payload_pool_size=args.payload_pool_size,
        stale_after=args.stale_after,
    )

    try:
        with monitor:
            print(
                "[monitor] reusable image monitor started "
                f"expected_fps={args.expected_fps:.3f} report_period={args.report_period:.3f}s",
                flush=True,
            )
            while not stop_requested:
                time.sleep(args.report_period)
                snapshot = monitor.snapshot()
                for topic in sorted(snapshot.image_stats):
                    report = snapshot.image_stats[topic]
                    latest = snapshot.images.get(topic)
                    age_ms = None
                    if latest is not None:
                        age_ms = (time.monotonic() - latest.received_monotonic) * 1000.0
                    print(format_fps_report_line("[fps]", topic, report, age_ms), flush=True)
                for topic in sorted(snapshot.camera_info_stats):
                    report = snapshot.camera_info_stats[topic]
                    print(format_fps_report_line("[camera-info-fps]", topic, report), flush=True)
    except ImportError as exc:
        raise SystemExit(
            "Failed to import ROS 2 Python modules. Source ROS first, for example:\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  source /home/unitree/dev/shf3l_ros_ws/install/setup.bash\n"
            f"Original import error: {exc}"
        )


if __name__ == "__main__":
    main()
