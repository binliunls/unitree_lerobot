#!/usr/bin/env python3
"""Measure per-stage GStreamer camera rates without ROS/DDS in the path."""

import argparse
import signal
import time
from dataclasses import dataclass

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GLib", "2.0")
from gi.repository import GLib, Gst  # noqa: E402


@dataclass
class StageStats:
    count: int = 0
    last_count: int = 0
    first_time: float | None = None
    last_time: float | None = None

    def record(self):
        now = time.monotonic()
        if self.first_time is None:
            self.first_time = now
        self.last_time = now
        self.count += 1

    def window_rate(self, elapsed):
        frames = self.count - self.last_count
        self.last_count = self.count
        return frames / elapsed if elapsed > 0 else 0.0

    def average_rate(self, now):
        if self.first_time is None:
            return 0.0
        return self.count / max(now - self.first_time, 1e-9)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Run a camera GStreamer pipeline and print per-stage buffer rates."
    )
    parser.add_argument("--kind", choices=("wrist", "argus"), required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--report-period", type=float, default=1.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--videoconvert-threads", type=int, default=2)
    parser.add_argument("--device", default="/dev/video2")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--capture-width", type=int, default=1920)
    parser.add_argument("--capture-height", type=int, default=1080)
    parser.add_argument("--output-width", type=int, default=640)
    parser.add_argument("--output-height", type=int, default=480)
    return parser.parse_args(argv)


def identity(name):
    return f"identity name={name} signal-handoffs=true silent=true"


def build_pipeline(args):
    if args.kind == "wrist":
        return (
            f"v4l2src device={args.device} do-timestamp=true ! "
            f"video/x-raw,format=UYVY,width={args.width},height={args.height},framerate={args.fps}/1 ! "
            f"{identity('after_source')} ! "
            "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
            f"{identity('after_queue')} ! "
            f"videoscale ! video/x-raw,format=UYVY,width={args.output_width},height={args.output_height},framerate={args.fps}/1 ! "
            f"{identity('after_videoscale')} ! "
            f"videoconvert n-threads={args.videoconvert_threads} ! "
            f"video/x-raw,format=RGB,width={args.output_width},height={args.output_height},framerate={args.fps}/1 ! "
            f"{identity('after_videoconvert')} ! "
            "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
        )

    return (
        f"nvarguscamerasrc sensor-id={args.sensor_id} ! "
        "video/x-raw(memory:NVMM),"
        f"width={args.capture_width},height={args.capture_height},framerate={args.fps}/1 ! "
        f"{identity('after_source')} ! "
        "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! "
        f"{identity('after_queue')} ! "
        f"nvvidconv ! video/x-raw,format=NV12,width={args.output_width},height={args.output_height} ! "
        f"{identity('after_nvvidconv')} ! "
        f"videoconvert n-threads={args.videoconvert_threads} ! "
        "video/x-raw,"
        f"format=RGB,width={args.output_width},height={args.output_height},framerate={args.fps}/1 ! "
        f"{identity('after_videoconvert')} ! "
        "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
    )


def run_probe(args):
    Gst.init(None)
    pipeline_text = build_pipeline(args)
    print(f"[pipeline] {pipeline_text}", flush=True)
    pipeline = Gst.parse_launch(pipeline_text)

    stats = {}
    for name in (
        "after_source",
        "after_queue",
        "after_videoscale",
        "after_nvvidconv",
        "after_videoconvert",
    ):
        element = pipeline.get_by_name(name)
        if not element:
            continue
        stats[name] = StageStats()
        element.connect("handoff", lambda _element, _buffer, name=name: stats[name].record())

    sink = pipeline.get_by_name("sink")
    stats["appsink"] = StageStats()

    def on_sample(appsink):
        sample = appsink.emit("pull-sample")
        if sample:
            stats["appsink"].record()
        return Gst.FlowReturn.OK

    sink.connect("new-sample", on_sample)

    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_message(_bus, message):
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"[error] {err}: {debug}", flush=True)
            loop.quit()
        elif message.type == Gst.MessageType.EOS:
            loop.quit()

    bus.connect("message", on_message)

    last_report = time.monotonic()

    def report():
        nonlocal last_report
        now = time.monotonic()
        elapsed = now - last_report
        last_report = now
        parts = []
        for name in sorted(stats):
            stage = stats[name]
            parts.append(
                f"{name}:window={stage.window_rate(elapsed):.2f},avg={stage.average_rate(now):.2f},count={stage.count}"
            )
        print("[rates] " + " ".join(parts), flush=True)
        return True

    def stop():
        loop.quit()
        return False

    def handle_signal(_signum, _frame):
        loop.quit()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    GLib.timeout_add(int(args.report_period * 1000), report)
    GLib.timeout_add(int(args.duration * 1000), stop)

    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)
        report()


def main(argv=None):
    run_probe(parse_args(argv))


if __name__ == "__main__":
    main()
