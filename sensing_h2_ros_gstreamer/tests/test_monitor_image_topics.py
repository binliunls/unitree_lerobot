import importlib.util
import pathlib
import os
import unittest


SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "monitor_image_topics.py"


def load_monitor_module():
    spec = importlib.util.spec_from_file_location("monitor_image_topics", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TopicFpsStatsTest(unittest.TestCase):
    def test_configure_default_rmw_prefers_cyclonedds_without_overriding(self):
        monitor = load_monitor_module()
        original = os.environ.get("RMW_IMPLEMENTATION")
        try:
            os.environ.pop("RMW_IMPLEMENTATION", None)
            monitor.configure_default_rmw()
            self.assertEqual(os.environ["RMW_IMPLEMENTATION"], "rmw_cyclonedds_cpp")

            os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
            monitor.configure_default_rmw()
            self.assertEqual(os.environ["RMW_IMPLEMENTATION"], "rmw_fastrtps_cpp")
        finally:
            if original is None:
                os.environ.pop("RMW_IMPLEMENTATION", None)
            else:
                os.environ["RMW_IMPLEMENTATION"] = original

    def test_parse_args_defaults_and_disable_flags(self):
        monitor = load_monitor_module()
        defaults = monitor.parse_args([])

        self.assertTrue(defaults.store_pixels)
        self.assertTrue(defaults.metadata_once)
        self.assertEqual(defaults.payload_pool_size, 2)
        self.assertFalse(monitor.parse_args(["--no-store-pixels"]).store_pixels)
        self.assertFalse(monitor.parse_args(["--no-metadata-once"]).metadata_once)

    def test_parse_serialized_image_reads_metadata_and_payload(self):
        monitor = load_monitor_module()
        raw = bytes.fromhex(
            "00010000"
            "0c000000"
            "40489014"
            "07000000"
            "63616d6572610074"
            "e0010000"
            "80020000"
            "05000000"
            "726762380000735f"
            "80070000"
            "04000000"
            "01020304"
        )

        image = monitor.parse_serialized_image(raw)

        self.assertEqual(image.stamp_sec, 12)
        self.assertEqual(image.stamp_nanosec, 345_000_000)
        self.assertEqual(image.frame_id, "camera")
        self.assertEqual(image.width, 640)
        self.assertEqual(image.height, 480)
        self.assertEqual(image.encoding, "rgb8")
        self.assertEqual(image.step, 1920)
        self.assertEqual(image.data_size, 4)
        self.assertEqual(bytes(image.data), b"\x01\x02\x03\x04")
        self.assertAlmostEqual(
            monitor.stamp_to_seconds(image.stamp_sec, image.stamp_nanosec),
            12.345,
        )

    def test_payload_storage_and_pool_reuse(self):
        monitor = load_monitor_module()
        payload = bytearray(b"abc")

        self.assertIs(monitor.payload_for_storage(payload, store_pixels=True).obj, payload)
        self.assertEqual(monitor.payload_for_storage(payload, store_pixels=False), b"")

        pool = monitor.ImagePayloadPool(slots_per_topic=2)

        first = pool.store("/camera", b"abc")
        second = pool.store("/camera", b"def")
        third = pool.store("/camera", b"ghi")

        self.assertEqual(bytes(first), b"ghi")
        self.assertEqual(bytes(second), b"def")
        self.assertIs(first.obj, third.obj)
        self.assertIsNot(first.obj, second.obj)

        small = pool.store("/camera", b"abc")
        large = pool.store("/camera", b"abcdef")

        self.assertEqual(bytes(small), b"abc")
        self.assertEqual(bytes(large), b"abcdef")
        self.assertEqual(len(large), 6)

    def test_fps_report_separates_receive_and_publish_rates(self):
        monitor = load_monitor_module()
        stats = monitor.TopicFpsStats(expected_fps=30.0, drop_gap_ratio=1.5)

        for index in range(30):
            stats.record(
                publish_time=100.0 + index / 30.0,
                receive_time=200.0 + index / 30.0,
            )
        report = stats.report(receive_now=201.0)

        self.assertAlmostEqual(report.receive_window_fps, 30.0)
        self.assertAlmostEqual(report.receive_average_fps, 30.0)
        self.assertAlmostEqual(report.publish_window_fps, 30.0)
        self.assertAlmostEqual(report.publish_average_fps, 30.0)
        self.assertEqual(report.frames, 30)
        self.assertEqual(report.estimated_drops, 0)

        stats.reset_report_window(receive_now=201.0)
        for index in range(30, 60):
            stats.record(
                publish_time=100.0 + index / 30.0,
                receive_time=200.0 + index / 30.0,
            )
        next_report = stats.report(receive_now=202.0)

        self.assertAlmostEqual(next_report.receive_window_fps, 30.0)
        self.assertAlmostEqual(next_report.publish_window_fps, 30.0)
        self.assertEqual(next_report.frames, 60)

    def test_receive_rate_survives_stale_publish_stamps_and_publish_gaps_count_drops(self):
        monitor = load_monitor_module()
        stats = monitor.TopicFpsStats(expected_fps=30.0, drop_gap_ratio=1.5)

        for index in range(30):
            stats.record(publish_time=1000.0, receive_time=2000.0 + index / 30.0)
        report = stats.report(receive_now=2001.0)

        self.assertAlmostEqual(report.receive_window_fps, 30.0)
        self.assertEqual(report.publish_window_fps, 0.0)
        self.assertEqual(report.estimated_drops, 0)

        gap_stats = monitor.TopicFpsStats(expected_fps=30.0, drop_gap_ratio=1.5)
        gap_stats.record(publish_time=10.0, receive_time=20.0)
        gap_stats.record(publish_time=10.1, receive_time=20.1)

        self.assertEqual(gap_stats.estimated_drops, 2)
        self.assertAlmostEqual(gap_stats.max_publish_gap_seconds, 0.1)

    def test_format_fps_report_line_uses_explicit_receive_and_publish_names(self):
        monitor = load_monitor_module()
        report = monitor.FpsReport(
            frames=30,
            receive_window_fps=29.5,
            receive_average_fps=29.75,
            publish_window_fps=30.0,
            publish_average_fps=30.1,
            estimated_drops=1,
            last_publish_gap_seconds=0.04,
            max_publish_gap_seconds=0.08,
        )

        image_line = monitor.format_fps_report_line(
            "[fps]",
            "/head/left/image_raw",
            report,
            last_receive_age_ms=4.1,
        )
        camera_info_line = monitor.format_fps_report_line(
            "[camera-info-fps]",
            "/head/left/camera_info",
            report,
        )

        self.assertEqual(
            image_line,
            "[fps] /head/left/image_raw "
            "receive_window=29.50 receive_avg=29.75 "
            "publish_window=30.00 publish_avg=30.10 "
            "frames=30 drops~=1 publish_last_gap_ms=40.0 publish_max_gap_ms=80.0 "
            "last_receive_age_ms=4.1",
        )
        self.assertEqual(
            camera_info_line,
            "[camera-info-fps] /head/left/camera_info "
            "receive_window=29.50 receive_avg=29.75 "
            "publish_window=30.00 publish_avg=30.10 "
            "frames=30 drops~=1 publish_last_gap_ms=40.0 publish_max_gap_ms=80.0",
        )


if __name__ == "__main__":
    unittest.main()
