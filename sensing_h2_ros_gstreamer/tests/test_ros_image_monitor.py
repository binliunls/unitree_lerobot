import importlib.util
import pathlib
import time
import unittest


MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "ros_image_monitor.py"


def load_monitor_module():
    spec = importlib.util.spec_from_file_location("ros_image_monitor", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RosImageMonitorApiTest(unittest.TestCase):
    def test_subscriber_records_images_callbacks_and_snapshots_without_ros_runtime(self):
        monitor = load_monitor_module()
        subscriber = monitor.ImageTopicSubscriber(
            image_topics=["/camera/image_raw"],
            camera_info_topics=[],
            expected_fps=30.0,
            store_pixels=True,
        )
        seen = []
        subscriber.add_image_callback(lambda topic, image: seen.append((topic, image)))

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

        image = subscriber.record_serialized_image("/camera/image_raw", raw, receive_now=10.0)
        snapshot = subscriber.snapshot(receive_now=10.1)

        self.assertEqual(image.encoding, "rgb8")
        self.assertEqual(bytes(image.data), b"\x01\x02\x03\x04")
        self.assertEqual(seen[0][0], "/camera/image_raw")
        self.assertIs(snapshot.images["/camera/image_raw"], image)
        self.assertEqual(snapshot.image_stats["/camera/image_raw"].frames, 1)
        self.assertFalse(snapshot.statuses["/camera/image_raw"].is_stale)

    def test_wait_for_image_and_background_wrapper_lifecycle_are_available(self):
        monitor = load_monitor_module()
        subscriber = monitor.ImageTopicSubscriber(image_topics=["/camera/image_raw"])

        self.assertIsNone(subscriber.wait_for_image("/camera/image_raw", timeout=0.01))
        subscriber.record_serialized_image("/camera/image_raw", _sample_raw_image(), receive_now=time.monotonic())
        self.assertIsNotNone(subscriber.wait_for_image("/camera/image_raw", timeout=0.01))

        runner = monitor.ImageMonitor(
            image_topics=["/camera/image_raw"],
            auto_start_ros=False,
            subscriber_factory=lambda **kwargs: subscriber,
        )
        with runner:
            self.assertTrue(runner.is_running)
            self.assertIs(runner.get_latest_image("/camera/image_raw"), subscriber.get_latest_image("/camera/image_raw"))
        self.assertFalse(runner.is_running)


def _sample_raw_image():
    return bytes.fromhex(
        "00010000"
        "01000000"
        "00000000"
        "07000000"
        "63616d6572610074"
        "01000000"
        "01000000"
        "05000000"
        "726762380000735f"
        "03000000"
        "03000000"
        "010203"
    )


if __name__ == "__main__":
    unittest.main()
