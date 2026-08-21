import hashlib
import json
import struct
import tempfile
import unittest
import uuid
from pathlib import Path

import numpy as np

from unitree_lerobot.utils.h2_sharpa_native_capture import (
    NativeCaptureError,
    load_capture_tools,
    load_native_capture,
)


CAPTURE_TOOLS_DIR = (
    Path(__file__).resolve().parents[2] / "sharpa-teleop" / "thor" / "docker_5_0_1"
)
TELEMETRY_STRUCT = struct.Struct("<4sBBBBQQQiQ22f")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _tactile_payload(pb, channel: int, sequence: int) -> bytes:
    msg = pb.Tactile()
    msg.deform.height = 2
    msg.deform.width = 3
    msg.deform.data = bytes((channel, sequence % 256, 1, 2, 3, 4))
    msg.force6d.force.x = channel + 0.1
    msg.force6d.force.y = channel + 0.2
    msg.force6d.force.z = channel + 0.3
    msg.force6d.torque.x = channel + 0.4
    msg.force6d.torque.y = channel + 0.5
    msg.force6d.torque.z = channel + 0.6
    msg.rate = 180.0
    msg.contact_point.extend((float(channel), 0.5, 1.0))
    msg.contact_point_shape.extend((1, 3))
    msg.stream_schema_version = 1
    msg.publisher_session_id = "test-session"
    msg.source_frame_id = sequence + 1000
    msg.source_timestamp = 100.0 + sequence / 180.0
    msg.publisher_sequence = sequence
    msg.receive_monotonic_ns = 10_000 + sequence
    msg.receive_realtime_ns = 20_000 + sequence
    msg.channel = channel
    msg.hand_side = "right" if channel < 5 else "left"
    msg.publisher_dropped_total = 0
    msg.publisher_queue_depth = 2
    msg.stream_protocol = "sharpa.tactile.v1"
    msg.tactile_config_sha256 = "a" * 64
    msg.tactile_config_path = "/workspace/config/tactile_180_dual.json"
    msg.record_type = "sample"
    return msg.SerializeToString()


class NativeCaptureReaderTest(unittest.TestCase):
    def setUp(self) -> None:
        if not (CAPTURE_TOOLS_DIR / "sharpa_capture_format.py").is_file():
            self.skipTest(f"Sharpa capture tools are unavailable at {CAPTURE_TOOLS_DIR}")
        self.temp = tempfile.TemporaryDirectory()
        self.episode = Path(self.temp.name) / "episode_0001"
        self.capture_dir = self.episode / "sharpa_native_capture"
        self.capture_dir.mkdir(parents=True)
        self.capture_id = str(uuid.uuid4())
        self.tools = load_capture_tools(CAPTURE_TOOLS_DIR)
        self._write_fixture()

    def tearDown(self) -> None:
        self.temp.cleanup()

    def _write_fixture(self) -> None:
        fmt = self.tools.capture_format
        chunks = []

        writer = fmt.ChunkWriter(self.capture_dir, 0, compression="none")
        for channel in range(5):
            writer.add(
                fmt.CaptureRecord(
                    fmt.STREAM_TACTILE,
                    1_000 + channel,
                    2_000 + channel,
                    _tactile_payload(self.tools.protobuf, channel, channel + 1),
                )
            )
        telemetry = TELEMETRY_STRUCT.pack(
            b"SHT1",
            1,
            3,
            1,
            1,
            77,
            5_000,
            6_000,
            0,
            70,
            *[float(value) for value in range(22)],
        )
        writer.add(fmt.CaptureRecord(fmt.STREAM_HAND_TELEMETRY, 1_100, 2_100, telemetry))
        chunks.append(writer.close())

        writer = fmt.ChunkWriter(self.capture_dir, 1, compression="none")
        for channel in range(5, 10):
            writer.add(
                fmt.CaptureRecord(
                    fmt.STREAM_TACTILE,
                    1_200 + channel,
                    2_200 + channel,
                    _tactile_payload(self.tools.protobuf, channel, channel + 1),
                )
            )
        # A second left-thumb event appears later in file order but has an
        # earlier recorder timestamp; canonical arrays must sort it stably.
        writer.add(
            fmt.CaptureRecord(
                fmt.STREAM_TACTILE,
                1_050,
                2_050,
                _tactile_payload(self.tools.protobuf, 9, 99),
            )
        )
        chunks.append(writer.close())

        clock_path = self.capture_dir / "clock_samples.jsonl"
        clock_path.write_text('{"op":"START"}\n', encoding="utf-8")
        total_records = sum(chunk["records"] for chunk in chunks)
        total_streams: dict[str, int] = {}
        for chunk in chunks:
            for stream, count in chunk["stream_counts"].items():
                total_streams[stream] = total_streams.get(stream, 0) + count
        manifest = {
            "format": "sharpa.capture.manifest.v1",
            "version": 1,
            "capture_id": self.capture_id,
            "state": "finalized",
            "valid": True,
            "invalid_reasons": [],
            "chunks": chunks,
            "counts": {
                "written": total_records,
                **{f"written_stream_{stream}": count for stream, count in total_streams.items()},
            },
            "control_clock": {
                "closed": True,
                "writer_error": None,
                "sha256": _sha256(clock_path),
            },
        }
        self._write_manifest(manifest)
        metadata = {
            "protocol": "sharpa.capture.v1",
            "capture_id": self.capture_id,
            "status": "complete",
            "valid": True,
            "degraded_reasons": [],
        }
        (self.episode / "sharpa_native_capture.json").write_text(
            json.dumps(metadata) + "\n", encoding="utf-8"
        )

    def _write_manifest(self, manifest: dict) -> None:
        manifest_path = self.capture_dir / "manifest.json"
        manifest_path.write_text(json.dumps(manifest, sort_keys=True) + "\n", encoding="utf-8")
        (self.capture_dir / "manifest.sha256").write_text(
            f"{_sha256(manifest_path)}  manifest.json\n", encoding="ascii"
        )

    def test_decodes_references_telemetry_and_canonical_arrays(self) -> None:
        capture = load_native_capture(self.episode, CAPTURE_TOOLS_DIR)

        self.assertEqual(capture.capture_id, self.capture_id)
        self.assertEqual([chunk.name for chunk in capture.chunks], ["chunk-000000.shc", "chunk-000001.shc"])
        self.assertEqual(len(capture.tactile_records), 11)
        self.assertEqual(len(capture.hand_telemetry_records), 1)
        event = capture.tactile_records[0]
        self.assertEqual((event.global_record_index, event.event_index, event.record_index), (0, 0, 0))
        self.assertEqual((event.deform_height, event.deform_width, event.deform_byte_length), (2, 3, 6))
        self.assertEqual(event.contact_point_shape, (1, 3))
        self.assertEqual(event.publisher_session_id, "test-session")
        np.testing.assert_allclose(event.f6, np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float32))

        telemetry = capture.hand_telemetry_records[0]
        self.assertEqual((telemetry.kind_name, telemetry.side_name, telemetry.sequence), ("state", "left", 77))
        self.assertTrue(telemetry.complete)
        np.testing.assert_array_equal(telemetry.values, np.arange(22, dtype=np.float32))

        channels = capture.canonical_channel_arrays()
        self.assertEqual([channel.raw_channel for channel in channels], [9, 8, 7, 6, 5, 4, 3, 2, 1, 0])
        self.assertEqual(channels[0].canonical_name, "left_thumb")
        np.testing.assert_array_equal(channels[0].recorder_receive_monotonic_ns, [1_050, 1_209])
        np.testing.assert_array_equal(channels[0].event_index, [10, 9])
        self.assertEqual(channels[0].f6.shape, (2, 6))

    def test_rejects_incomplete_workstation_metadata(self) -> None:
        metadata_path = self.episode / "sharpa_native_capture.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        metadata["status"] = "recording"
        metadata["valid"] = False
        metadata_path.write_text(json.dumps(metadata) + "\n", encoding="utf-8")

        with self.assertRaisesRegex(NativeCaptureError, "not complete and valid"):
            load_native_capture(self.episode, CAPTURE_TOOLS_DIR)

    def test_rejects_capture_id_and_chunk_stream_count_mismatches(self) -> None:
        manifest_path = self.capture_dir / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["capture_id"] = str(uuid.uuid4())
        self._write_manifest(manifest)
        with self.assertRaisesRegex(NativeCaptureError, "capture_id does not match"):
            load_native_capture(self.episode, CAPTURE_TOOLS_DIR)

        manifest["capture_id"] = self.capture_id
        manifest["chunks"][0]["stream_counts"]["1"] += 1
        self._write_manifest(manifest)
        with self.assertRaisesRegex(NativeCaptureError, "stream counts mismatch"):
            load_native_capture(self.episode, CAPTURE_TOOLS_DIR)


if __name__ == "__main__":
    unittest.main()

