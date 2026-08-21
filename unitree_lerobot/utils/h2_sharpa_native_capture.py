"""Strict reader for fetched Thor-native Sharpa capture sidecars.

This module deliberately keeps the lossless deformation payload in the source
``.shc`` file.  Decoded rows contain the chunk/record coordinates needed to
retrieve it later, while the in-memory representation keeps only its declared
shape and byte length.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import struct
import sys
import uuid
from collections import Counter
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from types import ModuleType
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


CAPTURE_METADATA_FILENAME = "sharpa_native_capture.json"
FETCHED_CAPTURE_DIRNAME = "sharpa_native_capture"
CAPTURE_PROTOCOL = "sharpa.capture.v1"
MANIFEST_FORMAT = "sharpa.capture.manifest.v1"
TACTILE_PROTOCOL = "sharpa.tactile.v1"

# Hardware channels arrive as R pinky..thumb, L pinky..thumb.  The policy and
# legacy dataset order is L thumb..pinky, R thumb..pinky.
CANONICAL_FINGER_NAMES = (
    "left_thumb",
    "left_index",
    "left_middle",
    "left_ring",
    "left_pinky",
    "right_thumb",
    "right_index",
    "right_middle",
    "right_ring",
    "right_pinky",
)
RAW_CHANNEL_FOR_CANONICAL = (9, 8, 7, 6, 5, 4, 3, 2, 1, 0)

TELEMETRY_STRUCT = struct.Struct("<4sBBBBQQQiQ22f")
TELEMETRY_MAGIC = b"SHT1"
TELEMETRY_VERSION = 1
TELEMETRY_KINDS = {1: "desired", 2: "applied", 3: "state"}
TELEMETRY_SIDES = {0: "right", 1: "left"}


class NativeCaptureError(RuntimeError):
    """A fetched native capture is incomplete, corrupt, or inconsistent."""


@dataclass(frozen=True)
class CaptureChunk:
    index: int
    name: str
    path: Path
    records: int
    stream_counts: Mapping[str, int]
    sha256: str


@dataclass(frozen=True)
class TactileRecord:
    global_record_index: int
    event_index: int
    chunk_index: int
    chunk_name: str
    record_index: int
    record_flags: int
    recorder_receive_monotonic_ns: int
    recorder_receive_realtime_ns: int
    publisher_receive_monotonic_ns: int
    publisher_receive_realtime_ns: int
    source_frame_id: int
    source_timestamp: float
    publisher_sequence: int
    publisher_session_id: str
    channel: int
    hand_side: str
    f6: np.ndarray
    contact_point: tuple[float, ...]
    contact_point_shape: tuple[int, ...]
    deform_height: int
    deform_width: int
    deform_byte_length: int
    stream_schema_version: int
    stream_protocol: str
    tactile_config_sha256: str
    tactile_config_path: str
    rate_hz: float
    publisher_dropped_total: int
    publisher_queue_depth: int


@dataclass(frozen=True)
class HandTelemetryRecord:
    global_record_index: int
    telemetry_index: int
    chunk_index: int
    chunk_name: str
    record_index: int
    record_flags: int
    recorder_receive_monotonic_ns: int
    recorder_receive_realtime_ns: int
    kind: int
    kind_name: str
    side: int
    side_name: str
    complete: bool
    sequence: int
    source_monotonic_ns: int
    source_realtime_ns: int
    status: int
    source_desired_sequence: int
    values: np.ndarray


@dataclass(frozen=True)
class CanonicalChannelArrays:
    canonical_index: int
    canonical_name: str
    raw_channel: int
    event_index: np.ndarray
    recorder_receive_monotonic_ns: np.ndarray
    recorder_receive_realtime_ns: np.ndarray
    publisher_receive_monotonic_ns: np.ndarray
    publisher_receive_realtime_ns: np.ndarray
    source_frame_id: np.ndarray
    publisher_sequence: np.ndarray
    f6: np.ndarray


@dataclass(frozen=True)
class NativeCapture:
    episode_dir: Path
    capture_dir: Path
    capture_id: str
    workstation_metadata: Mapping[str, Any]
    manifest: Mapping[str, Any]
    chunks: tuple[CaptureChunk, ...]
    tactile_records: tuple[TactileRecord, ...]
    hand_telemetry_records: tuple[HandTelemetryRecord, ...]

    def canonical_channel_arrays(self) -> tuple[CanonicalChannelArrays, ...]:
        return canonical_channel_arrays(self.tactile_records)


@dataclass(frozen=True)
class _CaptureTools:
    capture_format: ModuleType
    protobuf: ModuleType


_TOOLS_CACHE: dict[Path, _CaptureTools] = {}


def _load_module(path: Path, module_name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise NativeCaptureError(f"could not load Python module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(module_name, None)
        raise
    return module


def load_capture_tools(capture_tools_dir: Path | str) -> _CaptureTools:
    """Dynamically load the authoritative chunk reader and protobuf schema."""

    tools_dir = Path(capture_tools_dir).expanduser().resolve(strict=True)
    cached = _TOOLS_CACHE.get(tools_dir)
    if cached is not None:
        return cached
    if not tools_dir.is_dir():
        raise NativeCaptureError(f"capture tools path is not a directory: {tools_dir}")
    format_path = tools_dir / "sharpa_capture_format.py"
    protobuf_path = tools_dir / "sharpa_pb2.py"
    for path in (format_path, protobuf_path):
        if not path.is_file():
            raise NativeCaptureError(f"required capture tool is missing: {path}")

    token = hashlib.sha256(str(tools_dir).encode("utf-8")).hexdigest()[:12]
    try:
        capture_format = _load_module(format_path, f"_sharpa_capture_format_{token}")
        protobuf = _load_module(protobuf_path, f"_sharpa_pb2_{token}")
    except Exception as exc:
        raise NativeCaptureError(f"could not load capture tools from {tools_dir}: {exc}") from exc
    tools = _CaptureTools(capture_format=capture_format, protobuf=protobuf)
    _TOOLS_CACHE[tools_dir] = tools
    return tools


def _load_json_object(path: Path, label: str) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise NativeCaptureError(f"{label} is missing or unsafe: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise NativeCaptureError(f"could not read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise NativeCaptureError(f"{label} must contain a JSON object")
    return value


def _canonical_capture_id(value: Any, label: str) -> str:
    if not isinstance(value, str):
        raise NativeCaptureError(f"{label} must be a canonical UUID")
    try:
        canonical = str(uuid.UUID(value))
    except (AttributeError, ValueError) as exc:
        raise NativeCaptureError(f"{label} must be a canonical UUID") from exc
    if canonical != value:
        raise NativeCaptureError(f"{label} must use canonical lowercase UUID form")
    return canonical


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _safe_chunk_name(value: Any, position: int) -> str:
    if not isinstance(value, str):
        raise NativeCaptureError(f"chunk {position} path must be a string")
    path = PurePosixPath(value)
    if path.is_absolute() or len(path.parts) != 1 or path.name != value or path.suffix != ".shc":
        raise NativeCaptureError(f"chunk {position} path is not a safe .shc basename: {value!r}")
    return value


def _nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise NativeCaptureError(f"{label} must be a non-negative integer")
    return value


def _normalized_stream_counts(value: Any, label: str) -> dict[str, int]:
    if not isinstance(value, Mapping):
        raise NativeCaptureError(f"{label} must be an object")
    result: dict[str, int] = {}
    for raw_key, raw_count in value.items():
        try:
            key = str(int(raw_key))
        except (TypeError, ValueError) as exc:
            raise NativeCaptureError(f"{label} has an invalid stream key: {raw_key!r}") from exc
        result[key] = _nonnegative_int(raw_count, f"{label}[{key}]")
    return result


def _validate_workstation_metadata(metadata: Mapping[str, Any]) -> str:
    if metadata.get("protocol") != CAPTURE_PROTOCOL:
        raise NativeCaptureError(f"capture metadata protocol is not {CAPTURE_PROTOCOL}")
    capture_id = _canonical_capture_id(metadata.get("capture_id"), "capture metadata capture_id")
    reasons = metadata.get("degraded_reasons")
    if metadata.get("status") != "complete" or metadata.get("valid") is not True:
        raise NativeCaptureError(
            "workstation capture metadata is not complete and valid "
            f"(status={metadata.get('status')!r}, valid={metadata.get('valid')!r}, reasons={reasons!r})"
        )
    if not isinstance(reasons, list) or reasons:
        raise NativeCaptureError("workstation capture metadata has invalid or non-empty degraded_reasons")
    return capture_id


def _validate_manifest(manifest: Mapping[str, Any], capture_id: str) -> Sequence[Mapping[str, Any]]:
    if manifest.get("format") != MANIFEST_FORMAT or manifest.get("version") != 1:
        raise NativeCaptureError(f"manifest is not {MANIFEST_FORMAT} version 1")
    if manifest.get("capture_id") != capture_id:
        raise NativeCaptureError("manifest capture_id does not match workstation capture metadata")
    if manifest.get("state") != "finalized" or manifest.get("valid") is not True:
        raise NativeCaptureError("manifest is not finalized and valid")
    invalid_reasons = manifest.get("invalid_reasons")
    if not isinstance(invalid_reasons, list) or invalid_reasons:
        raise NativeCaptureError("manifest has invalid or non-empty invalid_reasons")
    chunks = manifest.get("chunks")
    if not isinstance(chunks, list) or not chunks:
        raise NativeCaptureError("manifest chunks must be a non-empty list")
    return chunks


def _verify_manifest_checksum(capture_dir: Path) -> None:
    checksum_path = capture_dir / "manifest.sha256"
    if checksum_path.is_symlink() or not checksum_path.is_file():
        raise NativeCaptureError(f"manifest.sha256 is missing or unsafe: {checksum_path}")
    fields = checksum_path.read_text(encoding="ascii").split()
    if not fields or len(fields[0]) != 64:
        raise NativeCaptureError("manifest.sha256 is malformed")
    actual = _sha256_file(capture_dir / "manifest.json")
    if actual != fields[0].lower():
        raise NativeCaptureError(f"manifest SHA-256 mismatch: expected {fields[0].lower()}, got {actual}")


def _verify_control_clock(capture_dir: Path, manifest: Mapping[str, Any]) -> None:
    control_clock = manifest.get("control_clock")
    if not isinstance(control_clock, Mapping):
        raise NativeCaptureError("manifest control_clock must be an object")
    if control_clock.get("closed") is not True or control_clock.get("writer_error") is not None:
        raise NativeCaptureError("manifest control clock is not cleanly finalized")
    expected_hash = control_clock.get("sha256")
    if not isinstance(expected_hash, str) or len(expected_hash) != 64:
        raise NativeCaptureError("manifest control clock SHA-256 is missing or malformed")
    clock_path = capture_dir / "clock_samples.jsonl"
    if clock_path.is_symlink() or not clock_path.is_file():
        raise NativeCaptureError(f"clock_samples.jsonl is missing or unsafe: {clock_path}")
    actual_hash = _sha256_file(clock_path)
    if actual_hash != expected_hash.lower():
        raise NativeCaptureError(
            f"clock_samples.jsonl SHA-256 mismatch: expected {expected_hash.lower()}, got {actual_hash}"
        )


def _decode_tactile(
    record: Any,
    protobuf: ModuleType,
    *,
    global_record_index: int,
    event_index: int,
    chunk_index: int,
    chunk_name: str,
    record_index: int,
) -> TactileRecord:
    msg = protobuf.Tactile()
    try:
        msg.ParseFromString(record.payload)
    except Exception as exc:
        raise NativeCaptureError(
            f"invalid tactile protobuf in {chunk_name} record {record_index}: {exc}"
        ) from exc
    if msg.record_type not in ("", "sample"):
        raise NativeCaptureError(
            f"unexpected tactile record_type in {chunk_name} record {record_index}: {msg.record_type!r}"
        )
    if (
        msg.stream_protocol != TACTILE_PROTOCOL
        or int(msg.stream_schema_version) < 1
        or not msg.publisher_session_id
    ):
        raise NativeCaptureError(f"tactile v1 metadata is missing in {chunk_name} record {record_index}")
    channel = int(msg.channel)
    if not 0 <= channel < 10:
        raise NativeCaptureError(f"tactile channel is out of range in {chunk_name}: {channel}")
    expected_side = "right" if channel < 5 else "left"
    if msg.hand_side != expected_side:
        raise NativeCaptureError(
            f"tactile channel/side mismatch in {chunk_name} record {record_index}: "
            f"channel={channel}, hand_side={msg.hand_side!r}"
        )
    f6 = np.asarray(
        (
            msg.force6d.force.x,
            msg.force6d.force.y,
            msg.force6d.force.z,
            msg.force6d.torque.x,
            msg.force6d.torque.y,
            msg.force6d.torque.z,
        ),
        dtype=np.float32,
    )
    return TactileRecord(
        global_record_index=global_record_index,
        event_index=event_index,
        chunk_index=chunk_index,
        chunk_name=chunk_name,
        record_index=record_index,
        record_flags=int(record.flags),
        recorder_receive_monotonic_ns=int(record.receive_monotonic_ns),
        recorder_receive_realtime_ns=int(record.receive_realtime_ns),
        publisher_receive_monotonic_ns=int(msg.receive_monotonic_ns),
        publisher_receive_realtime_ns=int(msg.receive_realtime_ns),
        source_frame_id=int(msg.source_frame_id),
        source_timestamp=float(msg.source_timestamp),
        publisher_sequence=int(msg.publisher_sequence),
        publisher_session_id=str(msg.publisher_session_id),
        channel=channel,
        hand_side=str(msg.hand_side),
        f6=f6,
        contact_point=tuple(float(value) for value in msg.contact_point),
        contact_point_shape=tuple(int(value) for value in msg.contact_point_shape),
        deform_height=int(msg.deform.height),
        deform_width=int(msg.deform.width),
        deform_byte_length=len(msg.deform.data),
        stream_schema_version=int(msg.stream_schema_version),
        stream_protocol=str(msg.stream_protocol),
        tactile_config_sha256=str(msg.tactile_config_sha256),
        tactile_config_path=str(msg.tactile_config_path),
        rate_hz=float(msg.rate),
        publisher_dropped_total=int(msg.publisher_dropped_total),
        publisher_queue_depth=int(msg.publisher_queue_depth),
    )


def _decode_telemetry(
    record: Any,
    *,
    global_record_index: int,
    telemetry_index: int,
    chunk_index: int,
    chunk_name: str,
    record_index: int,
) -> HandTelemetryRecord:
    if len(record.payload) != TELEMETRY_STRUCT.size:
        raise NativeCaptureError(
            f"telemetry size {len(record.payload)} != {TELEMETRY_STRUCT.size} "
            f"in {chunk_name} record {record_index}"
        )
    values = TELEMETRY_STRUCT.unpack(record.payload)
    magic, version, kind, side, flags = values[:5]
    if magic != TELEMETRY_MAGIC or version != TELEMETRY_VERSION:
        raise NativeCaptureError(f"bad telemetry magic/version in {chunk_name} record {record_index}")
    if kind not in TELEMETRY_KINDS or side not in TELEMETRY_SIDES:
        raise NativeCaptureError(f"bad telemetry kind/side in {chunk_name} record {record_index}")
    return HandTelemetryRecord(
        global_record_index=global_record_index,
        telemetry_index=telemetry_index,
        chunk_index=chunk_index,
        chunk_name=chunk_name,
        record_index=record_index,
        record_flags=int(record.flags),
        recorder_receive_monotonic_ns=int(record.receive_monotonic_ns),
        recorder_receive_realtime_ns=int(record.receive_realtime_ns),
        kind=int(kind),
        kind_name=TELEMETRY_KINDS[int(kind)],
        side=int(side),
        side_name=TELEMETRY_SIDES[int(side)],
        complete=bool(flags & 0x01),
        sequence=int(values[5]),
        source_monotonic_ns=int(values[6]),
        source_realtime_ns=int(values[7]),
        status=int(values[8]),
        source_desired_sequence=int(values[9]),
        values=np.asarray(values[10:], dtype=np.float32),
    )


def load_native_capture(
    episode_dir: Path | str,
    capture_tools_dir: Path | str,
) -> NativeCapture:
    """Validate and decode one episode's already-fetched native capture."""

    episode = Path(episode_dir).expanduser().resolve(strict=True)
    if not episode.is_dir():
        raise NativeCaptureError(f"episode path is not a directory: {episode}")
    metadata = _load_json_object(episode / CAPTURE_METADATA_FILENAME, CAPTURE_METADATA_FILENAME)
    capture_id = _validate_workstation_metadata(metadata)

    capture_dir = episode / FETCHED_CAPTURE_DIRNAME
    if capture_dir.is_symlink() or not capture_dir.is_dir():
        raise NativeCaptureError(f"fetched capture directory is missing or unsafe: {capture_dir}")
    manifest = _load_json_object(capture_dir / "manifest.json", "manifest.json")
    declared_chunks = _validate_manifest(manifest, capture_id)
    _verify_manifest_checksum(capture_dir)
    _verify_control_clock(capture_dir, manifest)
    tools = load_capture_tools(capture_tools_dir)

    tactile_records: list[TactileRecord] = []
    telemetry_records: list[HandTelemetryRecord] = []
    verified_chunks: list[CaptureChunk] = []
    total_stream_counts: Counter[str] = Counter()
    seen_names: set[str] = set()
    seen_indices: set[int] = set()
    global_record_index = 0

    for position, declaration in enumerate(declared_chunks):
        if not isinstance(declaration, Mapping):
            raise NativeCaptureError(f"chunk entry {position} must be an object")
        name = _safe_chunk_name(declaration.get("path"), position)
        if name in seen_names:
            raise NativeCaptureError(f"duplicate chunk path in manifest: {name}")
        seen_names.add(name)
        chunk_index = _nonnegative_int(declaration.get("index"), f"chunk {position} index")
        if chunk_index in seen_indices:
            raise NativeCaptureError(f"duplicate chunk index in manifest: {chunk_index}")
        seen_indices.add(chunk_index)
        expected_records = _nonnegative_int(declaration.get("records"), f"chunk {chunk_index} records")
        expected_stream_counts = _normalized_stream_counts(
            declaration.get("stream_counts"), f"chunk {chunk_index} stream_counts"
        )
        expected_sha256 = declaration.get("sha256")
        if not isinstance(expected_sha256, str) or len(expected_sha256) != 64:
            raise NativeCaptureError(f"chunk {chunk_index} SHA-256 is missing or malformed")
        path = capture_dir / name
        if path.is_symlink() or not path.is_file():
            raise NativeCaptureError(f"manifest chunk is missing or unsafe: {path}")
        actual_sha256 = _sha256_file(path)
        if actual_sha256 != expected_sha256.lower():
            raise NativeCaptureError(
                f"chunk SHA-256 mismatch for {name}: expected {expected_sha256.lower()}, got {actual_sha256}"
            )
        try:
            chunk_metadata, records = tools.capture_format.read_chunk(path, allow_partial=False)
        except Exception as exc:
            raise NativeCaptureError(f"could not read finalized chunk {name}: {exc}") from exc
        if int(chunk_metadata.get("chunk_index", -1)) != chunk_index:
            raise NativeCaptureError(f"chunk header index does not match manifest for {name}")
        if len(records) != expected_records:
            raise NativeCaptureError(
                f"chunk record count mismatch for {name}: expected {expected_records}, got {len(records)}"
            )
        actual_stream_counts = Counter(str(int(record.stream)) for record in records)
        if dict(actual_stream_counts) != expected_stream_counts:
            raise NativeCaptureError(
                f"chunk stream counts mismatch for {name}: "
                f"expected {expected_stream_counts}, got {dict(actual_stream_counts)}"
            )

        for record_index, record in enumerate(records):
            stream = int(record.stream)
            if stream == int(tools.capture_format.STREAM_TACTILE):
                tactile_records.append(
                    _decode_tactile(
                        record,
                        tools.protobuf,
                        global_record_index=global_record_index,
                        event_index=len(tactile_records),
                        chunk_index=chunk_index,
                        chunk_name=name,
                        record_index=record_index,
                    )
                )
            elif stream == int(tools.capture_format.STREAM_HAND_TELEMETRY):
                telemetry_records.append(
                    _decode_telemetry(
                        record,
                        global_record_index=global_record_index,
                        telemetry_index=len(telemetry_records),
                        chunk_index=chunk_index,
                        chunk_name=name,
                        record_index=record_index,
                    )
                )
            global_record_index += 1
        total_stream_counts.update(actual_stream_counts)
        verified_chunks.append(
            CaptureChunk(
                index=chunk_index,
                name=name,
                path=path,
                records=len(records),
                stream_counts=dict(actual_stream_counts),
                sha256=actual_sha256,
            )
        )

    if not tactile_records:
        raise NativeCaptureError("native capture contains no tactile records")
    if not telemetry_records:
        raise NativeCaptureError("native capture contains no hand telemetry records")
    manifest_counts = manifest.get("counts")
    if isinstance(manifest_counts, Mapping):
        if "written" in manifest_counts and int(manifest_counts["written"]) != global_record_index:
            raise NativeCaptureError("manifest counts.written does not match decoded record count")
        for stream, count in total_stream_counts.items():
            key = f"written_stream_{stream}"
            if key in manifest_counts and int(manifest_counts[key]) != count:
                raise NativeCaptureError(f"manifest {key} does not match decoded stream count")

    return NativeCapture(
        episode_dir=episode,
        capture_dir=capture_dir,
        capture_id=capture_id,
        workstation_metadata=metadata,
        manifest=manifest,
        chunks=tuple(verified_chunks),
        tactile_records=tuple(tactile_records),
        hand_telemetry_records=tuple(telemetry_records),
    )


def canonical_channel_arrays(
    records: Iterable[TactileRecord],
) -> tuple[CanonicalChannelArrays, ...]:
    """Return stable, recorder-time-sorted arrays in canonical finger order."""

    grouped: dict[int, list[TactileRecord]] = {channel: [] for channel in range(10)}
    for record in records:
        if record.channel not in grouped:
            raise NativeCaptureError(f"tactile channel is out of range: {record.channel}")
        grouped[record.channel].append(record)

    result: list[CanonicalChannelArrays] = []
    for canonical_index, (name, raw_channel) in enumerate(
        zip(CANONICAL_FINGER_NAMES, RAW_CHANNEL_FOR_CANONICAL)
    ):
        channel_records = sorted(
            grouped[raw_channel],
            key=lambda item: (item.recorder_receive_monotonic_ns, item.event_index),
        )
        if not channel_records:
            raise NativeCaptureError(f"native capture has no events for {name} (raw channel {raw_channel})")

        def int_array(attribute: str) -> np.ndarray:
            return np.asarray([getattr(item, attribute) for item in channel_records], dtype=np.int64)

        result.append(
            CanonicalChannelArrays(
                canonical_index=canonical_index,
                canonical_name=name,
                raw_channel=raw_channel,
                event_index=int_array("event_index"),
                recorder_receive_monotonic_ns=int_array("recorder_receive_monotonic_ns"),
                recorder_receive_realtime_ns=int_array("recorder_receive_realtime_ns"),
                publisher_receive_monotonic_ns=int_array("publisher_receive_monotonic_ns"),
                publisher_receive_realtime_ns=int_array("publisher_receive_realtime_ns"),
                source_frame_id=int_array("source_frame_id"),
                publisher_sequence=int_array("publisher_sequence"),
                f6=np.stack([item.f6 for item in channel_records]).astype(np.float32, copy=False),
            )
        )
    return tuple(result)
