#!/usr/bin/env python3
"""Convert H2 + Sharpa recordings with native tactile into LeRobot v2.1.

This is intentionally separate from the generic Unitree JSON converter.  It
preserves the established 30 Hz observation/action bundle and adds a causal
six-sample (nominally 180 Hz) tactile-force history at every 30 Hz policy row.
The same pure window builder is importable by deployment code.
"""

from __future__ import annotations

import argparse
import copy
import dataclasses
import json
import os
import shutil
import sys
import uuid
from pathlib import Path
from typing import Literal, Sequence

import cv2
import numpy as np
import pandas as pd

from unitree_lerobot.utils.constants import ROBOT_CONFIGS
from unitree_lerobot.utils.h2_sharpa_180_alignment import (
    ClockFit,
    build_causal_tactile_window,
    fit_thor_to_workstation_clock,
    target_offsets_ns,
)
from unitree_lerobot.utils.h2_sharpa_native_capture import (
    NativeCapture,
    NativeCaptureError,
    load_native_capture,
)


POLICY_FPS = 30
TACTILE_RATE_HZ = 180.0
TACTILE_WINDOW_SIZE = 6
ROBOT_TYPE = "Unitree_H2_Sharpa_Tactile_Mono"
ALIGNMENT_SCHEMA = "h2_sharpa_180_alignment.v1"

ROBOT_CONFIG = ROBOT_CONFIGS[ROBOT_TYPE]
MOTOR_NAMES = tuple(ROBOT_CONFIG.motors)
TACTILE_KEYS = tuple(ROBOT_CONFIG.tactile_keys)

CAMERA_LAYOUT = (
    ("color_0", "head_left", "observation.images.cam_left_high"),
    ("color_1", "wrist_left", "observation.images.cam_left_wrist"),
    ("color_2", "wrist_right", "observation.images.cam_right_wrist"),
)

CANONICAL_FINGERS = (
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

FORCE_NAMES = tuple(f"{finger}_f{axis}" for finger in CANONICAL_FINGERS for axis in range(6))
WINDOW_NAMES = tuple(f"t-{TACTILE_WINDOW_SIZE - 1 - index}" for index in range(TACTILE_WINDOW_SIZE))


class ConversionError(RuntimeError):
    """Raised when source evidence is missing or inconsistent."""


@dataclasses.dataclass(frozen=True)
class ConverterConfig:
    raw_dir: Path
    output_dir: Path
    repo_id: str
    task_description: str | None = None
    capture_tools_dir: Path | None = None
    native_storage_mode: Literal["copy", "hardlink"] = "copy"
    tactile_model_profile: Literal["bimanual", "left", "right"] = "bimanual"
    max_tactile_hold_ms: float = 50.0
    image_writer_processes: int = 0
    image_writer_threads: int = 4


@dataclasses.dataclass(frozen=True)
class EpisodeAlignment:
    capture_id: str
    clock_fit: ClockFit
    availability_ns_by_channel: tuple[np.ndarray, ...]
    event_index_by_channel: tuple[np.ndarray, ...]
    force_by_channel: tuple[np.ndarray, ...]


def _load_json(path: Path) -> dict:
    try:
        with path.open("r", encoding="utf-8") as handle:
            value = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise ConversionError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ConversionError(f"expected a JSON object in {path}")
    return value


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _require_vector(value: object, size: int, label: str, dtype: str = "float32") -> np.ndarray:
    array = np.asarray(value, dtype=dtype)
    if array.shape != (size,):
        raise ConversionError(f"{label} must have shape ({size},), found {array.shape}")
    if np.issubdtype(array.dtype, np.floating) and not np.all(np.isfinite(array)):
        raise ConversionError(f"{label} contains non-finite values")
    return array


def _require_int(value: object, label: str) -> int:
    if isinstance(value, bool):
        raise ConversionError(f"{label} must be an integer, found bool")
    try:
        result = int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ConversionError(f"{label} must be an integer") from exc
    return result


def _nested(mapping: object, path: Sequence[str], label: str) -> object:
    value = mapping
    for key in path:
        if not isinstance(value, dict) or key not in value:
            raise ConversionError(f"missing {label}: {'.'.join(path)}")
        value = value[key]
    return value


def discover_episode_dirs(raw_dir: Path) -> list[Path]:
    raw_dir = raw_dir.expanduser().resolve()
    if not raw_dir.is_dir():
        raise ConversionError(f"raw directory does not exist: {raw_dir}")
    episodes = sorted(
        path
        for path in raw_dir.glob("episode_*")
        if path.is_dir() and (path / "data.json").is_file()
    )
    if not episodes:
        raise ConversionError(f"no episode_*/data.json directories found directly under {raw_dir}")
    return episodes


def resolve_capture_tools_dir(explicit: Path | None) -> Path:
    candidates: list[Path] = []
    if explicit is not None:
        candidates.append(explicit.expanduser())
    environment = os.environ.get("SHARPA_CAPTURE_TOOLS_DIR")
    if environment:
        candidates.append(Path(environment).expanduser())
    workspace = Path(__file__).resolve().parents[3]
    candidates.append(workspace / "sharpa-teleop" / "thor" / "docker_5_0_1")

    for candidate in candidates:
        resolved = candidate.resolve()
        if (resolved / "sharpa_capture_format.py").is_file() and (resolved / "sharpa_pb2.py").is_file():
            return resolved
    rendered = ", ".join(str(candidate) for candidate in candidates)
    raise ConversionError(
        "could not find Sharpa capture format tools (sharpa_capture_format.py and "
        f"sharpa_pb2.py); checked: {rendered}"
    )


def load_source_episode(episode_dir: Path) -> tuple[dict, list[dict]]:
    source = _load_json(episode_dir / "data.json")
    rows = source.get("data")
    if not isinstance(rows, list) or len(rows) <= 1:
        raise ConversionError(f"{episode_dir}: need source row 0 plus at least one retained row")
    if not isinstance(rows[0], dict) or _require_int(rows[0].get("idx"), "source row 0 idx") != 0:
        raise ConversionError(f"{episode_dir}: the dropped first row is not source idx 0")
    for position, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ConversionError(f"{episode_dir}: source row {position} is not an object")
        source_index = _require_int(row.get("idx"), f"{episode_dir.name} row {position} idx")
        if source_index != position:
            raise ConversionError(
                f"{episode_dir}: source idx sequence is not contiguous at position {position}: {source_index}"
            )
    return source, rows


def resolve_task_text(source: dict, override: str | None, episode_dir: Path) -> tuple[str, dict]:
    raw_text = source.get("text") if isinstance(source.get("text"), dict) else {}
    if override is not None:
        task = override.strip()
        source_name = "command_line_override"
    else:
        task = str(raw_text.get("goal") or "").strip()
        source_name = "data.json.text.goal"
    if not task:
        raise ConversionError(
            f"{episode_dir}: task text is empty; provide --task-description or populate data.json text.goal"
        )
    provenance = {
        "selected": task,
        "selected_from": source_name,
        "raw_goal": raw_text.get("goal"),
        "raw_description": raw_text.get("desc"),
        "raw_steps": raw_text.get("steps"),
    }
    return task, provenance


def prepare_episode_alignment(capture: NativeCapture, max_hold_ms: float) -> EpisodeAlignment:
    if not np.isfinite(max_hold_ms) or max_hold_ms <= 0.0:
        raise ConversionError("max_tactile_hold_ms must be finite and positive")
    clock_samples = capture.workstation_metadata.get("clock_samples")
    if not isinstance(clock_samples, list):
        raise ConversionError(f"capture {capture.capture_id}: workstation clock samples are missing")
    clock_fit = fit_thor_to_workstation_clock(clock_samples)
    channels = capture.canonical_channel_arrays()
    if len(channels) != len(CANONICAL_FINGERS):
        raise ConversionError(f"capture {capture.capture_id}: expected 10 canonical tactile channels")

    availability: list[np.ndarray] = []
    event_indices: list[np.ndarray] = []
    forces: list[np.ndarray] = []
    for canonical_index, channel in enumerate(channels):
        mapped = np.asarray(clock_fit.apply(channel.recorder_receive_monotonic_ns), dtype=np.int64)
        if mapped.ndim != 1 or mapped.size == 0:
            raise ConversionError(
                f"capture {capture.capture_id}: no native events for {CANONICAL_FINGERS[canonical_index]}"
            )
        if np.any(np.diff(mapped) < 0):
            raise ConversionError(
                f"capture {capture.capture_id}: mapped availability is not monotonic for "
                f"{CANONICAL_FINGERS[canonical_index]}"
            )
        availability.append(mapped)
        event_indices.append(np.asarray(channel.event_index, dtype=np.int64))
        forces.append(np.asarray(channel.f6, dtype=np.float32))

    return EpisodeAlignment(
        capture_id=capture.capture_id,
        clock_fit=clock_fit,
        availability_ns_by_channel=tuple(availability),
        event_index_by_channel=tuple(event_indices),
        force_by_channel=tuple(forces),
    )


def _parse_state_action(row: dict, label: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    states = row.get("states")
    actions = row.get("actions")
    if not isinstance(states, dict) or not isinstance(actions, dict):
        raise ConversionError(f"{label}: states/actions must be objects")

    state = np.concatenate(
        (
            _require_vector(_nested(states, ("left_arm", "qpos"), label), 7, f"{label} left arm state"),
            _require_vector(_nested(states, ("right_arm", "qpos"), label), 7, f"{label} right arm state"),
            _require_vector(_nested(states, ("left_ee", "qpos"), label), 22, f"{label} left hand state"),
            _require_vector(_nested(states, ("right_ee", "qpos"), label), 22, f"{label} right hand state"),
        )
    ).astype(np.float32, copy=False)

    # Match the historical H2 Sharpa converter: the hand portion of action is
    # the observed-pose feedback proxy in action.qpos (in the source recordings
    # it matches the following hand state), not the desired DDS command.
    action = np.concatenate(
        (
            _require_vector(_nested(actions, ("left_arm", "qpos"), label), 7, f"{label} left arm action"),
            _require_vector(_nested(actions, ("right_arm", "qpos"), label), 7, f"{label} right arm action"),
            _require_vector(
                _nested(actions, ("left_ee", "qpos"), label),
                22,
                f"{label} left hand observed-pose action",
            ),
            _require_vector(
                _nested(actions, ("right_ee", "qpos"), label),
                22,
                f"{label} right hand observed-pose action",
            ),
        )
    ).astype(np.float32, copy=False)
    proxy = np.concatenate(
        (
            _require_vector(
                _nested(actions, ("left_ee", "qpos"), label), 22, f"{label} left hand action proxy"
            ),
            _require_vector(
                _nested(actions, ("right_ee", "qpos"), label), 22, f"{label} right hand action proxy"
            ),
        )
    ).astype(np.float32, copy=False)
    waist = _require_vector(_nested(states, ("body", "qpos"), label), 3, f"{label} waist state")

    if state.shape != (len(MOTOR_NAMES),) or action.shape != (len(MOTOR_NAMES),):
        raise AssertionError("internal H2 Sharpa state/action layout mismatch")
    return state, action, proxy, waist


def _load_rgb(path: Path, expected_shape: tuple[int, int, int], label: str) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ConversionError(f"{label}: cannot read image {path}")
    if image.shape != expected_shape:
        raise ConversionError(f"{label}: image {path} has shape {image.shape}, expected {expected_shape}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def _load_tactile_rgb(path: Path, label: str) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ConversionError(f"{label}: cannot read tactile image {path}")
    if image.shape != (240, 240):
        raise ConversionError(f"{label}: tactile image {path} has shape {image.shape}, expected (240, 240)")
    return np.repeat(image[:, :, None], 3, axis=2)


def _parse_legacy_tactile(episode_dir: Path, row: dict, label: str) -> tuple[dict[str, np.ndarray], np.ndarray]:
    tactile_root = row.get("tactiles")
    if not isinstance(tactile_root, dict):
        raise ConversionError(f"{label}: tactiles must be an object")
    images: dict[str, np.ndarray] = {}
    force_values: list[np.ndarray] = []
    for feature_key, hand_key, finger_name in TACTILE_KEYS:
        entry = _nested(tactile_root, (hand_key, finger_name), label)
        if not isinstance(entry, dict):
            raise ConversionError(f"{label}: tactile {hand_key}.{finger_name} is not an object")
        relative_path = entry.get("deform")
        if not isinstance(relative_path, str) or not relative_path:
            raise ConversionError(f"{label}: tactile {hand_key}.{finger_name} has no deform image")
        images[feature_key] = _load_tactile_rgb(
            episode_dir / relative_path,
            f"{label} tactile {hand_key}.{finger_name}",
        )
        # Missing force is an error.  This converter never manufactures a
        # zero tactile observation.
        force_values.append(
            _require_vector(entry.get("f6"), 6, f"{label} tactile {hand_key}.{finger_name} f6")
        )
    return images, np.concatenate(force_values).astype(np.float32, copy=False)


def _clock_pair(mapping: object, path: Sequence[str], label: str) -> tuple[int, int]:
    value = _nested(mapping, path, label)
    if not isinstance(value, dict):
        raise ConversionError(f"{label}: {'.'.join(path)} is not an object")
    return (
        _require_int(value.get("workstation_receive_monotonic_ns"), f"{label} receive monotonic"),
        _require_int(value.get("workstation_receive_realtime_ns"), f"{label} receive realtime"),
    )


def _parse_timing_features(
    row: dict,
    label: str,
    *,
    expected_capture_id: str,
) -> dict[str, np.ndarray]:
    timestamps = row.get("timestamps")
    native = row.get("native_capture")
    if not isinstance(timestamps, dict) or not isinstance(native, dict):
        raise ConversionError(f"{label}: timestamps/native_capture must be objects")

    source_index = _require_int(row.get("idx"), f"{label} source idx")
    if native.get("protocol") != "sharpa.capture.v1":
        raise ConversionError(f"{label}: native capture protocol is not sharpa.capture.v1")
    if native.get("capture_id") != expected_capture_id:
        raise ConversionError(
            f"{label}: row capture_id {native.get('capture_id')!r} does not match {expected_capture_id!r}"
        )
    native_index = _require_int(native.get("idx"), f"{label} native capture idx")
    if native_index != source_index:
        raise ConversionError(
            f"{label}: native capture idx {native_index} does not match source idx {source_index}"
        )
    workstation_mono = _require_int(timestamps.get("workstation_monotonic_ns"), f"{label} workstation mono")
    workstation_real = _require_int(timestamps.get("workstation_realtime_ns"), f"{label} workstation real")

    camera_rows: list[list[int]] = []
    for _color_key, source_name, _feature_key in CAMERA_LAYOUT:
        camera = _nested(timestamps, ("cameras", source_name), label)
        if not isinstance(camera, dict):
            raise ConversionError(f"{label}: camera clock {source_name} is not an object")
        camera_rows.append(
            [
                _require_int(camera.get("ros_header_stamp_ns"), f"{label} {source_name} ROS stamp"),
                _require_int(
                    camera.get("workstation_receive_monotonic_ns"), f"{label} {source_name} receive mono"
                ),
                _require_int(
                    camera.get("workstation_receive_realtime_ns"), f"{label} {source_name} receive real"
                ),
            ]
        )

    lowstate = _nested(timestamps, ("h2_lowstate",), label)
    arm_target = _nested(timestamps, ("h2_arm_command", "target"), label)
    arm_publish = _nested(timestamps, ("h2_arm_command", "last_publish"), label)
    if not all(isinstance(value, dict) for value in (lowstate, arm_target, arm_publish)):
        raise ConversionError(f"{label}: malformed H2 timing metadata")

    hand_clock_groups: dict[str, np.ndarray] = {}
    for output_key, source_key in (
        ("recording.sharpa_hand_state_clock_ns", "sharpa_hand_state"),
        ("recording.sharpa_hand_action_proxy_clock_ns", "sharpa_hand_action_proxy"),
        ("recording.sharpa_hand_desired_command_clock_ns", "sharpa_hand_desired_command"),
    ):
        pairs = [_clock_pair(timestamps, (source_key, side), label) for side in ("left", "right")]
        hand_clock_groups[output_key] = np.asarray(pairs, dtype=np.int64)

    tactile_clock: list[list[int]] = []
    tactile_root = row.get("tactiles")
    for _feature_key, hand_key, finger_name in TACTILE_KEYS:
        entry = _nested(tactile_root, (hand_key, finger_name), label)
        if not isinstance(entry, dict):
            raise ConversionError(f"{label}: malformed legacy tactile metadata")
        tactile_clock.append(
            [
                _require_int(entry.get("frame_id"), f"{label} {hand_key}.{finger_name} frame id"),
                int(round(float(entry.get("ts")) * 1_000_000_000.0)),
                _require_int(
                    entry.get("workstation_receive_monotonic_ns"),
                    f"{label} {hand_key}.{finger_name} receive mono",
                ),
                _require_int(
                    entry.get("workstation_receive_realtime_ns"),
                    f"{label} {hand_key}.{finger_name} receive real",
                ),
            ]
        )

    result = {
        "recording.frame_clock_ns": np.asarray(
            [source_index, workstation_mono, workstation_real], dtype=np.int64
        ),
        "recording.camera_clock_ns": np.asarray(camera_rows, dtype=np.int64),
        "recording.h2_lowstate_clock": np.asarray(
            [
                _require_int(lowstate.get("workstation_receive_monotonic_ns"), f"{label} H2 receive mono"),
                _require_int(lowstate.get("workstation_receive_realtime_ns"), f"{label} H2 receive real"),
                _require_int(lowstate.get("unitree_tick"), f"{label} H2 Unitree tick"),
            ],
            dtype=np.int64,
        ),
        "recording.h2_arm_command_clock": np.asarray(
            [
                _require_int(arm_target.get("target_generation"), f"{label} target generation"),
                _require_int(
                    arm_target.get("workstation_target_set_monotonic_ns"), f"{label} target set mono"
                ),
                _require_int(
                    arm_target.get("workstation_target_set_realtime_ns"), f"{label} target set real"
                ),
                _require_int(arm_publish.get("target_generation"), f"{label} published generation"),
                _require_int(
                    arm_publish.get("workstation_target_set_monotonic_ns"), f"{label} published target mono"
                ),
                _require_int(
                    arm_publish.get("workstation_target_set_realtime_ns"), f"{label} published target real"
                ),
                _require_int(
                    arm_publish.get("workstation_publish_monotonic_ns"), f"{label} publish mono"
                ),
                _require_int(
                    arm_publish.get("workstation_publish_realtime_ns"), f"{label} publish real"
                ),
            ],
            dtype=np.int64,
        ),
        "recording.legacy_tactile_clock": np.asarray(tactile_clock, dtype=np.int64),
        "recording.native_capture_clock_ns": np.asarray(
            [
                native_index,
                _require_int(native.get("workstation_monotonic_ns"), f"{label} native request mono"),
                _require_int(native.get("workstation_realtime_ns"), f"{label} native request real"),
            ],
            dtype=np.int64,
        ),
    }
    result.update(hand_clock_groups)
    return result


def _interval_metadata(
    previous_tick_ns: int,
    policy_tick_ns: int,
    alignment: EpisodeAlignment,
) -> tuple[np.ndarray, np.ndarray]:
    event_bounds = np.full((len(CANONICAL_FINGERS), 2), -1, dtype=np.int64)
    event_count = np.zeros((len(CANONICAL_FINGERS),), dtype=np.int64)
    for channel, (availability, event_indices) in enumerate(
        zip(alignment.availability_ns_by_channel, alignment.event_index_by_channel)
    ):
        start = int(np.searchsorted(availability, previous_tick_ns, side="right"))
        end = int(np.searchsorted(availability, policy_tick_ns, side="right"))
        count = max(0, end - start)
        event_count[channel] = count
        if count:
            event_bounds[channel, 0] = int(event_indices[start])
            event_bounds[channel, 1] = int(event_indices[end - 1])
    return event_bounds, event_count


def build_frame(
    episode_dir: Path,
    row: dict,
    previous_row: dict,
    alignment: EpisodeAlignment,
    max_hold_ns: int,
) -> tuple[dict[str, np.ndarray], dict]:
    source_index = _require_int(row.get("idx"), f"{episode_dir.name} source idx")
    label = f"{episode_dir.name} source row {source_index}"
    state, action, hand_action_proxy, waist = _parse_state_action(row, label)

    colors = row.get("colors")
    if not isinstance(colors, dict):
        raise ConversionError(f"{label}: colors must be an object")
    frame: dict[str, np.ndarray] = {
        "observation.state": state,
        "action": action,
        "observation.waist": waist,
        "recording.sharpa_hand_action_proxy": hand_action_proxy,
    }
    for color_key, _source_name, feature_key in CAMERA_LAYOUT:
        relative_path = colors.get(color_key)
        if not isinstance(relative_path, str) or not relative_path:
            raise ConversionError(f"{label}: missing camera image {color_key}")
        frame[feature_key] = _load_rgb(
            episode_dir / relative_path,
            (480, 640, 3),
            f"{label} camera {color_key}",
        )

    tactile_images, tactile_force = _parse_legacy_tactile(episode_dir, row, label)
    frame.update(tactile_images)
    frame["observation.tactile.force"] = tactile_force
    frame.update(
        _parse_timing_features(row, label, expected_capture_id=alignment.capture_id)
    )

    timestamps = row["timestamps"]
    previous_timestamps = previous_row.get("timestamps")
    if not isinstance(previous_timestamps, dict):
        raise ConversionError(f"{label}: previous row timestamps are missing")
    policy_tick_ns = _require_int(timestamps.get("workstation_monotonic_ns"), f"{label} policy tick")
    previous_tick_ns = _require_int(
        previous_timestamps.get("workstation_monotonic_ns"), f"{label} previous policy tick"
    )
    if previous_tick_ns > policy_tick_ns:
        raise ConversionError(f"{label}: policy timestamps are non-monotonic")

    window = build_causal_tactile_window(
        policy_tick_ns,
        alignment.availability_ns_by_channel,
        alignment.event_index_by_channel,
        alignment.force_by_channel,
        offsets_ns=target_offsets_ns(TACTILE_RATE_HZ, TACTILE_WINDOW_SIZE),
        max_hold_ns=max_hold_ns,
    )
    event_bounds, event_count = _interval_metadata(previous_tick_ns, policy_tick_ns, alignment)
    frame.update(
        {
            "observation.tactile_180.force": window.force,
            "observation.tactile_180.source_event_index": window.source_event_index,
            "observation.tactile_180.sample_age_ns": window.sample_age_ns,
            "observation.tactile_180.grid_hold_age_ns": window.grid_hold_age_ns,
            "observation.tactile_180.repeat_mask": window.repeat_mask,
            "observation.tactile_180.prefill_mask": window.prefill_mask,
            "recording.native_tactile_interval_event_index": event_bounds,
            "recording.native_tactile_interval_count": event_count,
        }
    )
    alignment_row = {
        "source_frame_index": source_index,
        "policy_tick_workstation_monotonic_ns": policy_tick_ns,
        "target_grid_workstation_monotonic_ns": window.target_grid_ns.tolist(),
        "source_event_index": window.source_event_index.reshape(-1).tolist(),
        "selected_availability_workstation_monotonic_ns": window.selected_availability_ns.reshape(-1).tolist(),
        "sample_age_ns": window.sample_age_ns.reshape(-1).tolist(),
        "grid_hold_age_ns": window.grid_hold_age_ns.reshape(-1).tolist(),
        "repeat_mask": window.repeat_mask.reshape(-1).tolist(),
        "prefill_mask": window.prefill_mask.reshape(-1).tolist(),
        "interval_first_last_event_index": event_bounds.reshape(-1).tolist(),
        "interval_event_count": event_count.tolist(),
    }
    return frame, alignment_row


def create_features() -> dict[str, dict]:
    features: dict[str, dict] = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(MOTOR_NAMES),),
            "names": [list(MOTOR_NAMES)],
        },
        "action": {
            "dtype": "float32",
            "shape": (len(MOTOR_NAMES),),
            "names": [list(MOTOR_NAMES)],
        },
        "observation.waist": {
            "dtype": "float32",
            "shape": (3,),
            "names": [["kWaistYaw", "kWaistRoll", "kWaistPitch"]],
        },
        "recording.sharpa_hand_action_proxy": {
            "dtype": "float32",
            "shape": (44,),
            "names": [list(MOTOR_NAMES[14:])],
        },
        "observation.tactile.force": {
            "dtype": "float32",
            "shape": (60,),
            "names": [list(FORCE_NAMES)],
        },
        "observation.tactile_180.force": {
            "dtype": "float32",
            "shape": (TACTILE_WINDOW_SIZE, 60),
            "names": [list(WINDOW_NAMES), list(FORCE_NAMES)],
        },
        "observation.tactile_180.source_event_index": {
            "dtype": "int64",
            "shape": (TACTILE_WINDOW_SIZE, 10),
            "names": [list(WINDOW_NAMES), list(CANONICAL_FINGERS)],
        },
        "observation.tactile_180.sample_age_ns": {
            "dtype": "int64",
            "shape": (TACTILE_WINDOW_SIZE, 10),
            "names": [list(WINDOW_NAMES), list(CANONICAL_FINGERS)],
        },
        "observation.tactile_180.grid_hold_age_ns": {
            "dtype": "int64",
            "shape": (TACTILE_WINDOW_SIZE, 10),
            "names": [list(WINDOW_NAMES), list(CANONICAL_FINGERS)],
        },
        "observation.tactile_180.repeat_mask": {
            "dtype": "uint8",
            "shape": (TACTILE_WINDOW_SIZE, 10),
            "names": [list(WINDOW_NAMES), list(CANONICAL_FINGERS)],
        },
        "observation.tactile_180.prefill_mask": {
            "dtype": "uint8",
            "shape": (TACTILE_WINDOW_SIZE, 10),
            "names": [list(WINDOW_NAMES), list(CANONICAL_FINGERS)],
        },
        "recording.frame_clock_ns": {
            "dtype": "int64",
            "shape": (3,),
            "names": [["source_frame_index", "workstation_monotonic_ns", "workstation_realtime_ns"]],
        },
        "recording.camera_clock_ns": {
            "dtype": "int64",
            "shape": (3, 3),
            "names": [
                ["head_left", "wrist_left", "wrist_right"],
                ["ros_header_stamp_ns", "workstation_receive_monotonic_ns", "workstation_receive_realtime_ns"],
            ],
        },
        "recording.h2_lowstate_clock": {
            "dtype": "int64",
            "shape": (3,),
            "names": [["workstation_receive_monotonic_ns", "workstation_receive_realtime_ns", "unitree_tick"]],
        },
        "recording.h2_arm_command_clock": {
            "dtype": "int64",
            "shape": (8,),
            "names": [[
                "target_generation",
                "target_set_monotonic_ns",
                "target_set_realtime_ns",
                "last_publish_target_generation",
                "last_publish_target_set_monotonic_ns",
                "last_publish_target_set_realtime_ns",
                "last_publish_monotonic_ns",
                "last_publish_realtime_ns",
            ]],
        },
        "recording.legacy_tactile_clock": {
            "dtype": "int64",
            "shape": (10, 4),
            "names": [
                list(CANONICAL_FINGERS),
                [
                    "source_frame_id",
                    "source_timestamp_ns",
                    "workstation_receive_monotonic_ns",
                    "workstation_receive_realtime_ns",
                ],
            ],
        },
        "recording.native_capture_clock_ns": {
            "dtype": "int64",
            "shape": (3,),
            "names": [["source_idx", "workstation_request_monotonic_ns", "workstation_request_realtime_ns"]],
        },
        "recording.native_tactile_interval_event_index": {
            "dtype": "int64",
            "shape": (10, 2),
            "names": [list(CANONICAL_FINGERS), ["first_event_index", "last_event_index"]],
        },
        "recording.native_tactile_interval_count": {
            "dtype": "int64",
            "shape": (10,),
            "names": [list(CANONICAL_FINGERS)],
        },
    }
    for key in (
        "recording.sharpa_hand_state_clock_ns",
        "recording.sharpa_hand_action_proxy_clock_ns",
        "recording.sharpa_hand_desired_command_clock_ns",
    ):
        features[key] = {
            "dtype": "int64",
            "shape": (2, 2),
            "names": [
                ["left", "right"],
                ["workstation_receive_monotonic_ns", "workstation_receive_realtime_ns"],
            ],
        }

    for _color_key, _source_name, feature_key in CAMERA_LAYOUT:
        features[feature_key] = {
            "dtype": "video",
            "shape": (3, 480, 640),
            "names": ["channel", "height", "width"],
        }
    for feature_key, _hand_key, _finger_name in TACTILE_KEYS:
        features[feature_key] = {
            "dtype": "video",
            "shape": (3, 240, 240),
            "names": ["channel", "height", "width"],
        }
    return features


def _event_rows(capture: NativeCapture, alignment: EpisodeAlignment) -> list[dict]:
    mapped_by_event: dict[int, int] = {}
    canonical_by_event: dict[int, int] = {}
    for canonical_index, channel in enumerate(capture.canonical_channel_arrays()):
        mapped = alignment.availability_ns_by_channel[canonical_index]
        for event_index, availability_ns in zip(channel.event_index, mapped):
            mapped_by_event[int(event_index)] = int(availability_ns)
            canonical_by_event[int(event_index)] = canonical_index

    rows: list[dict] = []
    for record in capture.tactile_records:
        event_index = int(record.event_index)
        rows.append(
            {
                "event_index": event_index,
                "canonical_channel": canonical_by_event[event_index],
                "canonical_finger": CANONICAL_FINGERS[canonical_by_event[event_index]],
                "raw_channel": int(record.channel),
                "chunk_index": int(record.chunk_index),
                "chunk_path": record.chunk_name,
                "record_index_in_chunk": int(record.record_index),
                "recorder_receive_monotonic_ns": int(record.recorder_receive_monotonic_ns),
                "recorder_receive_realtime_ns": int(record.recorder_receive_realtime_ns),
                "aligned_workstation_availability_monotonic_ns": mapped_by_event[event_index],
                "publisher_receive_monotonic_ns": int(record.publisher_receive_monotonic_ns),
                "publisher_receive_realtime_ns": int(record.publisher_receive_realtime_ns),
                "source_frame_id": int(record.source_frame_id),
                "source_timestamp": float(record.source_timestamp),
                "publisher_sequence": int(record.publisher_sequence),
                "publisher_session_id": record.publisher_session_id,
                "hand_side": record.hand_side,
                "f0": float(record.f6[0]),
                "f1": float(record.f6[1]),
                "f2": float(record.f6[2]),
                "f3": float(record.f6[3]),
                "f4": float(record.f6[4]),
                "f5": float(record.f6[5]),
                "contact_point_json": json.dumps(list(record.contact_point), separators=(",", ":")),
                "contact_point_shape_json": json.dumps(
                    list(record.contact_point_shape), separators=(",", ":")
                ),
                "deform_height": int(record.deform_height),
                "deform_width": int(record.deform_width),
                "deform_bytes": int(record.deform_byte_length),
                "stream_schema_version": int(record.stream_schema_version),
                "stream_protocol": record.stream_protocol,
                "tactile_config_sha256": record.tactile_config_sha256,
                "tactile_config_path": record.tactile_config_path,
            }
        )
    return rows


def _telemetry_rows(capture: NativeCapture) -> list[dict]:
    rows: list[dict] = []
    for record in capture.hand_telemetry_records:
        row = {
            "telemetry_index": int(record.telemetry_index),
            "chunk_index": int(record.chunk_index),
            "chunk_path": record.chunk_name,
            "record_index_in_chunk": int(record.record_index),
            "recorder_receive_monotonic_ns": int(record.recorder_receive_monotonic_ns),
            "recorder_receive_realtime_ns": int(record.recorder_receive_realtime_ns),
            "kind": int(record.kind),
            "kind_name": record.kind_name,
            "side": int(record.side),
            "side_name": record.side_name,
            "complete": bool(record.complete),
            "sequence": int(record.sequence),
            "source_monotonic_ns": int(record.source_monotonic_ns),
            "source_realtime_ns": int(record.source_realtime_ns),
            "status": int(record.status),
            "source_desired_sequence": int(record.source_desired_sequence),
        }
        row.update({f"joint_{index}": float(value) for index, value in enumerate(record.values)})
        rows.append(row)
    return rows


def write_episode_sidecars(
    stage_dir: Path,
    output_episode_index: int,
    source_episode_dir: Path,
    capture: NativeCapture,
    alignment: EpisodeAlignment,
    alignment_rows: list[dict],
) -> Path:
    destination = stage_dir / "chunk-000" / f"episode_{output_episode_index:06d}"
    destination.mkdir(parents=True, exist_ok=False)
    pd.DataFrame(_event_rows(capture, alignment)).to_parquet(
        destination / "events.parquet", index=False, compression="zstd"
    )
    pd.DataFrame(_telemetry_rows(capture)).to_parquet(
        destination / "hand_telemetry.parquet", index=False, compression="zstd"
    )
    pd.DataFrame(alignment_rows).to_parquet(
        destination / "alignment.parquet", index=False, compression="zstd"
    )
    shutil.copy2(source_episode_dir / "sharpa_native_capture.json", destination / "source_capture.json")
    return destination


def _copy_native_raw(
    source_capture_dir: Path,
    destination: Path,
    mode: Literal["copy", "hardlink"],
) -> None:
    destination.mkdir(parents=True, exist_ok=False)
    sources = sorted(path for path in source_capture_dir.iterdir() if path.is_file())
    if not sources:
        raise ConversionError(f"native capture directory is empty: {source_capture_dir}")
    for source in sources:
        target = destination / source.name
        if mode == "copy":
            shutil.copy2(source, target)
        elif mode == "hardlink":
            os.link(source, target)
        else:
            raise AssertionError(f"unexpected native storage mode: {mode}")


def _add_quantile_stats(dataset) -> None:
    from lerobot.datasets.utils import write_stats

    stats = copy.deepcopy(dataset.meta.stats)
    parquet_paths = sorted((dataset.root / "data").glob("*/*.parquet"))
    if not parquet_paths:
        raise ConversionError("LeRobot writer produced no episode parquet files")
    tables = [pd.read_parquet(path) for path in parquet_paths]

    def numeric_array(value: object) -> np.ndarray:
        array = np.asarray(value)
        while array.dtype == object:
            array = np.stack([np.asarray(item) for item in array], axis=0)
        return array.astype(np.float64, copy=False)

    for feature_key, feature in dataset.meta.info["features"].items():
        if "float" not in str(feature.get("dtype", "")):
            continue
        values: list[np.ndarray] = []
        for table in tables:
            if feature_key not in table:
                raise ConversionError(f"saved parquet is missing float feature {feature_key}")
            values.extend(numeric_array(item) for item in table[feature_key])
        array = np.stack(values, axis=0)
        # GR00T treats the six native-rate slots as the sample axis and the 60
        # wrench components as the feature axis. Match gr00t/data/stats.py's
        # np.vstack behavior so force_180 statistics are directly consumable.
        stats_array = (
            array.reshape(-1, array.shape[-1])
            if feature_key == "observation.tactile_180.force"
            else (array[:, None] if array.ndim == 1 else array)
        )
        stats[feature_key] = {
            "min": np.min(stats_array, axis=0),
            "max": np.max(stats_array, axis=0),
            "mean": np.mean(stats_array, axis=0),
            "std": np.std(stats_array, axis=0),
            "count": np.asarray([stats_array.shape[0]], dtype=np.int64),
            "q01": np.quantile(stats_array, 0.01, axis=0),
            "q99": np.quantile(stats_array, 0.99, axis=0),
        }
    dataset.meta.stats = stats
    write_stats(stats, dataset.root)


def _modality_metadata(
    tactile_model_profile: Literal["bimanual", "left", "right"] = "bimanual",
) -> dict:
    force_slices = {
        "bimanual": (0, 60),
        "left": (0, 30),
        "right": (30, 60),
    }
    try:
        force_start, force_end = force_slices[tactile_model_profile]
    except KeyError as exc:
        raise ConversionError(
            f"unsupported tactile model profile: {tactile_model_profile!r}"
        ) from exc
    tactile = {
        f"{finger}_deform": {"original_key": feature_key}
        for finger, (feature_key, _hand, _finger_name) in zip(CANONICAL_FINGERS, TACTILE_KEYS)
    }
    return {
        "state": {
            "left_arm": {"start": 0, "end": 7},
            "right_arm": {"start": 7, "end": 14},
            "left_hand": {"start": 14, "end": 36},
            "right_hand": {"start": 36, "end": 58},
        },
        "action": {
            "left_arm": {"start": 0, "end": 7},
            "right_arm": {"start": 7, "end": 14},
            "left_hand": {"start": 14, "end": 36},
            "right_hand": {"start": 36, "end": 58},
        },
        "video": {
            "high": {"original_key": "observation.images.cam_left_high"},
            "left_wrist_view": {"original_key": "observation.images.cam_left_wrist"},
            "right_wrist_view": {"original_key": "observation.images.cam_right_wrist"},
        },
        "annotation": {"human.task_description": {"original_key": "task_index"}},
        "tactile": tactile,
        "tactile_force": {
            # Existing tactile-GR00T configs keep using this 30 Hz key.
            "force": {
                "original_key": "observation.tactile.force",
                "start": force_start,
                "end": force_end,
            },
            # Aligned archival/future-adapter field. Current pretrained T-Rex
            # must keep selecting `force`: it requires a 16-sample window and
            # cannot consume this packed six-native-slot row directly.
            "force_180": {"original_key": "observation.tactile_180.force"},
        },
    }


def convert(config: ConverterConfig) -> Path:
    try:
        from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset
    except ImportError as exc:
        raise ConversionError(
            "LeRobot is not importable. Run this converter with the v2.1 environment "
            "(on this workstation: /home/haochen/anaconda3/envs/teleop/bin/python)."
        ) from exc
    if CODEBASE_VERSION != "v2.1":
        raise ConversionError(
            f"this converter requires LeRobot codebase v2.1, but imported {CODEBASE_VERSION!r}; "
            "use the teleop environment instead of this repository's v3 environment"
        )

    raw_dir = config.raw_dir.expanduser().resolve()
    output_dir = config.output_dir.expanduser().resolve()
    if output_dir.exists():
        raise ConversionError(f"refusing to overwrite existing output directory: {output_dir}")
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    episodes = discover_episode_dirs(raw_dir)
    capture_tools_dir = resolve_capture_tools_dir(config.capture_tools_dir)
    max_hold_ns = int(round(config.max_tactile_hold_ms * 1_000_000.0))

    partial_root = output_dir.parent / f".{output_dir.name}.partial-{uuid.uuid4().hex}"
    native_stage = output_dir.parent / f".{output_dir.name}.native-stage-{uuid.uuid4().hex}"
    native_stage.mkdir(parents=False, exist_ok=False)
    dataset = None
    source_mappings: list[dict] = []
    alignment_summaries: list[dict] = []
    try:
        dataset = LeRobotDataset.create(
            repo_id=config.repo_id,
            fps=POLICY_FPS,
            features=create_features(),
            root=partial_root,
            robot_type=ROBOT_TYPE,
            use_videos=True,
            tolerance_s=0.0001,
            image_writer_processes=config.image_writer_processes,
            image_writer_threads=config.image_writer_threads,
        )
        # The tactile GR00T loader resolves fingertip streams through its
        # tactile_path alias. They share LeRobot's ordinary per-video layout.
        dataset.meta.info["tactile_path"] = dataset.meta.info["video_path"]

        for output_episode_index, episode_dir in enumerate(episodes):
            source, rows = load_source_episode(episode_dir)
            task, task_provenance = resolve_task_text(source, config.task_description, episode_dir)
            capture = load_native_capture(episode_dir, capture_tools_dir)
            alignment = prepare_episode_alignment(capture, config.max_tactile_hold_ms)
            alignment_rows: list[dict] = []

            for output_frame_index, source_position in enumerate(range(1, len(rows))):
                frame, alignment_row = build_frame(
                    episode_dir,
                    rows[source_position],
                    rows[source_position - 1],
                    alignment,
                    max_hold_ns,
                )
                alignment_row["episode_index"] = output_episode_index
                alignment_row["frame_index"] = output_frame_index
                alignment_rows.append(alignment_row)
                # LeRobot's standard timestamp remains the exact nominal
                # frame_index/30 value required by its v2.1 synchronization
                # validator. Absolute source clocks are preserved separately.
                dataset.add_frame(frame, task=task)

            dataset.save_episode()
            sidecar_dir = write_episode_sidecars(
                native_stage,
                output_episode_index,
                episode_dir,
                capture,
                alignment,
                alignment_rows,
            )
            source_mappings.append(
                {
                    "episode_index": output_episode_index,
                    "source_episode": episode_dir.name,
                    "source_rows": len(rows),
                    "dropped_source_rows": [0],
                    "output_frames": len(rows) - 1,
                    "capture_id": capture.capture_id,
                    "task": task_provenance,
                    "source_info": source.get("info"),
                    "native_sidecar": str(
                        Path("native_tactile") / sidecar_dir.relative_to(native_stage)
                    ),
                }
            )
            fit_dict = dataclasses.asdict(alignment.clock_fit)
            alignment_summaries.append(
                {
                    "episode_index": output_episode_index,
                    "source_episode": episode_dir.name,
                    "capture_id": capture.capture_id,
                    "clock_fit": fit_dict,
                    "native_event_counts_canonical": [
                        int(array.size) for array in alignment.event_index_by_channel
                    ],
                    "prefill_slots": int(
                        sum(sum(row["prefill_mask"]) for row in alignment_rows)
                    ),
                    "repeated_slots": int(
                        sum(sum(row["repeat_mask"]) for row in alignment_rows)
                    ),
                }
            )
            print(
                f"converted {episode_dir.name} -> episode_{output_episode_index:06d}: "
                f"{len(rows) - 1} frames, capture {capture.capture_id}",
                flush=True,
            )

        if dataset is None:
            raise AssertionError("internal error: dataset was not created")
        dataset.stop_image_writer()
        _add_quantile_stats(dataset)

        (partial_root / "meta").mkdir(parents=True, exist_ok=True)
        _write_json(
            partial_root / "meta" / "modality.json",
            _modality_metadata(config.tactile_model_profile),
        )
        _write_json(
            partial_root / "meta" / "high_rate_tactile.json",
            {
                "schema": ALIGNMENT_SCHEMA,
                "policy_fps": POLICY_FPS,
                "nominal_tactile_rate_hz": TACTILE_RATE_HZ,
                "window_size": TACTILE_WINDOW_SIZE,
                "target_offsets_ns_oldest_to_current": target_offsets_ns(
                    TACTILE_RATE_HZ, TACTILE_WINDOW_SIZE
                ).tolist(),
                "canonical_finger_order": list(CANONICAL_FINGERS),
                "raw_channel_for_canonical_finger": [9, 8, 7, 6, 5, 4, 3, 2, 1, 0],
                "force_axis_order": ["fx", "fy", "fz", "tx", "ty", "tz"],
                "alignment_anchor": "timestamps.workstation_monotonic_ns at each retained 30 Hz row",
                "native_availability_boundary": (
                    "Thor recorder input receive_monotonic_ns mapped to workstation monotonic time"
                ),
                "resampling": "causal previous-sample hold on a fixed 180 Hz grid",
                "rate_shortfall_handling": "repeat the previous real event and set repeat_mask; never zero-fill",
                "startup_handling": "repeat the first event available by the policy tick and set prefill_mask",
                "long_gap_policy": f"fail when non-startup hold age exceeds {config.max_tactile_hold_ms} ms",
                "standard_lerobot_timestamp": "nominal frame_index / 30; absolute clocks are recording.* features",
                "deployment_contract": (
                    "Build the same six-slot history at the Thor recorder/preprocessor availability boundary "
                    "using unitree_lerobot.utils.h2_sharpa_180_alignment.build_causal_tactile_window. "
                    "Do not substitute future-nearest samples. Existing camera, robot and hand values remain "
                    "their latest causal cached values at the policy tick. For workstation inference, freeze "
                    "that observation bundle while waiting for the matching timestamped Thor window; transport "
                    "may add end-to-end latency but must not change relative observation ages."
                ),
                "episodes": alignment_summaries,
            },
        )
        _write_json(
            partial_root / "meta" / "conversion.json",
            {
                "converter": "convert_h2_sharpa_180_to_lerobot_v21.py",
                "alignment_schema": ALIGNMENT_SCHEMA,
                "repo_id": config.repo_id,
                "source_root": str(raw_dir),
                "row_policy": "drop source row 0 from every episode; reindex output rows and episodes from 0",
                "action_semantics": (
                    "arm actions use actions.{left,right}_arm.qpos; hand actions use "
                    "actions.{left,right}_ee.qpos, matching the historical H2 Sharpa converter's "
                    "observed-pose feedback-proxy semantics; that hand vector is also retained in "
                    "recording.sharpa_hand_action_proxy for explicit provenance"
                ),
                "native_storage_mode": config.native_storage_mode,
                "tactile_model_profile": config.tactile_model_profile,
                "capture_tools_dir": str(capture_tools_dir),
                "episodes": source_mappings,
            },
        )

        native_destination = partial_root / "native_tactile"
        os.replace(native_stage, native_destination)
        for mapping, episode_dir in zip(source_mappings, episodes):
            output_episode_index = int(mapping["episode_index"])
            raw_destination = (
                native_destination / "chunk-000" / f"episode_{output_episode_index:06d}" / "raw"
            )
            _copy_native_raw(
                episode_dir / "sharpa_native_capture",
                raw_destination,
                config.native_storage_mode,
            )

        os.replace(partial_root, output_dir)
        print(f"complete: {output_dir}", flush=True)
        return output_dir
    except Exception:
        if dataset is not None:
            try:
                dataset.stop_image_writer()
            except Exception:
                pass
        # Preserve a partially written LeRobot root as evidence. The staging
        # directory contains only generated sidecars and is safe to remove.
        if native_stage.exists():
            shutil.rmtree(native_stage)
        if partial_root.exists():
            print(f"conversion failed; partial output preserved at {partial_root}", file=sys.stderr)
        raise


def parse_args(argv: Sequence[str] | None = None) -> ConverterConfig:
    parser = argparse.ArgumentParser(
        description="Convert H2 Sharpa 30 Hz + native tactile captures to aligned LeRobot v2.1"
    )
    parser.add_argument("--raw-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repo-id", required=True, help="LeRobot/Hugging Face dataset identifier")
    parser.add_argument(
        "--task-description",
        default=None,
        help="Task text for every episode; defaults to each data.json text.goal",
    )
    parser.add_argument(
        "--capture-tools-dir",
        type=Path,
        default=None,
        help="Directory containing sharpa_capture_format.py and sharpa_pb2.py",
    )
    parser.add_argument(
        "--native-storage-mode",
        choices=("copy", "hardlink"),
        default="copy",
        help="How to preserve lossless .shc files inside the output dataset",
    )
    parser.add_argument(
        "--tactile-model-profile",
        choices=("bimanual", "left", "right"),
        default="bimanual",
        help=(
            "Legacy 30 Hz force slice exposed as tactile_force.force: bimanual=60, "
            "left/right=30. Storage remains bimanual for every profile."
        ),
    )
    parser.add_argument("--max-tactile-hold-ms", type=float, default=50.0)
    parser.add_argument("--image-writer-processes", type=int, default=0)
    parser.add_argument("--image-writer-threads", type=int, default=4)
    args = parser.parse_args(argv)
    return ConverterConfig(
        raw_dir=args.raw_dir,
        output_dir=args.output_dir,
        repo_id=args.repo_id,
        task_description=args.task_description,
        capture_tools_dir=args.capture_tools_dir,
        native_storage_mode=args.native_storage_mode,
        tactile_model_profile=args.tactile_model_profile,
        max_tactile_hold_ms=args.max_tactile_hold_ms,
        image_writer_processes=args.image_writer_processes,
        image_writer_threads=args.image_writer_threads,
    )


def main(argv: Sequence[str] | None = None) -> int:
    try:
        convert(parse_args(argv))
    except (ConversionError, NativeCaptureError, ValueError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
