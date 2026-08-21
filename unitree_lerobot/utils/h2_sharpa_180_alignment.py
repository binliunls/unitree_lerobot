"""Causal time alignment shared by H2 Sharpa conversion and deployment.

The policy clock is the workstation monotonic clock. Native tactile events are
timestamped on Thor, so :func:`fit_thor_to_workstation_clock` estimates a
robust affine map from four-timestamp clock exchanges. The tactile window
builder then uses only samples that were available at each target time. This
module deliberately has no LeRobot or capture-format dependencies so the same
window construction can be reused by an online deployment preprocessor.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np


class AlignmentError(ValueError):
    """Raised when a causal alignment cannot be constructed safely."""


@dataclass(frozen=True)
class ClockFit:
    """Affine mapping and diagnostics for ``Thor monotonic -> workstation monotonic``."""

    slope: float
    thor_origin_ns: float
    workstation_origin_ns: float
    intercept_ns: float
    input_anchor_count: int
    valid_anchor_count: int
    rtt_retained_anchor_count: int
    retained_anchor_count: int
    thor_span_ns: float
    rtt_median_ns: float
    rtt_mad_ns: float
    residual_median_abs_ns: float
    residual_max_abs_ns: float

    def apply(self, thor_monotonic_ns: int | np.ndarray) -> int | np.ndarray:
        """Map one or more Thor monotonic timestamps into workstation time."""

        values = np.asarray(thor_monotonic_ns, dtype=np.float64)
        mapped = np.rint(
            self.workstation_origin_ns + self.slope * (values - self.thor_origin_ns)
        ).astype(np.int64)
        if values.ndim == 0:
            return int(mapped)
        return mapped


@dataclass(frozen=True)
class CausalTactileWindow:
    """A fixed-rate tactile history assembled at one policy decision tick."""

    target_grid_ns: np.ndarray
    selected_availability_ns: np.ndarray
    force: np.ndarray
    source_event_index: np.ndarray
    sample_age_ns: np.ndarray
    grid_hold_age_ns: np.ndarray
    repeat_mask: np.ndarray
    prefill_mask: np.ndarray


_CLOCK_KEYS = (
    "workstation_send_monotonic_ns",
    "server_receive_monotonic_ns",
    "server_send_monotonic_ns",
    "workstation_receive_monotonic_ns",
)


def _median_absolute_deviation(values: np.ndarray) -> float:
    median = float(np.median(values))
    return float(np.median(np.abs(values - median)))


def _upper_mad_mask(values: np.ndarray, sigma: float) -> tuple[np.ndarray, float, float]:
    """Keep finite, non-negative values below a robust upper limit."""

    finite_nonnegative = np.isfinite(values) & (values >= 0.0)
    candidates = values[finite_nonnegative]
    if candidates.size == 0:
        return np.zeros(values.shape, dtype=bool), float("nan"), float("nan")

    median = float(np.median(candidates))
    mad = _median_absolute_deviation(candidates)
    robust_sigma = 1.4826 * mad
    # With identical good RTTs, MAD is zero. Keep those exact values while
    # still rejecting a strictly larger RTT.
    upper = median + sigma * robust_sigma if robust_sigma > 0.0 else median
    return finite_nonnegative & (values <= upper), median, mad


def _theil_sen_slope(x: np.ndarray, y: np.ndarray) -> float:
    slopes: list[np.ndarray] = []
    for index in range(x.size - 1):
        delta_x = x[index + 1 :] - x[index]
        valid = delta_x != 0.0
        if np.any(valid):
            slopes.append((y[index + 1 :][valid] - y[index]) / delta_x[valid])
    if not slopes:
        raise AlignmentError("clock anchors do not contain distinct Thor timestamps")
    return float(np.median(np.concatenate(slopes)))


def fit_thor_to_workstation_clock(
    samples: Sequence[Mapping[str, object]],
    *,
    min_anchors: int = 6,
    min_span_ns: int = 3_000_000_000,
    rtt_mad_sigma: float = 3.0,
    residual_mad_sigma: float = 4.0,
    max_slope_error_ppm: float = 1_000.0,
    max_abs_residual_ns: int = 5_000_000,
) -> ClockFit:
    """Fit a robust affine Thor-to-workstation monotonic clock mapping.

    Each input sample must contain a standard four-timestamp exchange:
    workstation send ``t1``, Thor receive ``t2``, Thor send ``t3``, and
    workstation receive ``t4``. The anchor pair is the midpoint on each host,
    and the network RTT is ``(t4-t1) - (t3-t2)``. High-RTT anchors are rejected
    before the affine fit, followed by one robust residual-rejection pass.
    """

    if min_anchors < 2:
        raise ValueError("min_anchors must be at least 2")
    if min_span_ns <= 0:
        raise ValueError("min_span_ns must be positive")
    if rtt_mad_sigma < 0.0 or residual_mad_sigma < 0.0:
        raise ValueError("MAD rejection thresholds must be non-negative")

    thor_midpoints: list[float] = []
    workstation_midpoints: list[float] = []
    network_rtts: list[float] = []

    for sample in samples:
        if any(sample.get(key) is None for key in _CLOCK_KEYS):
            continue
        try:
            t1, t2, t3, t4 = (int(sample[key]) for key in _CLOCK_KEYS)
        except (TypeError, ValueError, OverflowError):
            continue
        if t4 < t1 or t3 < t2:
            continue

        thor_midpoints.append(t2 + (t3 - t2) / 2.0)
        workstation_midpoints.append(t1 + (t4 - t1) / 2.0)
        network_rtts.append(float((t4 - t1) - (t3 - t2)))

    valid_anchor_count = len(thor_midpoints)
    if valid_anchor_count < min_anchors:
        raise AlignmentError(
            f"need at least {min_anchors} valid clock anchors, found {valid_anchor_count}"
        )

    x_all = np.asarray(thor_midpoints, dtype=np.float64)
    y_all = np.asarray(workstation_midpoints, dtype=np.float64)
    rtt_all = np.asarray(network_rtts, dtype=np.float64)
    rtt_mask, rtt_median_ns, rtt_mad_ns = _upper_mad_mask(rtt_all, rtt_mad_sigma)
    if int(np.count_nonzero(rtt_mask)) < min_anchors:
        raise AlignmentError(
            "RTT outlier rejection left "
            f"{int(np.count_nonzero(rtt_mask))} clock anchors; need {min_anchors}"
        )

    x = x_all[rtt_mask]
    y = y_all[rtt_mask]
    rtt_retained_anchor_count = int(x.size)
    order = np.argsort(x, kind="stable")
    x = x[order]
    y = y[order]
    if float(x[-1] - x[0]) < float(min_span_ns):
        raise AlignmentError(
            f"clock anchor span is {float(x[-1] - x[0]):.0f} ns; need at least {min_span_ns} ns"
        )

    # Centering avoids loss of precision from the workstation's large
    # monotonic-clock epoch. Theil-Sen gives a robust seed for residual
    # rejection; retained anchors are then fit by centered least squares.
    x_center = float(np.median(x))
    y_center = float(np.median(y))
    x_relative = x - x_center
    y_relative = y - y_center
    seed_slope = _theil_sen_slope(x_relative, y_relative)
    seed_offset = float(np.median(y_relative - seed_slope * x_relative))
    seed_residuals = y_relative - (seed_offset + seed_slope * x_relative)
    seed_residual_median = float(np.median(seed_residuals))
    seed_residual_mad = _median_absolute_deviation(seed_residuals)
    robust_residual_sigma = 1.4826 * seed_residual_mad
    if robust_residual_sigma > 0.0:
        residual_limit = residual_mad_sigma * robust_residual_sigma
        residual_mask = np.abs(seed_residuals - seed_residual_median) <= residual_limit
    else:
        residual_mask = seed_residuals == seed_residual_median

    if int(np.count_nonzero(residual_mask)) < min_anchors:
        raise AlignmentError(
            "clock residual outlier rejection left "
            f"{int(np.count_nonzero(residual_mask))} anchors; need {min_anchors}"
        )

    x = x[residual_mask]
    y = y[residual_mask]
    retained_anchor_count = int(x.size)
    thor_origin_ns = float(np.mean(x))
    workstation_mean_ns = float(np.mean(y))
    centered_x = x - thor_origin_ns
    centered_y = y - workstation_mean_ns
    denominator = float(np.dot(centered_x, centered_x))
    if denominator <= 0.0:
        raise AlignmentError("retained clock anchors do not span distinct Thor timestamps")
    slope = float(np.dot(centered_x, centered_y) / denominator)
    workstation_origin_ns = float(np.mean(y - slope * centered_x))

    slope_error_ppm = abs(slope - 1.0) * 1_000_000.0
    if slope_error_ppm > max_slope_error_ppm:
        raise AlignmentError(
            f"implausible clock slope {slope:.12f} ({slope_error_ppm:.1f} ppm from 1.0)"
        )

    residuals = y - (workstation_origin_ns + slope * (x - thor_origin_ns))
    absolute_residuals = np.abs(residuals)
    residual_median_abs_ns = float(np.median(absolute_residuals))
    residual_max_abs_ns = float(np.max(absolute_residuals))
    if residual_max_abs_ns > float(max_abs_residual_ns):
        raise AlignmentError(
            f"clock fit residual reaches {residual_max_abs_ns:.0f} ns; "
            f"limit is {max_abs_residual_ns} ns"
        )

    intercept_ns = workstation_origin_ns - slope * thor_origin_ns
    return ClockFit(
        slope=slope,
        thor_origin_ns=thor_origin_ns,
        workstation_origin_ns=workstation_origin_ns,
        intercept_ns=intercept_ns,
        input_anchor_count=len(samples),
        valid_anchor_count=valid_anchor_count,
        rtt_retained_anchor_count=rtt_retained_anchor_count,
        retained_anchor_count=retained_anchor_count,
        thor_span_ns=float(np.max(x) - np.min(x)),
        rtt_median_ns=rtt_median_ns,
        rtt_mad_ns=rtt_mad_ns,
        residual_median_abs_ns=residual_median_abs_ns,
        residual_max_abs_ns=residual_max_abs_ns,
    )


def target_offsets_ns(rate_hz: float = 180.0, window_size: int = 6) -> np.ndarray:
    """Return oldest-to-current fixed-rate offsets for one tactile window."""

    if not np.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError("rate_hz must be finite and positive")
    if window_size <= 0:
        raise ValueError("window_size must be positive")
    steps = np.arange(-(window_size - 1), 1, dtype=np.float64)
    return np.rint(steps * 1_000_000_000.0 / rate_hz).astype(np.int64)


def build_causal_tactile_window(
    policy_tick_ns: int,
    availability_ns_by_channel: Sequence[np.ndarray],
    source_event_index_by_channel: Sequence[np.ndarray],
    force_by_channel: Sequence[np.ndarray],
    *,
    offsets_ns: np.ndarray | None = None,
    rate_hz: float = 180.0,
    window_size: int = 6,
    max_hold_ns: int = 50_000_000,
) -> CausalTactileWindow:
    """Construct a 10-finger causal tactile window using zero-order hold.

    For every target-grid time, the latest event whose availability timestamp
    is no later than that grid point is selected. If a grid point predates the
    first event but that first event is already available by the policy tick,
    startup priming repeats it and marks ``prefill_mask``. A channel with no
    event available by the policy tick fails instead of inventing a zero.
    """

    channel_count = 10
    force_width = 6
    if not (
        len(availability_ns_by_channel)
        == len(source_event_index_by_channel)
        == len(force_by_channel)
        == channel_count
    ):
        raise ValueError("expected availability, event-index, and force arrays for exactly 10 channels")
    if max_hold_ns < 0:
        raise ValueError("max_hold_ns must be non-negative")

    if offsets_ns is None:
        offsets = target_offsets_ns(rate_hz=rate_hz, window_size=window_size)
    else:
        offsets = np.asarray(offsets_ns, dtype=np.int64)
        if offsets.ndim != 1 or offsets.size == 0:
            raise ValueError("offsets_ns must be a non-empty one-dimensional array")
        if np.any(np.diff(offsets) < 0):
            raise ValueError("offsets_ns must be ordered oldest to newest")
        if int(offsets[-1]) > 0:
            raise ValueError("offsets_ns cannot extend beyond the policy tick")

    tick = int(policy_tick_ns)
    grid = tick + offsets
    sample_count = int(offsets.size)
    output_force = np.empty((sample_count, channel_count * force_width), dtype=np.float32)
    output_indices = np.empty((sample_count, channel_count), dtype=np.int64)
    selected_availability = np.empty((sample_count, channel_count), dtype=np.int64)
    sample_age = np.empty((sample_count, channel_count), dtype=np.int64)
    grid_hold_age = np.empty((sample_count, channel_count), dtype=np.int64)
    repeat_mask = np.zeros((sample_count, channel_count), dtype=np.uint8)
    prefill_mask = np.zeros((sample_count, channel_count), dtype=np.uint8)

    for channel in range(channel_count):
        availability = np.asarray(availability_ns_by_channel[channel], dtype=np.int64)
        event_indices = np.asarray(source_event_index_by_channel[channel], dtype=np.int64)
        forces = np.asarray(force_by_channel[channel], dtype=np.float32)
        if availability.ndim != 1 or event_indices.ndim != 1:
            raise ValueError(f"channel {channel}: availability and event indices must be one-dimensional")
        if forces.ndim != 2 or forces.shape[1] != force_width:
            raise ValueError(f"channel {channel}: force must have shape [events, 6]")
        if not (availability.size == event_indices.size == forces.shape[0]):
            raise ValueError(f"channel {channel}: event arrays have inconsistent lengths")
        if availability.size and np.any(np.diff(availability) < 0):
            raise ValueError(f"channel {channel}: availability timestamps are not sorted")

        available_by_tick = int(np.searchsorted(availability, tick, side="right"))
        if available_by_tick == 0:
            raise AlignmentError(f"channel {channel}: no tactile event is available by policy tick {tick}")

        for sample_index, target_ns in enumerate(grid):
            selected_index = int(np.searchsorted(availability, int(target_ns), side="right")) - 1
            if selected_index < 0:
                # Startup priming is causal with respect to the policy tick,
                # although the first sample was not yet available at this
                # earlier target-grid point. The mask preserves this fact.
                selected_index = 0
                prefill_mask[sample_index, channel] = 1
            elif int(target_ns) - int(availability[selected_index]) > max_hold_ns:
                hold_ns = int(target_ns) - int(availability[selected_index])
                raise AlignmentError(
                    f"channel {channel}: zero-order hold age {hold_ns} ns exceeds {max_hold_ns} ns "
                    f"at target {int(target_ns)}"
                )

            selected_ns = int(availability[selected_index])
            if selected_ns > tick:
                raise AssertionError("internal error: selected tactile event is after the policy tick")
            if not prefill_mask[sample_index, channel] and selected_ns > int(target_ns):
                raise AssertionError("internal error: non-prefill tactile event is after its target grid")

            selected_availability[sample_index, channel] = selected_ns
            output_indices[sample_index, channel] = int(event_indices[selected_index])
            output_force[sample_index, channel * force_width : (channel + 1) * force_width] = forces[
                selected_index
            ]
            sample_age[sample_index, channel] = tick - selected_ns
            grid_hold_age[sample_index, channel] = max(0, int(target_ns) - selected_ns)
            if sample_index > 0 and output_indices[sample_index, channel] == output_indices[sample_index - 1, channel]:
                repeat_mask[sample_index, channel] = 1

    return CausalTactileWindow(
        target_grid_ns=grid,
        selected_availability_ns=selected_availability,
        force=output_force,
        source_event_index=output_indices,
        sample_age_ns=sample_age,
        grid_hold_age_ns=grid_hold_age,
        repeat_mask=repeat_mask,
        prefill_mask=prefill_mask,
    )
