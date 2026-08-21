import unittest

import numpy as np

from unitree_lerobot.utils.h2_sharpa_180_alignment import (
    AlignmentError,
    build_causal_tactile_window,
    fit_thor_to_workstation_clock,
    target_offsets_ns,
)


def _ten_channel_events(availability_ns: np.ndarray) -> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
    availability = np.asarray(availability_ns, dtype=np.int64)
    availability_by_channel: list[np.ndarray] = []
    event_indices_by_channel: list[np.ndarray] = []
    force_by_channel: list[np.ndarray] = []
    for channel in range(10):
        availability_by_channel.append(availability.copy())
        event_indices = channel * 10_000 + np.arange(availability.size, dtype=np.int64)
        event_indices_by_channel.append(event_indices)
        force = np.stack(
            [event_indices.astype(np.float32) + component / 10.0 for component in range(6)], axis=1
        )
        force_by_channel.append(force)
    return availability_by_channel, event_indices_by_channel, force_by_channel


class CausalTactileWindowTest(unittest.TestCase):
    def test_target_offsets_are_oldest_to_current(self) -> None:
        offsets = target_offsets_ns(rate_hz=180.0, window_size=6)
        np.testing.assert_array_equal(
            offsets,
            np.array([-27_777_778, -22_222_222, -16_666_667, -11_111_111, -5_555_556, 0]),
        )

    def test_177_hz_source_repeats_neighbor_without_future_selection(self) -> None:
        source_period_ns = int(round(1_000_000_000 / 177.0))
        availability, event_indices, forces = _ten_channel_events(
            np.arange(12, dtype=np.int64) * source_period_ns
        )
        policy_tick_ns = 27_777_778

        window = build_causal_tactile_window(
            policy_tick_ns,
            availability,
            event_indices,
            forces,
            max_hold_ns=10_000_000,
        )

        self.assertEqual(window.force.shape, (6, 60))
        self.assertEqual(window.source_event_index.shape, (6, 10))
        self.assertGreater(int(window.repeat_mask.sum()), 0)
        self.assertEqual(int(window.prefill_mask.sum()), 0)
        self.assertTrue(np.all(window.selected_availability_ns <= window.target_grid_ns[:, None]))
        self.assertTrue(np.all(window.selected_availability_ns <= policy_tick_ns))
        # The repeat is a real previous sample, not a zero-filled placeholder.
        repeated_slot, repeated_channel = np.argwhere(window.repeat_mask == 1)[0]
        np.testing.assert_array_equal(
            window.force[repeated_slot, repeated_channel * 6 : (repeated_channel + 1) * 6],
            window.force[repeated_slot - 1, repeated_channel * 6 : (repeated_channel + 1) * 6],
        )

    def test_startup_prefill_is_marked_and_uses_only_data_available_by_tick(self) -> None:
        availability, event_indices, forces = _ten_channel_events(np.array([5_000_000, 12_000_000]))
        policy_tick_ns = 10_000_000

        window = build_causal_tactile_window(
            policy_tick_ns,
            availability,
            event_indices,
            forces,
            max_hold_ns=50_000_000,
        )

        self.assertGreater(int(window.prefill_mask.sum()), 0)
        self.assertTrue(np.all(window.selected_availability_ns <= policy_tick_ns))
        self.assertTrue(np.all(window.source_event_index == window.source_event_index[0]))
        self.assertTrue(np.all(window.sample_age_ns == 5_000_000))
        self.assertTrue(np.all(window.grid_hold_age_ns >= 0))

    def test_startup_fails_when_any_channel_has_no_event_by_tick(self) -> None:
        availability, event_indices, forces = _ten_channel_events(np.array([5_000_000]))
        availability[4] = np.array([11_000_000], dtype=np.int64)

        with self.assertRaisesRegex(AlignmentError, "channel 4: no tactile event"):
            build_causal_tactile_window(10_000_000, availability, event_indices, forces)

    def test_long_zero_order_hold_fails_instead_of_hiding_stream_loss(self) -> None:
        availability, event_indices, forces = _ten_channel_events(np.array([0], dtype=np.int64))

        with self.assertRaisesRegex(AlignmentError, "zero-order hold age"):
            build_causal_tactile_window(
                100_000_000,
                availability,
                event_indices,
                forces,
                max_hold_ns=50_000_000,
            )


class ClockFitTest(unittest.TestCase):
    def test_rejects_high_rtt_anchor_and_recovers_affine_mapping(self) -> None:
        true_slope = 1.000020
        true_offset_ns = 6_465_000_000_000_000
        server_duration_ns = 100_000
        good_rtts_ns = [950_000, 1_000_000, 1_050_000, 980_000, 1_020_000, 970_000]
        samples: list[dict[str, int]] = []

        for index in range(12):
            thor_midpoint_ns = 700_000_000_000 + index * 500_000_000
            workstation_midpoint_ns = int(round(true_offset_ns + true_slope * thor_midpoint_ns))
            network_rtt_ns = good_rtts_ns[index % len(good_rtts_ns)]
            if index == 5:
                network_rtt_ns = 100_000_000
            total_exchange_ns = server_duration_ns + network_rtt_ns
            samples.append(
                {
                    "workstation_send_monotonic_ns": workstation_midpoint_ns - total_exchange_ns // 2,
                    "server_receive_monotonic_ns": thor_midpoint_ns - server_duration_ns // 2,
                    "server_send_monotonic_ns": thor_midpoint_ns + server_duration_ns // 2,
                    "workstation_receive_monotonic_ns": workstation_midpoint_ns + total_exchange_ns // 2,
                }
            )

        fit = fit_thor_to_workstation_clock(samples)

        self.assertEqual(fit.input_anchor_count, 12)
        self.assertEqual(fit.valid_anchor_count, 12)
        self.assertEqual(fit.rtt_retained_anchor_count, 11)
        self.assertGreaterEqual(fit.retained_anchor_count, 10)
        self.assertAlmostEqual(fit.slope, true_slope, places=8)
        test_thor_ns = 707_250_000_000
        expected_workstation_ns = int(round(true_offset_ns + true_slope * test_thor_ns))
        self.assertLessEqual(abs(fit.apply(test_thor_ns) - expected_workstation_ns), 4)
        self.assertLessEqual(fit.residual_max_abs_ns, 4.0)

    def test_requires_enough_span(self) -> None:
        samples = []
        for index in range(6):
            thor_ns = 1_000_000_000 + index * 100_000_000
            workstation_ns = 5_000_000_000 + thor_ns
            samples.append(
                {
                    "workstation_send_monotonic_ns": workstation_ns - 550_000,
                    "server_receive_monotonic_ns": thor_ns - 50_000,
                    "server_send_monotonic_ns": thor_ns + 50_000,
                    "workstation_receive_monotonic_ns": workstation_ns + 550_000,
                }
            )

        with self.assertRaisesRegex(AlignmentError, "clock anchor span"):
            fit_thor_to_workstation_clock(samples)


if __name__ == "__main__":
    unittest.main()
