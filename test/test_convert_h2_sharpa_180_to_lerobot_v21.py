import json
import tempfile
import unittest
from types import SimpleNamespace
from pathlib import Path

import numpy as np
import pandas as pd

from unitree_lerobot.utils.convert_h2_sharpa_180_to_lerobot_v21 import (
    _add_quantile_stats,
    _modality_metadata,
    _parse_state_action,
    create_features,
    load_source_episode,
    parse_args,
    resolve_task_text,
)


class H2Sharpa180ConverterTest(unittest.TestCase):
    def test_source_row_zero_is_identified_for_drop_and_task_can_be_overridden(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            episode = Path(temporary) / "episode_0007"
            episode.mkdir()
            source = {
                "text": {
                    "goal": "raw goal",
                    "desc": "raw description",
                    "steps": "raw steps",
                },
                "data": [{"idx": 0}, {"idx": 1}, {"idx": 2}],
            }
            (episode / "data.json").write_text(json.dumps(source), encoding="utf-8")

            loaded, rows = load_source_episode(episode)

            self.assertEqual([row["idx"] for row in rows[1:]], [1, 2])
            task, provenance = resolve_task_text(loaded, None, episode)
            self.assertEqual(task, "raw goal")
            self.assertEqual(provenance["raw_description"], "raw description")
            override, override_provenance = resolve_task_text(loaded, "  operator task  ", episode)
            self.assertEqual(override, "operator task")
            self.assertEqual(override_provenance["selected_from"], "command_line_override")

    def test_hand_action_matches_legacy_observed_pose_qpos(self) -> None:
        row = {
            "states": {
                "left_arm": {"qpos": list(range(7))},
                "right_arm": {"qpos": list(range(10, 17))},
                "left_ee": {"qpos": list(range(20, 42))},
                "right_ee": {"qpos": list(range(50, 72))},
                "body": {"qpos": [0.1, 0.2, 0.3]},
            },
            "actions": {
                "left_arm": {"qpos": list(range(100, 107))},
                "right_arm": {"qpos": list(range(110, 117))},
                "left_ee": {
                    "qpos": list(range(120, 142)),
                    "desired_qpos": list(range(220, 242)),
                },
                "right_ee": {
                    "qpos": list(range(150, 172)),
                    "desired_qpos": list(range(250, 272)),
                },
            },
        }

        state, action, proxy, waist = _parse_state_action(row, "fixture")

        self.assertEqual(state.shape, (58,))
        np.testing.assert_array_equal(action[14:36], np.arange(120, 142, dtype=np.float32))
        np.testing.assert_array_equal(action[36:58], np.arange(150, 172, dtype=np.float32))
        np.testing.assert_array_equal(proxy[:22], np.arange(120, 142, dtype=np.float32))
        np.testing.assert_allclose(waist, [0.1, 0.2, 0.3])

    def test_schema_is_v21_compatible_and_exposes_current_gr00t_modalities(self) -> None:
        features = create_features()
        self.assertEqual(features["observation.state"]["shape"], (58,))
        self.assertEqual(features["action"]["shape"], (58,))
        self.assertEqual(features["observation.tactile_180.force"]["shape"], (6, 60))
        self.assertEqual(features["observation.tactile_180.repeat_mask"]["dtype"], "uint8")
        self.assertEqual(features["observation.images.cam_left_high"]["shape"], (3, 480, 640))

        modality = _modality_metadata()
        self.assertEqual(modality["state"]["right_hand"], {"start": 36, "end": 58})
        self.assertEqual(
            modality["tactile_force"]["force"]["original_key"],
            "observation.tactile.force",
        )
        self.assertEqual(
            modality["tactile_force"]["force_180"]["original_key"],
            "observation.tactile_180.force",
        )
        self.assertEqual(len(modality["tactile"]), 10)

        self.assertEqual(
            _modality_metadata("left")["tactile_force"]["force"],
            {"original_key": "observation.tactile.force", "start": 0, "end": 30},
        )
        self.assertEqual(
            _modality_metadata("right")["tactile_force"]["force"],
            {"original_key": "observation.tactile.force", "start": 30, "end": 60},
        )

    def test_cli_has_explicit_task_description(self) -> None:
        config = parse_args(
            [
                "--raw-dir",
                "/tmp/raw",
                "--output-dir",
                "/tmp/output",
                "--repo-id",
                "owner/dataset",
                "--task-description",
                "assemble the trocar",
                "--tactile-model-profile",
                "left",
            ]
        )
        self.assertEqual(config.task_description, "assemble the trocar")
        self.assertEqual(config.tactile_model_profile, "left")

    def test_high_rate_statistics_collapse_policy_rows_and_native_slots(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data_dir = root / "data" / "chunk-000"
            (root / "meta").mkdir()
            data_dir.mkdir(parents=True)
            first = np.arange(6 * 60, dtype=np.float32).reshape(6, 60)
            second = first + 1000.0
            pd.DataFrame(
                {
                    "observation.tactile_180.force": [first.tolist(), second.tolist()],
                    "observation.state": [[0.0, 0.0]] * 2,
                }
            ).to_parquet(data_dir / "episode_000000.parquet", index=False)
            meta = SimpleNamespace(
                stats={},
                info={
                    "features": {
                        "observation.tactile_180.force": {"dtype": "float32"},
                        "observation.state": {"dtype": "float32"},
                    }
                },
            )
            dataset = SimpleNamespace(root=root, meta=meta)

            _add_quantile_stats(dataset)

            force_stats = dataset.meta.stats["observation.tactile_180.force"]
            self.assertEqual(force_stats["mean"].shape, (60,))
            self.assertEqual(int(force_stats["count"][0]), 12)
            self.assertEqual(force_stats["q01"].shape, (60,))


if __name__ == "__main__":
    unittest.main()
