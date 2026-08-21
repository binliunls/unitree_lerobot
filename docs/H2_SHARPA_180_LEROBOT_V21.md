# H2 Sharpa 180 Hz capture to LeRobot v2.1

This dedicated converter keeps the established H2 + Sharpa 30 Hz dataset
interface and adds a causally aligned six-sample tactile-force history to every
30 Hz policy row. It is intentionally separate from the generic Unitree JSON
converter.

## Conversion command

Use the `teleop` environment. It provides the LeRobot v2.1 writer required by
this converter; the LeRobot v3 environment in this repository is rejected.

```bash
cd /home/haochen/Projects_Haochen/unitree_lerobot

/home/haochen/anaconda3/envs/teleop/bin/python -m \
  unitree_lerobot.utils.convert_h2_sharpa_180_to_lerobot_v21 \
  --raw-dir /home/haochen/Projects_Haochen/datasets_H2_sharpa/0812_test \
  --output-dir /home/haochen/Projects_Haochen/datasets_H2_sharpa/0812_test_lerobot_v21 \
  --repo-id shcSteven/h2-sharpa-0812-test \
  --task-description "Assemble the trocar and place it on the table." \
  --tactile-model-profile left \
  --native-storage-mode copy
```

`--repo-id` becomes the dataset identifier in the LeRobot metadata; this
command creates a local dataset and does not push it. The converter refuses to
overwrite an existing output directory. If a conversion fails, it prints the
location of the preserved partial LeRobot output.

The repository's v2.1 environment lock uses `pyarrow==21.0.0`. Keep that pin
when using `LeRobotDataset.__getitem__` on the converter's two-dimensional
features. The currently installed `datasets==3.3` with PyArrow 22 can write the
dataset, and the GR00T pandas loader can read it, but that particular pair has
an upstream `Array2D.to_pylist` incompatibility in direct LeRobot indexing.

`--task-description` overrides the task text for every converted episode. If
it is omitted, each episode uses `data.json["text"]["goal"]`; conversion fails
instead of inventing an empty task when neither is available. The selected
text and the raw `goal`, `desc`, and `steps` values are retained as provenance
in `meta/conversion.json`.

`--tactile-model-profile left` makes the legacy `tactile_force.force` modality
select the left-hand `[0:30]` slice required by the repository's current
five-fingertip SATA + T-Rex + delay-augmentation training config. Use
`bimanual` for ten-fingertip training (`[0:60]`) or `right` for `[30:60]`.
This option changes only `meta/modality.json`: the converted Parquet, native
sidecars, and raw `.shc` files retain both hands in every profile.

The native capture reader needs the authoritative `sharpa_capture_format.py`
and `sharpa_pb2.py`. Their directory is resolved in this order:

1. `--capture-tools-dir /path/to/docker_5_0_1`
2. the `SHARPA_CAPTURE_TOOLS_DIR` environment variable
3. the standard sibling checkout at
   `/home/haochen/Projects_Haochen/sharpa-teleop/thor/docker_5_0_1`

The default `--native-storage-mode copy` makes a portable copy of every raw
native-capture file and therefore requires additional disk space.
`--native-storage-mode hardlink` avoids duplicating file blocks, but source and
output must be on the same filesystem and modifying either linked file also
modifies the other view. Use `copy` for an independently distributable
dataset.

## Output and row semantics

The standard LeRobot v2.1 structure is created under the output root:

```text
0812_test_lerobot_v21/
├── data/chunk-000/episode_000000.parquet
├── videos/chunk-000/<video-feature>/episode_000000.mp4
├── meta/
│   ├── info.json
│   ├── tasks.jsonl
│   ├── episodes.jsonl
│   ├── episodes_stats.jsonl
│   ├── stats.json
│   ├── modality.json
│   ├── conversion.json
│   └── high_rate_tactile.json
└── native_tactile/chunk-000/episode_000000/
    ├── events.parquet
    ├── hand_telemetry.parquet
    ├── alignment.parquet
    ├── source_capture.json
    └── raw/
        ├── manifest.json
        ├── manifest.sha256
        ├── clock_samples.jsonl
        └── *.shc
```

For every source episode, source row/index 0 is dropped. Retained rows are
reindexed from frame 0, and output episodes are reindexed from episode 0. For
example, source `episode_0001` becomes output `episode_000000`, and its source
row 1 becomes output frame 0. The exact mapping is recorded in
`meta/conversion.json`.

The original policy-rate modalities remain one-to-one with each retained 30 Hz
row:

- three RGB views: mono head, left wrist, and right wrist;
- ten legacy 30 Hz tactile deformation images and their 60-value force vector;
- 58-value H2/Sharpa state and action vectors; and
- the three-value waist state plus capture and timing provenance.

The primary arm action still comes from each arm's `qpos`. To remain compatible
with the previous H2 Sharpa converter, the primary hand action uses
`actions.{left,right}_ee.qpos`: the observed-pose feedback proxy arranged by the
recorder as approximately `action[t] = state[t+1]`. It does not use
`desired_qpos`. The same hand vector is retained separately in
`recording.sharpa_hand_action_proxy` so that its provenance remains explicit.

The standard LeRobot `timestamp` remains the exact nominal
`frame_index / 30`, as required by the v2.1 synchronization validator. The
original absolute and source clocks are not discarded: `recording.*` fields
retain the workstation frame clock, camera ROS and receive clocks, H2 low-state
clock and Unitree tick, arm target/publish clocks, hand state/action/desired
command receive clocks, legacy tactile source/receive clocks, native-capture
request clock, and the native-event interval indices/counts.

## The 180 Hz field and missing slots

Each policy row contains:

```text
observation.tactile_180.force   shape [6, 60]
```

The first dimension is the six nominal 180 Hz slots, ordered oldest to current
(`t-5` through `t`). The 60 force values are ten canonical fingers (left thumb
through pinky, then right thumb through pinky), with six force/torque values
per finger.

Actual device delivery around 177--179 Hz does not cause zero filling. At each
fixed 180 Hz grid time, the converter selects the most recent real event that
was already available: causal previous-sample/zero-order hold. A missing slot
therefore repeats its previous neighbor, and
`observation.tactile_180.repeat_mask` identifies those repeats. No future
nearest sample is selected.

At startup, a grid slot can precede a finger's first native event. If that
first event is already available by the current policy tick, it is repeated
into the leading slot and marked in
`observation.tactile_180.prefill_mask`. If no real event is available for a
finger, conversion fails rather than creating zeros. Outside startup priming,
a held sample older than 50 ms also fails conversion by default; the threshold
is configurable with `--max-tactile-hold-ms`.

The selected event indices, sample ages, grid hold ages, repeat/prefill masks,
and per-policy-interval event bounds/counts are stored in the main episode
Parquet. `native_tactile/.../alignment.parquet` provides the expanded
per-frame alignment audit trail.

## Clock alignment and lossless native data

Native event availability is defined at the Thor recorder-input boundary. The
converter robustly fits an affine Thor-monotonic to workstation-monotonic map
from the four-timestamp clock exchanges, rejects high-RTT and residual
outliers, and records the fit diagnostics in `meta/high_rate_tactile.json`.
The mapped recorder receive time is then compared with each 30 Hz workstation
policy tick.

`events.parquet` contains decoded native tactile metadata, force values, clock
values, and the chunk/record coordinates of the deformation payload.
`hand_telemetry.parquet` preserves native desired/applied/state telemetry.
The deformation bytes and every other native record remain lossless in the raw
`.shc` chunks under `raw/`; they are not duplicated into Parquet.

Current `meta/modality.json` continues to expose the legacy 30 Hz tactile
images and force vector as `tactile_force.force`, so existing GR00T/tactile-
manipulation configs remain unchanged. It also exposes the aligned field as
`tactile_force.force_180` for archival discovery and a future packed-window
adapter. Existing tactile-GR00T must continue selecting `force`, not
`force_180`: the pretrained T-Rex VQ-VAE requires a 16-sample input window,
whereas one `force_180` policy row contains six native slots. The current
generic loader also stacks policy-row deltas and does not preserve a separate
candidate-row/native-slot dimension. Therefore `[6, 60]` is deliberately
stored now but is not claimed as a directly trainable current-T-Rex input.

## Deployment timing contract

Training/deployment parity is defined by availability, not by choosing samples
that are merely close in timestamp. At a policy tick, camera, robot, and hand
values remain the latest cached causal values that the recorder placed in that
30 Hz row. They must not be retimed or replaced with future-nearest values.

Deployment should construct the six tactile slots at the same Thor
recorder/preprocessor availability boundary with the pure shared function:

```python
from unitree_lerobot.utils.h2_sharpa_180_alignment import build_causal_tactile_window
```

Run that builder on Thor. Inference can run at the same boundary, or Thor can
return a timestamped window for a requested workstation policy tick. In the
latter case, freeze the camera/robot/hand bundle captured at that tick while
waiting for the matching tactile window, then infer from that frozen bundle.
The transport adds end-to-end action latency, but it does not change the
relative ages of the observations used by the policy. Do not replace the
frozen modalities with newer values while waiting.

If deployment instead continuously streams raw 180 Hz events and simply uses
whatever has arrived at the workstation, equivalent timing cannot be assumed:
this capture version did not record raw-180 workstation arrival times. A future
capture must record those arrival times and conversion/deployment must both use
that new availability boundary.

## Focused validation

Run the converter's focused unit tests without performing a full video/native
dataset conversion:

```bash
cd /home/haochen/Projects_Haochen/unitree_lerobot

/home/haochen/anaconda3/envs/teleop/bin/python -m unittest discover \
  -s test -p 'test_*h2_sharpa*.py'
```
