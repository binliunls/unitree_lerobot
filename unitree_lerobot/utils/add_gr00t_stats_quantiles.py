#!/usr/bin/env python3
"""Add GR00T-style q01/q99 quantiles to a LeRobot v2.1 dataset's meta/stats.json.

GR00T's DatasetFactory calls gr00t.data.stats.generate_stats on rank 0; it only skips the
(slow, and in the gr00tn17 container crashing: pandas concat of `datasets` extension dtypes)
recomputation when meta/stats.json has mean/std/min/max/q01/q99 for EVERY float feature.
LeRobot's own aggregate_stats never writes q01/q99, so a merged/aggregated stats.json fails
that check. This script reproduces GR00T's calculate_dataset_statistics numbers with pyarrow
(no pandas concat): per float feature, np.vstack of every row (a [6,60] cell contributes 6
rows), then quantiles over axis 0 -> same shapes as the medical_pads stats.json.

Usage: python -m unitree_lerobot.utils.add_gr00t_stats_quantiles DATASET_DIR
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq


def _dense(v):
    if isinstance(v, np.ndarray) and v.dtype == object:
        return np.stack([_dense(x) for x in v])
    return np.asarray(v)


def add_quantiles(dataset_dir: Path) -> list[str]:
    info = json.load(open(dataset_dir / "meta/info.json"))
    feats = [k for k, v in info["features"].items() if "float" in v["dtype"]]
    files = sorted(dataset_dir.glob("data/chunk-*/episode_*.parquet"))
    if not files:
        raise SystemExit(f"no parquet files under {dataset_dir}")
    stacks: dict[str, list[np.ndarray]] = {f: [] for f in feats}
    for p in files:
        t = pq.read_table(p, columns=feats)
        for f in feats:
            col = t.column(f).to_pandas().to_numpy()
            stacks[f].append(np.vstack([np.asarray(_dense(x), dtype=np.float32).reshape(-1, _dense(x).shape[-1]) if _dense(x).ndim > 1 else np.asarray(_dense(x), dtype=np.float32).reshape(1, -1) for x in col]))
    stats_path = dataset_dir / "meta/stats.json"
    stats = json.load(open(stats_path))
    for f in feats:
        data = np.vstack(stacks[f])
        stats.setdefault(f, {})
        stats[f]["q01"] = np.quantile(data, 0.01, axis=0).tolist()
        stats[f]["q99"] = np.quantile(data, 0.99, axis=0).tolist()
        for k, fn in (("mean", np.mean), ("std", np.std), ("min", np.min), ("max", np.max)):
            stats[f].setdefault(k, fn(data, axis=0).tolist())
        print(f"  {f:45s} rows={data.shape[0]:8d} dim={data.shape[1]:3d}  q01/q99 added")
    json.dump(stats, open(stats_path, "w"), indent=4)
    return feats


if __name__ == "__main__":
    d = Path(sys.argv[1]).resolve()
    feats = add_quantiles(d)
    print(f"updated {d/'meta/stats.json'} for {len(feats)} float features")
