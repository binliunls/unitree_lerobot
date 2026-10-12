#!/usr/bin/env python3
"""Batched + resumable driver for convert_h2_sharpa_180_to_lerobot_v21.

Why: a single long conversion run dies intermittently after a few dozen episodes with
interpreter heap corruption (impossible AttributeError / unpack errors). Converting in small
batches, each in a fresh process (via convert_h2_sharpa_180_ffmpeg_cli, i.e. external video
encoding), and retrying a batch on failure makes completion deterministic. The batch
datasets are then merged into ONE LeRobot v2.1 dataset identical in layout to a single-run
output: parquet episode_index/index renumbered, videos + native sidecars renamed,
episodes.jsonl / episodes_stats.jsonl / conversion.json / high_rate_tactile.json renumbered,
info.json totals recomputed, stats.json re-aggregated with LeRobot's aggregate_stats.

Usage (teleop env, cwd unitree_lerobot, run inside tmux):
  PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python -m unitree_lerobot.utils.convert_h2_sharpa_180_batched \
      --raw-dir RAW --output-dir OUT --repo-id ID --task-description "..." \
      [--tactile-model-profile bimanual] [--native-storage-mode hardlink] [--batch-size 15] [--max-retries 3]
Re-running resumes: finished batches (complete meta/info.json) are skipped.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq

from lerobot.datasets.compute_stats import aggregate_stats
from lerobot.datasets.utils import cast_stats_to_numpy, write_stats


def log(msg: str) -> None:
    print(f"[batched {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def run_batch(args, batch_index: int, episodes: list[Path], work: Path) -> Path:
    raw_b = work / f"raw_b{batch_index:02d}"
    out_b = work / f"out_b{batch_index:02d}"
    info_b = out_b / "meta" / "info.json"
    if info_b.is_file() and json.load(open(info_b))["total_episodes"] == len(episodes):
        log(f"batch {batch_index}: already complete ({len(episodes)} episodes), skipping")
        return out_b
    if raw_b.exists():
        shutil.rmtree(raw_b)
    raw_b.mkdir(parents=True)
    for k, ep in enumerate(episodes, start=1):
        os.symlink(ep.resolve(), raw_b / f"episode_{k:04d}")
    env = dict(os.environ, PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION="python", PYTHONUNBUFFERED="1")
    cmd = [
        sys.executable, "-X", "faulthandler", "-m", "unitree_lerobot.utils.convert_h2_sharpa_180_ffmpeg_cli",
        "--raw-dir", str(raw_b), "--output-dir", str(out_b), "--repo-id", args.repo_id,
        "--task-description", args.task_description,
        "--tactile-model-profile", args.tactile_model_profile,
        "--native-storage-mode", args.native_storage_mode,
        "--max-tactile-hold-ms", str(args.max_tactile_hold_ms),
    ]
    if args.capture_tools_dir:
        cmd += ["--capture-tools-dir", str(args.capture_tools_dir)]
    for attempt in range(1, args.max_retries + 1):
        for stale in work.glob(f".out_b{batch_index:02d}.*"):
            shutil.rmtree(stale, ignore_errors=True)
        if out_b.exists():
            shutil.rmtree(out_b)
        log_path = work / f"out_b{batch_index:02d}.attempt{attempt}.log"
        log(f"batch {batch_index}: attempt {attempt}/{args.max_retries}, {len(episodes)} episodes -> {out_b.name} (log {log_path.name})")
        t0 = time.time()
        with open(log_path, "w") as lf:
            rc = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT, env=env, cwd=str(Path(__file__).resolve().parents[2])).returncode
        if rc == 0 and info_b.is_file() and json.load(open(info_b))["total_episodes"] == len(episodes):
            log(f"batch {batch_index}: OK in {(time.time()-t0)/60:.1f} min")
            return out_b
        tail = subprocess.run(["tail", "-n", "3", str(log_path)], capture_output=True, text=True).stdout.strip().replace("\n", " | ")
        log(f"batch {batch_index}: FAILED (rc={rc}) after {(time.time()-t0)/60:.1f} min: {tail[-300:]}")
    raise SystemExit(f"batch {batch_index} failed {args.max_retries} times")


def _link_or_copy(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def _rewrite_int_column(table: pa.Table, name: str, values) -> pa.Table:
    idx = table.schema.get_field_index(name)
    field = table.schema.field(idx)
    return table.set_column(idx, field, pa.array(values, type=field.type))


def merge(batch_dirs: list[Path], output_dir: Path, raw_dir: Path, repo_id: str) -> None:
    if output_dir.exists():
        raise SystemExit(f"refusing to overwrite existing output directory: {output_dir}")
    first = batch_dirs[0]
    info = json.load(open(first / "meta/info.json"))
    chunks_size = int(info["chunks_size"])
    video_keys = [k for k, v in info["features"].items() if v["dtype"] == "video"]
    tasks = open(first / "meta/tasks.jsonl").read()
    conv = json.load(open(first / "meta/conversion.json"))
    hrt = json.load(open(first / "meta/high_rate_tactile.json"))
    modality = (first / "meta/modality.json").read_text()

    (output_dir / "meta").mkdir(parents=True)
    ep_off = 0
    frame_off = 0
    episodes_lines: list[str] = []
    stats_lines: list[str] = []
    conv_eps: list[dict] = []
    hrt_eps: list[dict] = []
    all_ep_stats: list[dict] = []

    for b in batch_dirs:
        info_b = json.load(open(b / "meta/info.json"))
        assert info_b["features"] == info["features"], f"{b}: feature schema differs"
        assert open(b / "meta/tasks.jsonl").read() == tasks, f"{b}: tasks differ"
        eps_b = [json.loads(l) for l in open(b / "meta/episodes.jsonl")]
        st_b = {d["episode_index"]: d for d in (json.loads(l) for l in open(b / "meta/episodes_stats.jsonl"))}
        conv_b = {d["episode_index"]: d for d in json.load(open(b / "meta/conversion.json"))["episodes"]}
        hrt_b = {d["episode_index"]: d for d in json.load(open(b / "meta/high_rate_tactile.json"))["episodes"]}
        n_b = info_b["total_episodes"]
        assert len(eps_b) == n_b == len(st_b) == len(conv_b) == len(hrt_b), f"{b}: inconsistent episode counts"
        for e in sorted(eps_b, key=lambda d: d["episode_index"]):
            old = int(e["episode_index"]); new = ep_off + old
            c_old, c_new = old // chunks_size, new // chunks_size
            # parquet: renumber episode_index and global index
            t = pq.read_table(b / f"data/chunk-{c_old:03d}/episode_{old:06d}.parquet")
            n = t.num_rows
            assert n == int(e["length"]), f"{b} ep {old}: parquet rows {n} != episodes.jsonl length {e['length']}"
            t = _rewrite_int_column(t, "episode_index", [new] * n)
            t = _rewrite_int_column(t, "index", [frame_off + i for i in range(n)])
            dst = output_dir / f"data/chunk-{c_new:03d}/episode_{new:06d}.parquet"
            dst.parent.mkdir(parents=True, exist_ok=True)
            pq.write_table(t, dst)
            # videos
            for vk in video_keys:
                _link_or_copy(b / f"videos/chunk-{c_old:03d}/{vk}/episode_{old:06d}.mp4",
                              output_dir / f"videos/chunk-{c_new:03d}/{vk}/episode_{new:06d}.mp4")
            # native tactile sidecar
            src_sc = b / f"native_tactile/chunk-{c_old:03d}/episode_{old:06d}"
            dst_sc = output_dir / f"native_tactile/chunk-{c_new:03d}/episode_{new:06d}"
            for p in src_sc.rglob("*"):
                rel = p.relative_to(src_sc)
                if p.is_dir():
                    (dst_sc / rel).mkdir(parents=True, exist_ok=True)
                elif rel.name == "alignment.parquet":
                    at = pq.read_table(p)
                    if "episode_index" in at.column_names:
                        at = _rewrite_int_column(at, "episode_index", [new] * at.num_rows)
                    (dst_sc / rel).parent.mkdir(parents=True, exist_ok=True)
                    pq.write_table(at, dst_sc / rel)
                else:
                    _link_or_copy(p, dst_sc / rel)
            # meta lines
            e2 = dict(e, episode_index=new); episodes_lines.append(json.dumps(e2))
            s2 = dict(st_b[old], episode_index=new); stats_lines.append(json.dumps(s2)); all_ep_stats.append(cast_stats_to_numpy(s2["stats"]))
            ce = dict(conv_b[old], episode_index=new, native_sidecar=f"native_tactile/chunk-{c_new:03d}/episode_{new:06d}"); conv_eps.append(ce)
            he = dict(hrt_b[old], episode_index=new); hrt_eps.append(he)
            frame_off += n
        ep_off += n_b
        log(f"merged {b.name}: {n_b} episodes -> total {ep_off} episodes, {frame_off} frames")

    (output_dir / "meta/episodes.jsonl").write_text("\n".join(episodes_lines) + "\n")
    (output_dir / "meta/episodes_stats.jsonl").write_text("\n".join(stats_lines) + "\n")
    (output_dir / "meta/tasks.jsonl").write_text(tasks)
    (output_dir / "meta/modality.json").write_text(modality)
    info.update(total_episodes=ep_off, total_frames=frame_off, total_videos=ep_off * len(video_keys),
                total_chunks=math.ceil(ep_off / chunks_size), splits={"train": f"0:{ep_off}"})
    json.dump(info, open(output_dir / "meta/info.json", "w"), indent=4)
    conv.update(episodes=conv_eps, repo_id=repo_id, source_root=str(raw_dir.resolve()),
                batched_conversion={"batches": [d.name for d in batch_dirs], "driver": Path(__file__).name})
    json.dump(conv, open(output_dir / "meta/conversion.json", "w"), indent=2)
    hrt.update(episodes=hrt_eps)
    json.dump(hrt, open(output_dir / "meta/high_rate_tactile.json", "w"), indent=2)
    write_stats(aggregate_stats(all_ep_stats), output_dir)
    # GR00T's generate_stats only skips its (container-crashing) recomputation when every
    # float feature already has q01/q99; LeRobot's aggregate_stats does not write them.
    from unitree_lerobot.utils.add_gr00t_stats_quantiles import add_quantiles
    add_quantiles(output_dir)
    (output_dir / "images").mkdir(exist_ok=True)
    log(f"merge done: {ep_off} episodes, {frame_off} frames -> {output_dir}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw-dir", type=Path, required=True)
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--repo-id", required=True)
    ap.add_argument("--task-description", required=True)
    ap.add_argument("--tactile-model-profile", default="bimanual")
    ap.add_argument("--native-storage-mode", default="hardlink")
    ap.add_argument("--max-tactile-hold-ms", type=float, default=50.0)
    ap.add_argument("--capture-tools-dir", type=Path, default=None)
    ap.add_argument("--batch-size", type=int, default=15)
    ap.add_argument("--max-retries", type=int, default=3)
    ap.add_argument("--work-dir", type=Path, default=None)
    ap.add_argument("--merge-only", action="store_true", help="skip conversion; merge existing batch outputs")
    a = ap.parse_args(argv)
    work = a.work_dir or a.output_dir.parent / f".{a.output_dir.name}.batches"
    work.mkdir(parents=True, exist_ok=True)
    episodes = sorted(p for p in a.raw_dir.glob("episode_*") if (p / "data.json").is_file())
    if not episodes:
        raise SystemExit(f"no episodes under {a.raw_dir}")
    batches = [episodes[i:i + a.batch_size] for i in range(0, len(episodes), a.batch_size)]
    log(f"{len(episodes)} episodes in {len(batches)} batches of <= {a.batch_size}; work dir {work}")
    batch_dirs = []
    for bi, eps in enumerate(batches):
        batch_dirs.append(work / f"out_b{bi:02d}" if a.merge_only else run_batch(a, bi, eps, work))
    merge(batch_dirs, a.output_dir, a.raw_dir, a.repo_id)
    log("ALL DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
