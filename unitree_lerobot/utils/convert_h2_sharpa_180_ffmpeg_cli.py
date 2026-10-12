#!/usr/bin/env python3
"""Run convert_h2_sharpa_180_to_lerobot_v21 with video encoding done by an external ffmpeg process.

Why: the long-running converter intermittently dies with impossible AttributeErrors (wrong
attribute name / wrong object inside a running frame) — Python heap corruption from native code.
LeRobot 0.3.4's `encode_video_frames` encodes IN-PROCESS via PyAV + libsvtav1. This wrapper
replaces it with an equivalent `ffmpeg` CLI invocation (same codec, pix_fmt, GOP, CRF) so the
encoder runs in its own process, then calls the converter's unmodified `main()`.

Usage: identical CLI to the converter, e.g.
  PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python -m \
    unitree_lerobot.utils.convert_h2_sharpa_180_ffmpeg_cli --raw-dir ... --output-dir ... --repo-id ...
The ffmpeg binary is taken from $LEROBOT_FFMPEG, else <sys.prefix>/bin/ffmpeg, else PATH; it must
provide the `libsvtav1` encoder (the `teleop` conda env's ffmpeg does).
"""
from __future__ import annotations

import glob
import os
import shutil
import subprocess
import sys
from pathlib import Path

import lerobot.datasets.lerobot_dataset as _lerobot_dataset
import lerobot.datasets.video_utils as _video_utils


def _ffmpeg_binary() -> str:
    for candidate in (os.environ.get("LEROBOT_FFMPEG"), os.path.join(sys.prefix, "bin", "ffmpeg"), shutil.which("ffmpeg")):
        if candidate and os.access(candidate, os.X_OK):
            return candidate
    raise FileNotFoundError("no ffmpeg binary found (set LEROBOT_FFMPEG)")


FFMPEG = _ffmpeg_binary()
_encoders = subprocess.run([FFMPEG, "-hide_banner", "-encoders"], capture_output=True, text=True).stdout
if "libsvtav1" not in _encoders:
    raise RuntimeError(f"{FFMPEG} lacks the libsvtav1 encoder")


def encode_video_frames_cli(
    imgs_dir,
    video_path,
    fps: int,
    vcodec: str = "libsvtav1",
    pix_fmt: str = "yuv420p",
    g: int | None = 2,
    crf: int | None = 30,
    fast_decode: int = 0,
    log_level=None,
    overwrite: bool = False,
) -> None:
    """Drop-in for lerobot.datasets.video_utils.encode_video_frames using an ffmpeg subprocess."""
    if vcodec not in ("h264", "hevc", "libsvtav1"):
        raise ValueError(f"Unsupported video codec: {vcodec}")
    video_path = Path(video_path)
    imgs_dir = Path(imgs_dir)
    video_path.parent.mkdir(parents=True, exist_ok=overwrite)
    if vcodec in ("libsvtav1", "hevc") and pix_fmt == "yuv444p":
        pix_fmt = "yuv420p"
    frames = sorted(glob.glob(str(imgs_dir / ("frame_" + "[0-9]" * 6 + ".png"))))
    if not frames:
        raise FileNotFoundError(f"No images found in {imgs_dir}.")
    start = int(Path(frames[0]).stem.split("_")[-1])
    cmd = [
        FFMPEG, "-hide_banner", "-loglevel", "error", "-y" if overwrite else "-n",
        "-f", "image2", "-framerate", str(fps), "-start_number", str(start),
        "-i", str(imgs_dir / "frame_%06d.png"),
        "-vcodec", vcodec, "-pix_fmt", pix_fmt,
    ]
    if g is not None:
        cmd += ["-g", str(g)]
    if crf is not None:
        cmd += ["-crf", str(crf)]
    if fast_decode:
        cmd += (["-svtav1-params", f"fast-decode={fast_decode}"] if vcodec == "libsvtav1" else ["-tune", "fastdecode"])
    cmd += [str(video_path)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise OSError(f"ffmpeg failed ({result.returncode}) for {video_path}: {result.stderr.strip()[-500:]}")
    if not video_path.exists():
        raise OSError(f"Video encoding did not work. File not found: {video_path}.")


# Patch both the defining module and the name imported into lerobot_dataset.
_video_utils.encode_video_frames = encode_video_frames_cli
_lerobot_dataset.encode_video_frames = encode_video_frames_cli

from unitree_lerobot.utils.convert_h2_sharpa_180_to_lerobot_v21 import main  # noqa: E402

if __name__ == "__main__":
    print(f"[ffmpeg-cli wrapper] video encoding via external process: {FFMPEG}", flush=True)
    raise SystemExit(main())
