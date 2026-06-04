"""
Upload a converted LeRobot dataset directory to a Hugging Face Hub repo.

Uploads the entire local dataset folder (data/, images/, videos/, meta/, ...)
into a target sub-folder of the destination repo. The target sub-folder is
created on the Hub automatically if it doesn't exist.

Usage (tv conda env):

    python -m unitree_lerobot.utils.upload_lerobot_dataset \\
        --local-path ~/lerobot \\
        --repo-id nvidia/orca-template1-dev \\
        --path-in-repo 0512_1751_liming_collection

The Hub repo itself must already exist (or pass --create to create it).
Auth: pass --token, or set $HF_TOKEN, or run `huggingface-cli login` first.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

try:
    from huggingface_hub import HfApi
    from huggingface_hub.errors import HfHubHTTPError
except ImportError:
    sys.exit(
        "huggingface_hub is required. Install with:\n"
        "    pip install -U huggingface_hub"
    )


def _resolve_token(token_arg: str | None) -> str | None:
    """Pick a token in order: --token, HF_TOKEN env, cached login (None lets HF resolve)."""
    if token_arg:
        return token_arg
    env_tok = os.environ.get("HF_TOKEN") or os.environ.get("HUGGING_FACE_HUB_TOKEN")
    if env_tok:
        return env_tok
    # Returning None lets HfApi fall back to ~/.cache/huggingface/token.
    return None


def upload(
    local_path: Path,
    repo_id: str,
    path_in_repo: str,
    repo_type: str = "dataset",
    token: str | None = None,
    commit_message: str | None = None,
    create_repo: bool = False,
    private: bool = False,
    ignore_patterns: list[str] | None = None,
) -> None:
    local_path = local_path.expanduser().resolve()
    if not local_path.is_dir():
        raise FileNotFoundError(f"local_path is not a directory: {local_path}")

    api = HfApi(token=token)

    if create_repo:
        api.create_repo(
            repo_id=repo_id, repo_type=repo_type, private=private, exist_ok=True
        )
        print(f"[upload] ensured repo exists: {repo_type}:{repo_id}")

    if commit_message is None:
        commit_message = f"Upload {local_path.name} to {path_in_repo}"

    print(f"[upload] source     : {local_path}")
    print(f"[upload] repo       : {repo_type}:{repo_id}")
    print(f"[upload] target dir : {path_in_repo}")
    print(f"[upload] commit msg : {commit_message}")

    api.upload_folder(
        folder_path=str(local_path),
        repo_id=repo_id,
        repo_type=repo_type,
        path_in_repo=path_in_repo,
        commit_message=commit_message,
        ignore_patterns=ignore_patterns,
    )
    print(
        f"[upload] done. Browse at: "
        f"https://huggingface.co/{'datasets/' if repo_type == 'dataset' else ''}"
        f"{repo_id}/tree/main/{path_in_repo}"
    )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--local-path", required=True, type=Path,
                   help="Local LeRobot dataset directory to upload (contains data/, images/, videos/, meta/).")
    p.add_argument("--repo-id", required=True,
                   help="Destination repo, e.g. nvidia/orca-template1-dev")
    p.add_argument("--path-in-repo", required=True,
                   help="Target sub-folder within the repo, e.g. 0512_1751_liming_collection. "
                        "Created automatically if it doesn't exist.")
    p.add_argument("--repo-type", default="dataset", choices=["dataset", "model", "space"],
                   help="HF repo type (default: dataset).")
    p.add_argument("--token", default=None,
                   help="HF access token. Falls back to $HF_TOKEN, then cached login.")
    p.add_argument("--commit-message", default=None,
                   help="Commit message (default: auto).")
    p.add_argument("--create", action="store_true",
                   help="Create the destination repo if it doesn't exist (idempotent).")
    p.add_argument("--private", action="store_true",
                   help="Create as a private repo (only used with --create).")
    p.add_argument("--ignore", nargs="*", default=None, metavar="GLOB",
                   help="Glob pattern(s) of files to skip, e.g. --ignore '*.tmp' '__pycache__'.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    token = _resolve_token(args.token)
    try:
        upload(
            local_path=args.local_path,
            repo_id=args.repo_id,
            path_in_repo=args.path_in_repo,
            repo_type=args.repo_type,
            token=token,
            commit_message=args.commit_message,
            create_repo=args.create,
            private=args.private,
            ignore_patterns=args.ignore,
        )
    except HfHubHTTPError as e:
        sys.exit(f"[upload] HF Hub error: {e}")
    except Exception as e:
        sys.exit(f"[upload] failed: {type(e).__name__}: {e}")


if __name__ == "__main__":
    main()
