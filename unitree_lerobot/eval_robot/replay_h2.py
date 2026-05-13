"""
Replay a LeRobot episode on the H2 robot with Sharpa dexterous hands.

Usage (tv conda env, run from repo root):
    # replay arm + hands via direct SDK (must run on Thor, no bridge active)
    python unitree_lerobot/eval_robot/replay_h2.py \
        --dataset-dir /home/nvidia/lerobot --num-repeats 3

    # replay through the DDS bridge on Thor (run from workstation):
    python unitree_lerobot/eval_robot/replay_h2.py \
        --dataset-dir /home/nvidia/lerobot --hands-source dds

    # full replay with the robot standing on its legs (WBC yields arms via rt/arm_sdk):
    python unitree_lerobot/eval_robot/replay_h2.py \
        --dataset-dir /home/nvidia/lerobot --hands-source dds --motion

    # arm only  (no Sharpa hardware required)
    python unitree_lerobot/eval_robot/replay_h2.py \
        --dataset-dir /home/nvidia/lerobot --no-hands

    # hands only
    python unitree_lerobot/eval_robot/replay_h2.py \
        --dataset-dir /home/nvidia/lerobot --no-arm

    # specific episode, half speed
    python unitree_lerobot/eval_robot/replay_h2.py \
        --dataset-dir /home/nvidia/lerobot --episode 0 --speed 0.5
"""

import argparse
import math
import os
import sys
import time

import numpy as np
import torch

# ── paths ─────────────────────────────────────────────────────────────────────
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_XR_ROOT   = os.path.expanduser("~/binliu/xr_teleoperate")
_SHARPA_SDK = os.path.expanduser("~/Sharpa/SharpaWaveSDK_4.6.6/python")

for p in [_REPO_ROOT, _XR_ROOT]:
    if p not in sys.path:
        sys.path.insert(0, p)

# ── action layout (H2 + Sharpa, 58-dim) ──────────────────────────────────────
ARM_SLICE       = slice(0, 14)
LEFT_EE_SLICE   = slice(14, 36)
RIGHT_EE_SLICE  = slice(36, 58)
SHARPA_DOF      = 22

import logging_mp
logging_mp.basicConfig(level=logging_mp.INFO)
logger = logging_mp.getLogger(__name__)


# ── Sharpa hand helpers ───────────────────────────────────────────────────────

def _sharpa_path():
    if _SHARPA_SDK not in sys.path:
        sys.path.insert(0, _SHARPA_SDK)


def connect_hand(side: str, speed_coeff: float = 0.5, current_coeff: float = 0.6):
    _sharpa_path()
    from sharpa import SharpaWaveManager, ControlMode, ControlSource, HandSide

    hand_side = HandSide.LEFT if side == "left" else HandSide.RIGHT
    manager   = SharpaWaveManager.get_instance()
    time.sleep(1.0)

    logger.info(f"[{side}] connecting to {hand_side.name} hand...")
    try:
        hand = manager.connect(hand_side)
    except Exception as e:
        logger.warning(f"[{side}] connect failed: {e}  — skipping")
        return None

    if hand is None:
        logger.warning(f"[{side}] manager.connect returned None — skipping")
        return None

    for fn, val in [
        ("set_control_mode",   ControlMode.POSITION),
        ("set_control_source", ControlSource.SDK),
    ]:
        err = getattr(hand, fn)(val)
        if err.code != 0:
            logger.error(f"[{side}] {fn} failed: {err.message}")
            return None

    hand.set_speed_coeff(speed_coeff)
    hand.set_current_coeff(current_coeff)
    hand.start()
    hand.set_joint_position([0.0] * SHARPA_DOF, True)
    logger.info(f"[{side}] Sharpa hand ready.")
    return hand


def disconnect_hand(hand, side: str):
    if hand is None:
        return
    _sharpa_path()
    from sharpa import SharpaWaveManager
    try:
        hand.set_joint_position([0.0] * SHARPA_DOF, True)
        time.sleep(0.5)
        hand.stop()
    except Exception as e:
        logger.warning(f"[{side}] stop error: {e}")
    try:
        SharpaWaveManager.get_instance().disconnect_all()
    except Exception:
        pass
    logger.info(f"[{side}] disconnected.")


# ── Sharpa hand helpers — DDS bridge path ────────────────────────────────────

def connect_hand_dds(side: str):
    """Construct a SharpaHandDDSClient that talks to the bridge on Thor."""
    from teleop.robot_control.robot_hand_sharpa_dds import SharpaHandDDSClient
    logger.info(f"[{side}] creating DDS client (bridge on Thor must be running)...")
    client = SharpaHandDDSClient(side)
    # Send a neutral target so the bridge applies it immediately.
    client.set_joint_position([0.0] * SHARPA_DOF, False)
    logger.info(f"[{side}] DDS client ready.")
    return client


def disconnect_hand_dds(client, side: str):
    if client is None:
        return
    try:
        client.go_neutral()
        time.sleep(0.5)
    except Exception as e:
        logger.warning(f"[{side}] DDS go_neutral error: {e}")
    logger.info(f"[{side}] DDS client released.")


# ── dataset loading ───────────────────────────────────────────────────────────

def load_episode(dataset_dir: str, episode_idx: int) -> torch.Tensor:
    """Return action tensor (T, 58) for the requested episode."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset_dir = os.path.expanduser(dataset_dir)
    if os.path.exists(os.path.join(dataset_dir, "meta", "info.json")):
        root, repo_id = dataset_dir, os.path.basename(dataset_dir.rstrip("/"))
    else:
        root, repo_id = os.path.dirname(dataset_dir), os.path.basename(dataset_dir.rstrip("/"))

    dataset = LeRobotDataset(repo_id=repo_id, root=root)
    indices = [i for i, ep in enumerate(dataset.hf_dataset["episode_index"]) if ep == episode_idx]
    if not indices:
        raise ValueError(f"Episode {episode_idx} not found in {dataset_dir}")

    actions = torch.stack([dataset[i]["action"] for i in indices])
    logger.info(f"Loaded episode {episode_idx}: {len(actions)} frames, shape={tuple(actions.shape)}")
    return actions


# ── control helpers ───────────────────────────────────────────────────────────

def _countdown(seconds: int, label: str):
    for i in range(seconds, 0, -1):
        print(f"  {label}: {i}s ", end="\r", flush=True)
        time.sleep(1.0)
    print(f"  {label}: done.          ")


def _go_home(arm_ctrl, left_hand, right_hand, hold_s: int):
    logger.info(f"Going to home position (hold {hold_s}s)...")
    if arm_ctrl is not None:
        arm_ctrl.ctrl_dual_arm_go_home()
    for side, hand in [("left", left_hand), ("right", right_hand)]:
        if hand is not None:
            hand.set_joint_position([0.0] * SHARPA_DOF, True)
    _countdown(hold_s, "holding home")


def _run_episode(arm_ctrl, left_hand, right_hand, actions: torch.Tensor, dt: float):
    T = len(actions)
    for frame_idx in range(T):
        t0     = time.time()
        action = actions[frame_idx].numpy()

        if arm_ctrl is not None:
            arm_ctrl.ctrl_dual_arm(
                action[ARM_SLICE].astype(np.float64),
                np.zeros(14, dtype=np.float64),
            )

        if left_hand is not None:
            err = left_hand.set_joint_position(action[LEFT_EE_SLICE].tolist(), False)
            # DDS client returns None; SDK returns an Error object.
            if err is not None and getattr(err, "code", 0) != 0:
                logger.warning(f"[frame {frame_idx}] left hand error: {err.message}")

        if right_hand is not None:
            err = right_hand.set_joint_position(action[RIGHT_EE_SLICE].tolist(), False)
            if err is not None and getattr(err, "code", 0) != 0:
                logger.warning(f"[frame {frame_idx}] right hand error: {err.message}")

        elapsed = time.time() - t0
        time.sleep(max(0.0, dt - elapsed))

        if (frame_idx + 1) % 30 == 0:
            logger.info(f"  frame {frame_idx + 1}/{T}  loop_dt={1000*(elapsed + max(0.0, dt-elapsed)):.1f}ms")


# ── main ──────────────────────────────────────────────────────────────────────

def replay(args):
    actions = load_episode(args.dataset_dir, args.episode)
    dt      = (1.0 / 30.0) / args.speed

    # DDS factory must be initialised once before any DDS publisher/subscriber.
    # Both the arm controller and the SharpaHandDDSClient use the same factory.
    need_dds = args.arm or (args.hands and args.hands_source == "dds")
    if need_dds:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(args.dds_domain)
        logger.info(f"DDS ChannelFactory initialised on domain {args.dds_domain}.")

    # ── arm ──────────────────────────────────────────────────────────────────
    arm_ctrl = None
    if args.arm:
        from teleop.robot_control.robot_arm import H2_ArmController
        logger.info(
            f"Initialising H2_ArmController (motion_mode={args.motion}, "
            f"head_pitch_home={args.head_pitch_home})..."
        )
        arm_ctrl = H2_ArmController(
            motion_mode=args.motion,
            head_pitch_home=args.head_pitch_home,
            kp_low=args.kp_low, kp_wrist=args.kp_wrist,
            kd_low=args.kd_low, kd_wrist=args.kd_wrist,
        )
        arm_ctrl.speed_gradual_max()
        logger.info("H2_ArmController ready.")

    # ── hands ─────────────────────────────────────────────────────────────────
    left_hand  = None
    right_hand = None
    if args.hands:
        if args.hands_source == "dds":
            logger.info("Connecting hands via DDS bridge...")
            left_hand  = connect_hand_dds("left")
            right_hand = connect_hand_dds("right")
        else:
            logger.info("Connecting left hand (direct SDK)...")
            left_hand  = connect_hand("left",  args.hand_speed_coeff, args.hand_current_coeff)
            logger.info("Connecting right hand (direct SDK)...")
            right_hand = connect_hand("right", args.hand_speed_coeff, args.hand_current_coeff)
        logger.info("Waiting for hands to reach neutral...")
        time.sleep(2.0)

    logger.info(f"Hardware ready.  arm={args.arm}  hands={args.hands}  repeats={args.num_repeats}")

    # ── first home + hold ─────────────────────────────────────────────────────
    _go_home(arm_ctrl, left_hand, right_hand, hold_s=args.hold_s)

    interrupted = False
    try:
        for rep in range(args.num_repeats):
            logger.info(f"\n=== Replay {rep + 1}/{args.num_repeats} ===")
            _run_episode(arm_ctrl, left_hand, right_hand, actions, dt)

            if rep < args.num_repeats - 1:
                _go_home(arm_ctrl, left_hand, right_hand, hold_s=args.hold_s)

    except KeyboardInterrupt:
        interrupted = True
        logger.info("Interrupted by user.")

    if not interrupted and args.hold_s > 0:
        logger.info(f"All repeats done. Holding last action for {args.hold_s}s...")
        last_arm_q = actions[-1].numpy()[ARM_SLICE].astype(np.float64)
        for i in range(args.hold_s, 0, -1):
            print(f"  holding: {i}s ", end="\r", flush=True)
            if arm_ctrl is not None:
                arm_ctrl.ctrl_dual_arm(last_arm_q, np.zeros(14, dtype=np.float64))
            time.sleep(1.0)
        print("  done.          ")

    if args.hands:
        if args.hands_source == "dds":
            disconnect_hand_dds(left_hand,  "left")
            disconnect_hand_dds(right_hand, "right")
        else:
            disconnect_hand(left_hand,  "left")
            disconnect_hand(right_hand, "right")
    logger.info("Replay finished.")


def parse_args():
    ap = argparse.ArgumentParser(description="Replay H2 + Sharpa episode from a LeRobot dataset.")
    ap.add_argument("--dataset-dir", required=True,
                    help="LeRobot dataset root, e.g. /home/nvidia/lerobot")
    ap.add_argument("--episode",     type=int,   default=0,   help="Episode index (default: 0)")
    ap.add_argument("--num-repeats", type=int,   default=1,   help="Number of replay repetitions (default: 1)")
    ap.add_argument("--speed",       type=float, default=1.0, help="Playback speed multiplier (default: 1.0)")
    ap.add_argument("--hold-s",      type=int,   default=20,
                    help="Seconds to hold at home before/between/after the replay. Use a small "
                         "value (e.g. 3) for fast iteration, 0 to skip the final hold entirely. "
                         "Default: 20.")

    g = ap.add_mutually_exclusive_group()
    g.add_argument("--arm",    dest="arm",   action="store_true",  default=True)
    g.add_argument("--no-arm", dest="arm",   action="store_false")

    g = ap.add_mutually_exclusive_group()
    g.add_argument("--hands",    dest="hands", action="store_true",  default=True)
    g.add_argument("--no-hands", dest="hands", action="store_false")

    ap.add_argument("--hands-source", choices=["sdk", "dds"], default="sdk",
                    help="How to drive the Sharpa hands. 'sdk' = direct Sharpa SDK on Thor "
                         "(legacy; the bridge must be stopped). 'dds' = publish HandCmd_ to "
                         "rt/sharpa/{left,right}/cmd via the sharpa_dds_bridge running on Thor. "
                         "Default: sdk.")
    ap.add_argument("--dds-domain", type=int, default=0,
                    help="DDS domain ID for both arm and DDS-bridge hand commands (default: 0).")

    # H2 motion mode (publish to rt/arm_sdk so the WBC yields arms+head;
    # required when the robot is standing on its legs)
    ap.add_argument("--motion", action="store_true",
                    help="Enable H2 motion mode: publish on rt/arm_sdk with handover weight=1 "
                         "so the WBC yields the arm joints. Required when replaying on the "
                         "legs-active robot. Default: off (debug mode on rt/lowcmd).")
    ap.add_argument("--head-pitch-home", type=float, default=0.6, metavar="RAD",
                    help="H2 head pitch held throughout replay (radians). Replay actions only "
                         "cover arms+hands, so the head stays at this fixed pose. "
                         "Range -0.523 (up) to 0.837 (down). Default: 0.6 (looking down).")

    # H2 arm PD gains (optional overrides)
    ap.add_argument("--kp-low",   type=float, default=None, help="H2 arm kp_low  (default: 150)")
    ap.add_argument("--kp-wrist", type=float, default=None, help="H2 wrist kp    (default: 50)")
    ap.add_argument("--kd-low",   type=float, default=None, help="H2 arm kd_low  (default: 10)")
    ap.add_argument("--kd-wrist", type=float, default=None, help="H2 wrist kd    (default: 3)")

    # Sharpa hand tuning
    ap.add_argument("--hand-speed-coeff",   type=float, default=0.5, help="Sharpa speed coeff 0–1 (default: 0.5)")
    ap.add_argument("--hand-current-coeff", type=float, default=0.6, help="Sharpa current coeff 0–1 (default: 0.6)")
    return ap.parse_args()


if __name__ == "__main__":
    replay(parse_args())
