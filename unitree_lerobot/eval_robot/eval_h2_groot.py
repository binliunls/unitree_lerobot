"""
Policy inference on H2 + Sharpa using a remote GR00T policy server.

The GR00T server runs in GR00T's uv environment (separate terminal):

    cd /home/nvidia/binliu/Isaac-GR00T
    uv run python gr00t/eval/run_gr00t_server.py \\
        --model-path /path/to/checkpoint \\
        --embodiment-tag new_embodiment \\
        --port 5555

This script runs in the tv conda environment (no gr00t package required).
It communicates via ZMQ using the same msgpack wire format as GR00T's
PolicyServer / PolicyClient.

Observation keys sent to the server (from training modality config):
    video.high              (H, W, 3) uint8
    video.left_wrist_view   (H, W, 3) uint8
    video.right_wrist_view  (H, W, 3) uint8
    state.left_arm          (7,)  float32
    state.right_arm         (7,)  float32
    state.left_hand         (22,) float32
    state.right_hand        (22,) float32
    language.annotation.human.task_description  str

Action chunk returned by the server (keys depend on training config):
    left_arm   (B, T, 7)
    right_arm  (B, T, 7)
    left_hand  (B, T, 22)
    right_hand (B, T, 22)

Usage:
    python unitree_lerobot/eval_robot/eval_h2_groot.py \\
        --policy-host localhost --policy-port 5555 \\
        --task "pick up the apple" \\
        --img-server-ip 192.168.124.162

    # arm only (no Sharpa hardware)
    python unitree_lerobot/eval_robot/eval_h2_groot.py \\
        --policy-host localhost --no-hands \\
        --task "pick up the apple"
"""

import argparse
import io
import math
import os
import sys
import time

import msgpack
import numpy as np

# ── paths ─────────────────────────────────────────────────────────────────────
_REPO_ROOT  = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_XR_ROOT    = os.path.expanduser("~/binliu/xr_teleoperate")
_SHARPA_SDK = os.path.expanduser("~/Sharpa/SharpaWaveSDK_4.6.6/python")

for p in [_REPO_ROOT, _XR_ROOT]:
    if p not in sys.path:
        sys.path.insert(0, p)

# ── action layout ─────────────────────────────────────────────────────────────
ARM_LEFT_SLICE  = slice(0, 7)
ARM_RIGHT_SLICE = slice(7, 14)
LEFT_EE_SLICE   = slice(14, 36)
RIGHT_EE_SLICE  = slice(36, 58)
SHARPA_DOF      = 22
ARM_DOF         = 14

import logging_mp
logging_mp.basicConfig(level=logging_mp.INFO)
logger = logging_mp.getLogger(__name__)


# ── GR00T wire protocol (compatible with gr00t.policy.server_client) ──────────

def _np_encode(obj):
    if isinstance(obj, np.ndarray):
        buf = io.BytesIO()
        np.save(buf, obj, allow_pickle=False)
        return {"__ndarray_class__": True, "as_npy": buf.getvalue()}
    raise TypeError(f"Unknown type: {type(obj)}")


def _np_decode(obj):
    if not isinstance(obj, dict):
        return obj
    if b"__ndarray_class__" in obj or "__ndarray_class__" in obj:
        key = b"as_npy" if b"as_npy" in obj else "as_npy"
        return np.load(io.BytesIO(obj[key]), allow_pickle=False)
    return obj


class GR00TClient:
    """
    Minimal ZMQ client compatible with gr00t.policy.server_client.PolicyServer.
    Only requires zmq, msgpack, numpy — no gr00t package needed.
    """

    def __init__(self, host: str = "localhost", port: int = 5555, timeout_ms: int = 15000):
        import zmq
        self._ctx    = zmq.Context()
        self._socket = self._ctx.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._socket.connect(f"tcp://{host}:{port}")
        self._zmq = zmq

    def _send(self, endpoint: str, data: dict | None = None):
        request = {"endpoint": endpoint}
        if data is not None:
            request["data"] = data
        self._socket.send(msgpack.packb(request, default=_np_encode))
        raw = self._socket.recv()
        resp = msgpack.unpackb(raw, object_hook=_np_decode)
        if isinstance(resp, dict) and "error" in resp:
            raise RuntimeError(f"GR00T server error: {resp['error']}")
        return resp

    def _reinit_socket(self):
        import zmq
        self._socket.close()
        self._socket = self._ctx.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, 15000)
        self._socket.setsockopt(zmq.SNDTIMEO, 15000)

    def ping(self) -> bool:
        try:
            resp = self._send("ping")
            return isinstance(resp, dict) and resp.get("status") == "ok"
        except Exception:
            self._reinit_socket()
            return False

    def reset(self):
        self._send("reset", {"options": None})

    def get_action(self, obs: dict) -> tuple[dict, dict]:
        """
        obs: nested dict, values are np.ndarray with shape (B, T, ...) or lists of strings.
        Returns (action_chunk, info) where action_chunk values are (B, T, D) arrays.
        """
        response = self._send("get_action", {"observation": obs, "options": None})
        if isinstance(response, (list, tuple)) and len(response) == 2:
            return response[0], response[1]
        return response, {}

    def __del__(self):
        try:
            self._socket.close()
            self._ctx.term()
        except Exception:
            pass


# ── Sharpa hand helpers (same as eval_h2.py) ──────────────────────────────────

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

    err = hand.set_control_mode(ControlMode.POSITION)
    if err.code != 0:
        logger.error(f"[{side}] set_control_mode failed: {err.message}")
        return None

    err = hand.set_speed_coeff(speed_coeff)
    if err.code != 0:
        logger.warning(f"[{side}] set_speed_coeff failed: {err.message}")

    err = hand.set_current_coeff(current_coeff)
    if err.code != 0:
        logger.warning(f"[{side}] set_current_coeff failed: {err.message}")

    err = hand.set_control_source(ControlSource.SDK)
    if err.code != 0:
        logger.error(f"[{side}] set_control_source failed: {err.message}")
        return None

    hand.start()
    hand.set_joint_position([0.0] * SHARPA_DOF, True)  # blocking: wait for hand to reach neutral
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
    except Exception:
        pass
    try:
        SharpaWaveManager.get_instance().disconnect_all()
    except Exception:
        pass
    logger.info(f"[{side}] disconnected.")


def read_hand_state(hand) -> np.ndarray:
    if hand is None:
        return np.zeros(SHARPA_DOF, dtype=np.float32)
    try:
        err, angles_deg = hand.get_joint_position_degree()
        if err.code != 0:
            return np.zeros(SHARPA_DOF, dtype=np.float32)
        return np.array([math.radians(a) for a in angles_deg[:SHARPA_DOF]], dtype=np.float32)
    except Exception:
        return np.zeros(SHARPA_DOF, dtype=np.float32)


# ── home / countdown helpers (mirrors replay_episode.py) ─────────────────────

def _countdown(seconds: int, label: str):
    for i in range(seconds, 0, -1):
        print(f"  {label}: {i}s ", end="\r", flush=True)
        time.sleep(1.0)
    print(f"  {label}: done.          ")


def _go_home(arm_ctrl, left_hand, right_hand, hold_s: int):
    """Move arm to home, hands to neutral, then hold for hold_s seconds."""
    logger.info(f"Going to home position (hold {hold_s}s)...")
    if arm_ctrl is not None:
        arm_ctrl.ctrl_dual_arm_go_home()
    for side, hand in [("left", left_hand), ("right", right_hand)]:
        if hand is not None:
            hand.set_joint_position([0.0] * SHARPA_DOF, True)
    _countdown(hold_s, "holding home")


# ── observation helpers ───────────────────────────────────────────────────────

def _add_bt_dims(obs: dict) -> dict:
    """
    Add (B=1, T=1) leading dims to every numpy array in the nested obs dict.
    Strings become [[str]] (two levels of list wrapping).
    """
    out = {}
    for k, v in obs.items():
        if isinstance(v, np.ndarray):
            out[k] = v[np.newaxis, np.newaxis, ...]   # (H,W,3) → (1,1,H,W,3)
        elif isinstance(v, dict):
            out[k] = _add_bt_dims(v)
        else:
            out[k] = [[v]]   # str → [[str]]
    return out


def get_observations(img_client, arm_ctrl, left_hand, right_hand, cam_config, task: str) -> dict:
    """Build the nested GR00T observation dict (before adding batch/time dims)."""
    cam_name_to_key = {
        "head_camera":        "high",
        "left_wrist_camera":  "left_wrist_view",
        "right_wrist_camera": "right_wrist_view",
    }
    getter_map = {
        "high":            img_client.get_head_frame,
        "left_wrist_view": img_client.get_left_wrist_frame,
        "right_wrist_view": img_client.get_right_wrist_frame,
    }

    video = {}
    for cam_name, cfg in cam_config.items():
        if not cfg.get("enable_zmq"):
            continue
        key    = cam_name_to_key.get(cam_name)
        getter = getter_map.get(key)
        if key is None or getter is None:
            continue
        frame = getter()
        if frame is not None and frame.bgr is not None:
            video[key] = frame.bgr[:, :, ::-1].copy()   # BGR→RGB, uint8 (H,W,3)

    arm_q = arm_ctrl.get_current_dual_arm_q().astype(np.float32)
    state = {
        "left_arm":  arm_q[ARM_LEFT_SLICE],
        "right_arm": arm_q[ARM_RIGHT_SLICE],
        "left_hand":  read_hand_state(left_hand),
        "right_hand": read_hand_state(right_hand),
    }

    obs = {
        "video":    video,
        "state":    state,
        "language": {"annotation.human.task_description": task},
    }
    return _add_bt_dims(obs)


# ── check mode ────────────────────────────────────────────────────────────────

def check_observations(args):
    """
    Build one full observation dict (exactly as sent to the GR00T server) and
    print every field's key path, shape, dtype, and value range.
    Camera snapshots are saved to /tmp/check_<key>.png for visual inspection.
    Requires arm and image server; hands are optional.
    """
    import cv2

    # ── hands ─────────────────────────────────────────────────────────────────
    left_hand  = None
    right_hand = None
    if args.hands:
        logger.info("Connecting left hand...")
        left_hand  = connect_hand("left",  args.hand_speed_coeff, args.hand_current_coeff)
        logger.info("Connecting right hand...")
        right_hand = connect_hand("right", args.hand_speed_coeff, args.hand_current_coeff)
        time.sleep(2.0)

    # ── arm ───────────────────────────────────────────────────────────────────
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(0)
    from teleop.robot_control.robot_arm import H2_ArmController
    logger.info("Initialising H2_ArmController...")
    arm_ctrl = H2_ArmController(
        kp_low=args.kp_low, kp_wrist=args.kp_wrist,
        kd_low=args.kd_low, kd_wrist=args.kd_wrist,
    )
    logger.info("H2_ArmController ready.")

    # ── image client ──────────────────────────────────────────────────────────
    from teleimager.image_client import ImageClient
    logger.info(f"Connecting to image server at {args.img_server_ip} ...")
    img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
    cam_config = img_client.get_cam_config()
    logger.info(f"Camera config keys: {list(cam_config.keys())}")

    # ── diagnose camera config ────────────────────────────────────────────────
    logger.info("\n=== Camera config from image server ===")
    for cam_name, cfg in cam_config.items():
        logger.info(f"  {cam_name}: enable_zmq={cfg.get('enable_zmq')}  cfg={cfg}")

    # ── probe each camera directly before building obs ────────────────────────
    cam_name_to_key = {
        "head_camera":        "high",
        "left_wrist_camera":  "left_wrist_view",
        "right_wrist_camera": "right_wrist_view",
    }
    getter_map = {
        "high":             img_client.get_head_frame,
        "left_wrist_view":  img_client.get_left_wrist_frame,
        "right_wrist_view": img_client.get_right_wrist_frame,
    }
    # Wait for ZMQ subscriber threads to receive the first frame from each camera.
    # The ring buffer is empty immediately after subscribe() is called, so we
    # poll until bgr is not None or the timeout expires.
    _PROBE_TIMEOUT_S = 5.0
    _PROBE_INTERVAL_S = 0.1

    logger.info("\n=== Raw camera probe (waiting up to 5s per camera) ===")
    for cam_name, cfg in cam_config.items():
        key    = cam_name_to_key.get(cam_name)
        getter = getter_map.get(key) if key else None
        if getter is None:
            logger.warning(f"  [{cam_name}] no getter — unknown camera name")
            continue
        if not cfg.get("enable_zmq"):
            logger.warning(f"  [{cam_name}] enable_zmq=False — frame will be skipped")
            continue

        deadline = time.perf_counter() + _PROBE_TIMEOUT_S
        frame = None
        while time.perf_counter() < deadline:
            frame = getter()
            if frame is not None and frame.bgr is not None:
                break
            time.sleep(_PROBE_INTERVAL_S)

        if frame is None:
            logger.error(f"  [{key}] getter returned None after {_PROBE_TIMEOUT_S}s")
        elif frame.bgr is None:
            logger.error(f"  [{key}] frame.bgr still None after {_PROBE_TIMEOUT_S}s — "
                         "image server may not be publishing")
        else:
            img = frame.bgr
            logger.info(f"  [{key}] OK  shape={img.shape}  dtype={img.dtype}"
                        f"  min={img.min()}  max={img.max()}")

    # ── build full observation (cameras are now warmed up from the probe) ────
    obs = get_observations(img_client, arm_ctrl, left_hand, right_hand, cam_config, args.task)

    # ── print every field ─────────────────────────────────────────────────────
    def _print_nested(d: dict, prefix: str = ""):
        for k, v in d.items():
            path = f"{prefix}.{k}" if prefix else k
            if isinstance(v, dict):
                _print_nested(v, path)
            elif isinstance(v, np.ndarray):
                logger.info(
                    f"  {path:45s}  shape={str(v.shape):20s}  dtype={v.dtype}"
                    f"  min={v.min():.4f}  max={v.max():.4f}  mean={v.mean():.4f}"
                )
            else:
                logger.info(f"  {path:45s}  value={v!r}")

    logger.info("\n=== Observation fields (after batch/time dims added) ===")
    _print_nested(obs)

    if not obs.get("video"):
        logger.error("video dict is EMPTY — no camera frames received. "
                     "Check image server and enable_zmq settings above.")

    # ── save camera snapshots ─────────────────────────────────────────────────
    logger.info("\n=== Camera snapshots ===")
    for key, arr in obs.get("video", {}).items():
        # arr shape: (1, 1, H, W, 3) RGB uint8
        img_rgb = arr[0, 0]
        img_bgr = img_rgb[:, :, ::-1].copy()
        save_path = f"/tmp/check_{key}.png"
        cv2.imwrite(save_path, img_bgr)
        logger.info(f"  [{key}] saved → {save_path}")

    logger.info("\nCheck complete.")

    # ── cleanup to avoid segfault on exit ─────────────────────────────────────
    if args.hands:
        disconnect_hand(left_hand,  "left")
        disconnect_hand(right_hand, "right")


# ── main eval loop ─────────────────────────────────────────────────────────────

def eval_h2_groot(args):
    if args.check:
        check_observations(args)
        return
    # ── hands ─────────────────────────────────────────────────────────────────
    left_hand  = None
    right_hand = None
    if args.hands:
        logger.info("Connecting left hand...")
        left_hand  = connect_hand("left",  args.hand_speed_coeff, args.hand_current_coeff)
        logger.info("Connecting right hand...")
        right_hand = connect_hand("right", args.hand_speed_coeff, args.hand_current_coeff)
        logger.info("Waiting for hands to reach neutral position...")
        time.sleep(2.0)

    # ── arm ───────────────────────────────────────────────────────────────────
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(0)
    from teleop.robot_control.robot_arm import H2_ArmController
    logger.info("Initialising H2_ArmController...")
    arm_ctrl = H2_ArmController(
        kp_low=args.kp_low, kp_wrist=args.kp_wrist,
        kd_low=args.kd_low, kd_wrist=args.kd_wrist,
    )
    arm_ctrl.speed_gradual_max()
    logger.info("H2_ArmController ready.")

    # ── go home and hold 20s — gives Sharpa SDK time to fully start up ────────
    _go_home(arm_ctrl, left_hand, right_hand, hold_s=20)

    # ── image client ──────────────────────────────────────────────────────────
    from teleimager.image_client import ImageClient
    img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
    cam_config = img_client.get_cam_config()
    logger.info(f"Camera config: {list(cam_config.keys())}")

    # Trigger ZMQ subscriptions now so ring buffers are filled before inference.
    # get_head/wrist_frame() starts the background thread on first call; poll
    # until each camera returns a valid frame (typically one 33ms cycle at 30fps).
    _getters = [img_client.get_head_frame, img_client.get_left_wrist_frame,
                img_client.get_right_wrist_frame]
    logger.info("Warming up cameras...")
    deadline = time.perf_counter() + 5.0
    warmed = set()
    while len(warmed) < len(_getters) and time.perf_counter() < deadline:
        for i, getter in enumerate(_getters):
            if i in warmed:
                continue
            f = getter()
            if f is not None and f.bgr is not None:
                warmed.add(i)
        if len(warmed) < len(_getters):
            time.sleep(0.05)
    logger.info(f"Cameras ready ({len(warmed)}/{len(_getters)} with valid frames).")

    # ── GR00T policy client ───────────────────────────────────────────────────
    logger.info(f"Connecting to GR00T server at {args.policy_host}:{args.policy_port} ...")
    client = GR00TClient(host=args.policy_host, port=args.policy_port,
                         timeout_ms=args.timeout_ms)
    for attempt in range(30):
        if client.ping():
            logger.info("GR00T server: connected.")
            break
        logger.info(f"  waiting for server... ({attempt+1}/30)")
        time.sleep(2.0)
    else:
        raise RuntimeError("Could not reach GR00T server after 60s. Is it running?")

    # ── prompt user ───────────────────────────────────────────────────────────
    user_input = input("\nPress [Enter] to start inference (or 'q' to quit): ")
    if user_input.lower() == "q":
        return

    logger.info(f"Inference config: {args.frequency} Hz, "
                f"action_horizon={args.action_horizon}, steps_per_episode={args.steps}, task='{args.task}'")
    logger.info("Running: home(20s) → inference → home(20s) → inference → ...  Ctrl+C to stop.")

    dt       = 1.0 / args.frequency
    episode  = 0
    try:
        while True:
            episode += 1
            client.reset()
            logger.info(f"\n=== Episode {episode} ===")

            steps_executed = 0
            while steps_executed < args.steps:
                # ── get action chunk from GR00T ───────────────────────────────
                obs             = get_observations(img_client, arm_ctrl, left_hand, right_hand,
                                                   cam_config, args.task)
                action_chunk, _ = client.get_action(obs)

                # ── execute up to action_horizon steps, not past the step limit ─
                chunk_len = action_chunk["left_arm"].shape[1] if "left_arm" in action_chunk else args.action_horizon
                horizon   = min(args.action_horizon, chunk_len, args.steps - steps_executed)

                for t in range(horizon):
                    t0 = time.perf_counter()

                    left_arm     = action_chunk["left_arm"][0][t].astype(np.float64)
                    right_arm    = action_chunk["right_arm"][0][t].astype(np.float64)
                    arm_q_target = np.concatenate([left_arm, right_arm])

                    arm_ctrl.ctrl_dual_arm(arm_q_target, np.zeros(ARM_DOF, dtype=np.float64))

                    if args.hands:
                        if left_hand is not None and "left_hand" in action_chunk:
                            err = left_hand.set_joint_position(action_chunk["left_hand"][0][t].tolist(), False)
                            if err.code != 0:
                                logger.warning(f"left hand error t={t}: {err.message}")
                        if right_hand is not None and "right_hand" in action_chunk:
                            err = right_hand.set_joint_position(action_chunk["right_hand"][0][t].tolist(), False)
                            if err.code != 0:
                                logger.warning(f"right hand error t={t}: {err.message}")

                    elapsed = time.perf_counter() - t0
                    time.sleep(max(0.0, dt - elapsed))
                    steps_executed += 1

                logger.info(f"chunk done ({horizon} steps), total={steps_executed}/{args.steps}")

            logger.info(f"Episode {episode} complete ({args.steps} steps). Going home...")
            _go_home(arm_ctrl, left_hand, right_hand, hold_s=20)

    except KeyboardInterrupt:
        logger.info("Stopped by user.")
    finally:
        _go_home(arm_ctrl, left_hand, right_hand, hold_s=20)
        if args.hands:
            disconnect_hand(left_hand,  "left")
            disconnect_hand(right_hand, "right")
        logger.info("Done.")


def parse_args():
    ap = argparse.ArgumentParser(
        description="H2 + Sharpa real-robot eval using a remote GR00T policy server.")
    ap.add_argument("--check", action="store_true",
                    help="Check mode: verify camera inputs only, skip arm/hands/GR00T")
    ap.add_argument("--policy-host",   default="localhost",      help="GR00T server host")
    ap.add_argument("--policy-port",   type=int, default=5555,   help="GR00T server port")
    ap.add_argument("--timeout-ms",    type=int, default=15000,  help="ZMQ recv timeout ms")
    ap.add_argument("--task",          default="",               help="Language instruction for the policy")
    ap.add_argument("--img-server-ip", default="192.168.124.162", help="Image server IP")
    ap.add_argument("--frequency",     type=float, default=30.0, help="Control frequency Hz")
    ap.add_argument("--action-horizon", type=int, default=16,  # matches training delta_indices=[0..15]
                    help="Number of steps to execute per action chunk before re-querying")
    ap.add_argument("--steps", type=int, default=30,
                    help="Total number of control steps to run before stopping (default: 30)")

    g = ap.add_mutually_exclusive_group()
    g.add_argument("--hands",    dest="hands", action="store_true",  default=True)
    g.add_argument("--no-hands", dest="hands", action="store_false")

    # H2 PD gains
    ap.add_argument("--kp-low",   type=float, default=None)
    ap.add_argument("--kp-wrist", type=float, default=None)
    ap.add_argument("--kd-low",   type=float, default=None)
    ap.add_argument("--kd-wrist", type=float, default=None)

    # Sharpa tuning
    ap.add_argument("--hand-speed-coeff",   type=float, default=0.5)
    ap.add_argument("--hand-current-coeff", type=float, default=0.6)
    return ap.parse_args()


if __name__ == "__main__":
    eval_h2_groot(parse_args())
