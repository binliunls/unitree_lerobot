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

Observation keys sent to the server (single-eye head mode, default):
    video.high              (H, W, 3) uint8   ← chosen head eye (left or right)
    video.left_wrist_view   (H, W, 3) uint8
    video.right_wrist_view  (H, W, 3) uint8
    state.left_arm          (7,)  float32
    state.right_arm         (7,)  float32
    state.left_hand         (22,) float32
    state.right_hand        (22,) float32
    language.annotation.human.task_description  str

Stereo head mode (`--head-eye both`):
    video.left_eye_view     (H, W, 3) uint8
    video.right_eye_view    (H, W, 3) uint8
    (plus wrists and state as above)

Action chunk returned by the server:
    left_arm   (B, T, 7)
    right_arm  (B, T, 7)
    left_hand  (B, T, 22)
    right_hand (B, T, 22)

Image source: ROS 2 (CycloneDDS) on the four sensing_h2_ros_gstreamer topics,
or the legacy ZMQ teleimager server via `--image-source zmq`.

Hand control: through the C++ sharpa_dds_bridge on Thor (default), or the
direct Sharpa SDK on the local machine via `--hands-source sdk`.

Usage:
    # Standing replay, ROS cameras, DDS bridge for hands:
    python unitree_lerobot/eval_robot/eval_h2_groot.py \\
        --policy-host localhost --policy-port 5555 \\
        --task "pick up the apple" \\
        --motion

    # Right-eye only as video.high:
    python unitree_lerobot/eval_robot/eval_h2_groot.py \\
        --policy-host localhost --task "..." --head-eye right

    # Stereo head (video.left_eye_view + video.right_eye_view):
    python unitree_lerobot/eval_robot/eval_h2_groot.py \\
        --policy-host localhost --task "..." --head-eye both

    # Arm only (no hands), legacy ZMQ image server:
    python unitree_lerobot/eval_robot/eval_h2_groot.py \\
        --policy-host localhost --no-hands \\
        --image-source zmq --img-server-ip 192.168.124.162
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
_SHARPA_SDK = os.environ.get("SHARPA_SDK_PATH", "/usr/lib/sharpa-wave-sdk/python")

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
    """Read 22 joint angles (radians) from either the SDK SharpaWave or the DDS client."""
    if hand is None:
        return np.zeros(SHARPA_DOF, dtype=np.float32)
    try:
        err, angles_deg = hand.get_joint_position_degree()
        # SDK returns an Error obj with .code; DDS client returns a plain int.
        code = getattr(err, "code", err)
        if code != 0:
            return np.zeros(SHARPA_DOF, dtype=np.float32)
        return np.array([math.radians(a) for a in angles_deg[:SHARPA_DOF]], dtype=np.float32)
    except Exception:
        return np.zeros(SHARPA_DOF, dtype=np.float32)


# ── Sharpa hand helpers — DDS bridge path ────────────────────────────────────

def connect_hand_dds(side: str):
    """Construct a SharpaHandDDSClient that talks to the C++ bridge on Thor."""
    from teleop.robot_control.robot_hand_sharpa_dds import SharpaHandDDSClient
    logger.info(f"[{side}] creating DDS client (bridge on Thor must be running)...")
    client = SharpaHandDDSClient(side)
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


# ── camera client factory ────────────────────────────────────────────────────

def _make_image_client(args):
    """Build a camera client (ROS or ZMQ) and return (client, cam_config)."""
    if args.image_source == "ros":
        from teleop.utils.ros_image_client import ROSImageClient
        client = ROSImageClient(warmup_timeout=args.cam_warmup_s)
        cam_config = client.get_cam_config()
        logger.info("Camera client: ROS 2 (CycloneDDS)")
    else:
        from teleimager.image_client import ImageClient
        client = ImageClient(host=args.img_server_ip, request_bgr=True)
        cam_config = client.get_cam_config()
        logger.info(f"Camera client: ZMQ teleimager at {args.img_server_ip}")
    return client, cam_config


def _warm_cameras(img_client, timeout_s: float = 5.0):
    """Poll all four getters until each returns a non-None bgr or timeout."""
    getters = [
        ("head",        img_client.get_head_frame),
        ("left_wrist",  img_client.get_left_wrist_frame),
        ("right_wrist", img_client.get_right_wrist_frame),
    ]
    deadline = time.perf_counter() + timeout_s
    warmed = set()
    while len(warmed) < len(getters) and time.perf_counter() < deadline:
        for name, getter in getters:
            if name in warmed:
                continue
            f = getter()
            if f is not None and getattr(f, "bgr", None) is not None:
                warmed.add(name)
        if len(warmed) < len(getters):
            time.sleep(0.05)
    logger.info(f"Cameras warmed: {sorted(warmed)} ({len(warmed)}/{len(getters)})")


# ── hand setup / teardown ────────────────────────────────────────────────────

def _setup_hands(args):
    """Connect both hands via the chosen source. Returns (left, right)."""
    if not args.hands:
        return None, None

    if args.hands_source == "dds":
        logger.info("Connecting hands via DDS bridge...")
        left  = connect_hand_dds("left")
        right = connect_hand_dds("right")
    else:
        logger.info("Connecting hands via direct Sharpa SDK...")
        left  = connect_hand("left",  args.hand_speed_coeff, args.hand_current_coeff)
        right = connect_hand("right", args.hand_speed_coeff, args.hand_current_coeff)
    logger.info("Waiting for hands to reach neutral...")
    time.sleep(2.0)
    return left, right


def _teardown_hands(args, left_hand, right_hand):
    if not args.hands:
        return
    if args.hands_source == "dds":
        disconnect_hand_dds(left_hand,  "left")
        disconnect_hand_dds(right_hand, "right")
    else:
        disconnect_hand(left_hand,  "left")
        disconnect_hand(right_hand, "right")


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


def _split_head_image(stitched: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """ROSImageClient.get_head_frame() returns a (H, 2W, 3) horizontally-stitched
    stereo image. Split it back into (left_eye, right_eye)."""
    if stitched is None:
        return None, None
    w = stitched.shape[1] // 2
    return stitched[:, :w, :], stitched[:, w:, :]


def get_observations(img_client, arm_ctrl, left_hand, right_hand,
                     cam_config, task: str, head_eye: str = "left",
                     image_source: str = "ros") -> dict:
    """Build the nested GR00T observation dict (before adding batch/time dims)."""
    video: dict[str, np.ndarray] = {}

    if image_source == "ros":
        head_frame  = img_client.get_head_frame()
        l_wrist     = img_client.get_left_wrist_frame()
        r_wrist     = img_client.get_right_wrist_frame()

        if head_frame is not None and head_frame.bgr is not None:
            left_eye, right_eye = _split_head_image(head_frame.bgr)
            if head_eye == "left":
                video["high"] = left_eye[:, :, ::-1].copy()
            elif head_eye == "right":
                video["high"] = right_eye[:, :, ::-1].copy()
            else:  # both
                video["left_eye_view"]  = left_eye[:, :, ::-1].copy()
                video["right_eye_view"] = right_eye[:, :, ::-1].copy()
        if l_wrist is not None and l_wrist.bgr is not None:
            video["left_wrist_view"]  = l_wrist.bgr[:, :, ::-1].copy()
        if r_wrist is not None and r_wrist.bgr is not None:
            video["right_wrist_view"] = r_wrist.bgr[:, :, ::-1].copy()
    else:  # legacy ZMQ teleimager
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
        for cam_name, cfg in cam_config.items():
            if not cfg.get("enable_zmq"):
                continue
            key    = cam_name_to_key.get(cam_name)
            getter = getter_map.get(key) if key else None
            if getter is None:
                continue
            frame = getter()
            if frame is not None and frame.bgr is not None:
                video[key] = frame.bgr[:, :, ::-1].copy()

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
    """
    import cv2

    # DDS factory: needed for arm and for DDS-bridge hand client.
    need_dds = args.arm or (args.hands and args.hands_source == "dds") \
        or args.image_source == "ros"  # ROSImageClient uses CycloneDDS too
    if args.arm or (args.hands and args.hands_source == "dds"):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(args.dds_domain)
        logger.info(f"DDS ChannelFactory initialised on domain {args.dds_domain}.")

    left_hand, right_hand = _setup_hands(args)

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
        logger.info("H2_ArmController ready.")

    img_client, cam_config = _make_image_client(args)
    logger.info(f"Camera config keys: {list(cam_config.keys())}")
    _warm_cameras(img_client, timeout_s=args.cam_warmup_s)

    obs = get_observations(img_client, arm_ctrl, left_hand, right_hand,
                           cam_config, args.task,
                           head_eye=args.head_eye,
                           image_source=args.image_source)

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
        logger.error("video dict is EMPTY — no camera frames received.")

    logger.info("\n=== Camera snapshots ===")
    for key, arr in obs.get("video", {}).items():
        img_rgb = arr[0, 0]   # (1, 1, H, W, 3) → (H, W, 3) RGB
        img_bgr = img_rgb[:, :, ::-1].copy()
        save_path = f"/tmp/check_{key}.png"
        cv2.imwrite(save_path, img_bgr)
        logger.info(f"  [{key}] saved → {save_path}")

    logger.info("\nCheck complete.")
    _teardown_hands(args, left_hand, right_hand)
    try:
        img_client.close()
    except Exception:
        pass


# ── main eval loop ─────────────────────────────────────────────────────────────

def eval_h2_groot(args):
    if args.check:
        check_observations(args)
        return

    # DDS factory must be ready before any DDS publisher/subscriber is created
    # (H2_ArmController and SharpaHandDDSClient both need this).
    if args.arm or (args.hands and args.hands_source == "dds"):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(args.dds_domain)
        logger.info(f"DDS ChannelFactory initialised on domain {args.dds_domain}.")

    left_hand, right_hand = _setup_hands(args)

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

    _go_home(arm_ctrl, left_hand, right_hand, hold_s=args.hold_s)

    img_client, cam_config = _make_image_client(args)
    logger.info(f"Camera config: {list(cam_config.keys())}")
    _warm_cameras(img_client, timeout_s=args.cam_warmup_s)

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
                obs             = get_observations(img_client, arm_ctrl, left_hand, right_hand,
                                                   cam_config, args.task,
                                                   head_eye=args.head_eye,
                                                   image_source=args.image_source)
                action_chunk, _ = client.get_action(obs)

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
                            # DDS client returns None; SDK returns Error obj.
                            if err is not None and getattr(err, "code", 0) != 0:
                                logger.warning(f"left hand error t={t}: {err.message}")
                        if right_hand is not None and "right_hand" in action_chunk:
                            err = right_hand.set_joint_position(action_chunk["right_hand"][0][t].tolist(), False)
                            if err is not None and getattr(err, "code", 0) != 0:
                                logger.warning(f"right hand error t={t}: {err.message}")

                    elapsed = time.perf_counter() - t0
                    time.sleep(max(0.0, dt - elapsed))
                    steps_executed += 1

                logger.info(f"chunk done ({horizon} steps), total={steps_executed}/{args.steps}")

            logger.info(f"Episode {episode} complete ({args.steps} steps). Going home...")
            _go_home(arm_ctrl, left_hand, right_hand, hold_s=args.hold_s)

    except KeyboardInterrupt:
        logger.info("Stopped by user.")
    finally:
        _go_home(arm_ctrl, left_hand, right_hand, hold_s=args.hold_s)
        _teardown_hands(args, left_hand, right_hand)
        try:
            img_client.close()
        except Exception:
            pass
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

    # Hand and arm enable/disable
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--hands",    dest="hands", action="store_true",  default=True)
    g.add_argument("--no-hands", dest="hands", action="store_false")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--arm",      dest="arm",   action="store_true",  default=True)
    g.add_argument("--no-arm",   dest="arm",   action="store_false")

    # Image source: ROS 2 (default, four CycloneDDS topics) or legacy ZMQ teleimager.
    ap.add_argument("--image-source", choices=["ros", "zmq"], default="ros",
                    help="Camera source. 'ros' = 4 sensor_msgs/Image topics published by "
                         "sensing_h2_ros_gstreamer; 'zmq' = legacy teleimager server. "
                         "Default: ros.")
    ap.add_argument("--head-eye", choices=["left", "right", "both"], default="left",
                    help="Which head eye(s) to feed to the policy. 'left' or 'right' map to "
                         "video.high; 'both' produces video.left_eye_view + video.right_eye_view. "
                         "Default: left.")
    ap.add_argument("--cam-warmup-s", type=float, default=5.0,
                    help="How long to wait for the first frame from each camera (default: 5s).")

    # Sharpa hands source: through the C++ DDS bridge on Thor or direct SDK locally.
    ap.add_argument("--hands-source", choices=["dds", "sdk"], default="dds",
                    help="How to drive Sharpa hands. 'dds' = publish HandCmd_ to "
                         "rt/sharpa/{left,right}/cmd via sharpa_dds_bridge on Thor. "
                         "'sdk' = direct Sharpa SDK locally (bridge must be stopped). "
                         "Default: dds.")
    ap.add_argument("--dds-domain", type=int, default=0,
                    help="DDS domain ID for the arm controller and DDS-bridge hand client.")

    # H2 motion mode for standing replay (publish to rt/arm_sdk so WBC yields arms).
    ap.add_argument("--motion", action="store_true",
                    help="Enable H2 motion mode (publish on rt/arm_sdk with handover weight=1). "
                         "Required when the robot is standing on its legs.")
    ap.add_argument("--head-pitch-home", type=float, default=0.6, metavar="RAD",
                    help="H2 head pitch held during inference (radians). "
                         "Range -0.523 (up) to 0.837 (down). Default: 0.6.")

    # Hold timing
    ap.add_argument("--hold-s", type=int, default=20,
                    help="Seconds to hold at home before/between/after inference. Default: 20.")

    # H2 PD gains
    ap.add_argument("--kp-low",   type=float, default=None)
    ap.add_argument("--kp-wrist", type=float, default=None)
    ap.add_argument("--kd-low",   type=float, default=None)
    ap.add_argument("--kd-wrist", type=float, default=None)

    # Sharpa SDK tuning (only used when --hands-source sdk)
    ap.add_argument("--hand-speed-coeff",   type=float, default=0.5)
    ap.add_argument("--hand-current-coeff", type=float, default=0.6)
    return ap.parse_args()


if __name__ == "__main__":
    eval_h2_groot(parse_args())
