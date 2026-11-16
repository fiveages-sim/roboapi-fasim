"""Shared ROS 2 session for fasim RoboAPI drivers.

Registration modules must import this only from connect() and other explicit
device methods, never at module import time.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

_LOCK = threading.Lock()
_SPINNER: "_Spinner | None" = None
_ROBOT: "_RobotSession | None" = None
_ROBOT_USERS = 0

CONTROL_POINT = "flange_centre"
FLANGE_FRAME = "tool0"
DEFAULT_ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class _Spinner:
    def __init__(self) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        if not rclpy.ok():
            rclpy.init()
        self.executor = MultiThreadedExecutor()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="roboapi-fasim-spin", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            self.executor.spin_once(timeout_sec=0.05)

    def add(self, node) -> None:
        self.executor.add_node(node)

    def remove(self, node) -> None:
        try:
            self.executor.remove_node(node)
        except Exception:
            pass

    def shutdown(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)


def spinner() -> _Spinner:
    global _SPINNER
    with _LOCK:
        if _SPINNER is None:
            _SPINNER = _Spinner()
        return _SPINNER


class _RobotSession:
    def __init__(self, arm_cfg) -> None:
        from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

        from .probe import normalize_arm_side

        self.arm_cfg = arm_cfg
        # Arm Cartesian: map the selected side into primary (left_*) slots.
        # Gripper: keep real left/right fields — connect() auto-detect overwrites
        # gripper_command_topic back to /left_* when both sides exist, so right
        # control must use right_gripper_handler rather than remapping into left_*.
        self.arm_side = normalize_arm_side(getattr(arm_cfg, "arm_side", None))
        joint_names = [name.strip() for name in arm_cfg.joint_names.split(",") if name.strip()]
        gripper_kwargs: dict[str, object] = {}
        if self.arm_side == "right":
            gripper_kwargs["right_gripper_command_topic"] = arm_cfg.gripper_command_topic
            gripper_kwargs["right_gripper_controller_name"] = arm_cfg.gripper_controller_name
        else:
            gripper_kwargs["gripper_command_topic"] = arm_cfg.gripper_command_topic
            gripper_kwargs["left_gripper_controller_name"] = arm_cfg.gripper_controller_name
        config = ROS2RobotInterfaceConfig.default_single_arm_ocs2_arm_controller(
            joint_states_topic=arm_cfg.joint_states_topic,
            current_pose_topic=arm_cfg.pose_topic,
            target_pose_topic=arm_cfg.target_topic,
            current_target_topic=arm_cfg.current_target_topic or None,
            joint_names=joint_names,
            gripper_joint_name=arm_cfg.gripper_joint_name,
            namespace=arm_cfg.namespace,
            node_name=f"roboapi_fasim_{self.arm_side}",
            **gripper_kwargs,
        )
        # Install inverted 2F-85 limits only as a hint for arrival checks; the
        # gripper driver maps RoboAPI ratios itself because min must stay <= max.
        config.gripper_min_position = min(arm_cfg.gripper_closed, arm_cfg.gripper_open)
        config.gripper_max_position = max(arm_cfg.gripper_closed, arm_cfg.gripper_open)
        self.interface = ROS2RobotInterface(config)
        self.interface.connect()
        self._ensure_right_gripper()
        # The interface already spins robot_node on its own executor thread.
        self._control_ready = False

    def _ensure_right_gripper(self) -> None:
        """Make sure right gripper handler publishes to the configured command topic."""
        if self.arm_side != "right":
            return
        from std_msgs.msg import Float64
        from ros2_robot_interface.handler import GripperHandler, GripperType

        arm_cfg = self.arm_cfg
        self.interface.config.right_gripper_command_topic = arm_cfg.gripper_command_topic
        self.interface.config.right_gripper_controller_name = arm_cfg.gripper_controller_name
        right = self.interface.right_gripper_handler
        if right is None:
            self.interface.right_gripper_handler = GripperHandler(
                self.interface.robot_node,
                GripperType.RIGHT,
                self.interface.config,
            )
            self.interface.right_gripper_handler.initialize()
            return
        if right.command_topic != arm_cfg.gripper_command_topic or right.command_pub is None:
            right.command_topic = arm_cfg.gripper_command_topic
            if arm_cfg.gripper_command_topic:
                right.command_pub = self.interface.robot_node.create_publisher(
                    Float64, arm_cfg.gripper_command_topic, 10
                )

    def prepare_control(self) -> None:
        if self._control_ready:
            return
        self._prepare_wbc_selected_arm()
        self.interface.auto_switch_fsm_for_control("arm_pose")
        self._control_ready = True

    def _prepare_wbc_selected_arm(self) -> None:
        """In WBC dual-arm mode, enable the selected arm and disable the other.

        RoboAPI drives one arm only; leaving the opposite arm ENABLED lets WBC
        keep tracking it and fight single-arm Cartesian targets.
        """
        interface = self.interface
        if not getattr(interface, "is_wbc", False):
            return
        if not getattr(interface, "mode_command_pub", None):
            return

        if self.arm_side == "right":
            enable_cmd, disable_cmd = "RIGHT_ARM_ENABLE", "LEFT_ARM_DISABLE"
        else:
            enable_cmd, disable_cmd = "LEFT_ARM_ENABLE", "RIGHT_ARM_DISABLE"

        commands = ("ARMS_INDEPENDENT", enable_cmd, disable_cmd)
        for command in commands:
            try:
                interface.send_mode_command(command)
            except Exception:
                # Non-WBC stacks or missing mode_command should not block control.
                return
        wait = getattr(interface, "wait_until_mode_commands_applied", None)
        if callable(wait):
            try:
                wait(commands, timeout=2.0)
            except Exception:
                pass

    def close(self) -> None:
        try:
            self.interface.disconnect()
        except Exception:
            pass


def active_arm_handler(session: "_RobotSession"):
    """Selected-side arm handler. Cartesian topics are remapped onto left_* slots."""
    handler = session.interface.left_arm_handler
    if handler is None:
        raise RuntimeError("arm handler is not available")
    return handler


def active_gripper_handler(session: "_RobotSession"):
    """Selected-side gripper handler (real left/right, not remapped)."""
    if session.arm_side == "right":
        handler = session.interface.right_gripper_handler
        if handler is None:
            raise RuntimeError(
                "right gripper handler is not available; check gripper.command_topic / controller_name"
            )
        return handler
    handler = session.interface.left_gripper_handler
    if handler is None:
        raise RuntimeError(
            "left gripper handler is not available; check gripper.command_topic / controller_name"
        )
    return handler


def acquire_robot(arm_cfg) -> _RobotSession:
    global _ROBOT, _ROBOT_USERS
    with _LOCK:
        if _ROBOT is None:
            _ROBOT = _RobotSession(arm_cfg)
        _ROBOT_USERS += 1
        return _ROBOT


def release_robot() -> None:
    global _ROBOT, _ROBOT_USERS
    with _LOCK:
        _ROBOT_USERS = max(0, _ROBOT_USERS - 1)
        if _ROBOT_USERS == 0 and _ROBOT is not None:
            session = _ROBOT
            _ROBOT = None
            session.close()


def wait_until(predicate, timeout_s: float, message: str):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise TimeoutError(message)


def pose_to_wxyz(pose) -> np.ndarray:
    return np.array(
        [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ],
        dtype=float,
    )


def pose7_to_msg(pose7: np.ndarray):
    from geometry_msgs.msg import Pose

    values = np.asarray(pose7, dtype=float).reshape(7)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(values[0]), float(values[1]), float(values[2]))
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = (
        float(values[3]),
        float(values[4]),
        float(values[5]),
        float(values[6]),
    )
    return pose


def device_identity(arm_cfg, extra: dict[str, Any] | None = None) -> dict[str, Any]:
    identity = {
        "connection_id": arm_cfg.connection_id,
        "control_point": CONTROL_POINT,
        "flange_frame": FLANGE_FRAME,
        "arm_side": getattr(arm_cfg, "arm_side", "left"),
        "base_frame": arm_cfg.base_frame,
        "pose_topic": arm_cfg.pose_topic,
        "joint_states_topic": arm_cfg.joint_states_topic,
    }
    if extra:
        identity.update(extra)
    return identity


def require_same_device(expected: dict, actual: dict) -> None:
    keys = ("connection_id", "control_point", "flange_frame", "base_frame", "pose_topic")
    mismatches = [key for key in keys if expected.get(key) != actual.get(key)]
    if mismatches:
        raise ValueError(
            "control connection does not match the calibrated device: "
            + ", ".join(f"{key}: {expected.get(key)!r} != {actual.get(key)!r}" for key in mismatches)
        )


def joint_positions(interface, names: list[str]) -> np.ndarray:
    state = interface.get_joint_state(categorized=False)
    if not state:
        raise RuntimeError("no /joint_states sample yet")
    received = list(state.get("name") or state.get("names") or [])
    positions = list(state.get("position") or state.get("positions") or [])
    lookup = {name: float(value) for name, value in zip(received, positions)}
    if names:
        missing = [name for name in names if name not in lookup]
        if missing:
            raise RuntimeError(f"joint_states missing {missing}")
        return np.array([lookup[name] for name in names], dtype=float)
    return np.array(positions, dtype=float)


def named_joint(interface, name: str) -> float:
    values = joint_positions(interface, [name])
    return float(values[0])


def ratio_from_position(position: float, closed: float, opened: float) -> float:
    span = opened - closed
    if abs(span) < 1e-9:
        raise ValueError("gripper_open and gripper_closed must differ")
    return float(np.clip((position - closed) / span, 0.0, 1.0))


def position_from_ratio(ratio: float, closed: float, opened: float) -> float:
    ratio = float(np.clip(ratio, 0.0, 1.0))
    return closed + ratio * (opened - closed)
