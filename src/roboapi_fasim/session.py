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

        self.arm_cfg = arm_cfg
        joint_names = [name.strip() for name in arm_cfg.joint_names.split(",") if name.strip()]
        config = ROS2RobotInterfaceConfig.default_single_arm_ocs2_arm_controller(
            joint_states_topic=arm_cfg.joint_states_topic,
            current_pose_topic=arm_cfg.pose_topic,
            target_pose_topic=arm_cfg.target_topic,
            current_target_topic=arm_cfg.current_target_topic or None,
            joint_names=joint_names,
            gripper_joint_name=arm_cfg.gripper_joint_name,
            gripper_command_topic=arm_cfg.gripper_command_topic,
            namespace=arm_cfg.namespace,
            node_name="roboapi_fasim",
            left_gripper_controller_name=arm_cfg.gripper_controller_name,
        )
        # Install inverted 2F-85 limits only as a hint for arrival checks; the
        # gripper driver maps RoboAPI ratios itself because min must stay <= max.
        config.gripper_min_position = min(arm_cfg.gripper_closed, arm_cfg.gripper_open)
        config.gripper_max_position = max(arm_cfg.gripper_closed, arm_cfg.gripper_open)
        self.interface = ROS2RobotInterface(config)
        self.interface.connect()
        # The interface already spins robot_node on its own executor thread.
        self._control_ready = False

    def prepare_control(self) -> None:
        if self._control_ready:
            return
        self.interface.auto_switch_fsm_for_control("arm_pose")
        self._control_ready = True

    def close(self) -> None:
        try:
            self.interface.disconnect()
        except Exception:
            pass


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
