"""fasim arm registration. Keep SDK imports out of this module."""

from __future__ import annotations

from dataclasses import dataclass

from roboapi.hardware import ArmConfig, ArmPlugin, ComponentOption

DEFAULT_JOINT_NAMES = (
    "shoulder_pan_joint,shoulder_lift_joint,elbow_joint,"
    "wrist_1_joint,wrist_2_joint,wrist_3_joint"
)


@dataclass
class FasimArmConfig(ArmConfig):
    type: str = "fasim"
    pose_topic: str = "/left_current_pose"
    target_topic: str = "/left_target"
    current_target_topic: str = "/left_current_target"
    joint_states_topic: str = "/joint_states"
    base_frame: str = "base_link"
    joint_names: str = DEFAULT_JOINT_NAMES
    namespace: str = ""
    gripper_joint_name: str = "gripper_joint"
    gripper_command_topic: str = "/gripper_joint/position_command"
    gripper_controller_name: str = "gripper_controller"
    gripper_closed: float = 0.8
    gripper_open: float = 0.0

    @property
    def connection_id(self) -> str:
        return f"{self.pose_topic}|{self.base_frame}|tool0"

    def validate(self) -> None:
        super().validate()
        for name in ("pose_topic", "target_topic", "joint_states_topic", "base_frame"):
            if not str(getattr(self, name) or "").strip():
                raise ValueError(f"arm.{name} is required")


def configure(option, setup):
    return option.config_type(
        pose_topic=setup.ask("末端位姿话题（法兰中心 / tool0）", "/left_current_pose"),
        target_topic=setup.ask("末端目标话题", "/left_target"),
        current_target_topic=setup.ask("当前目标话题（可空）", "/left_current_target"),
        joint_states_topic=setup.ask("关节状态话题", "/joint_states"),
        base_frame=setup.ask("原生基坐标系", "base_link"),
        joint_names=setup.ask("手臂关节名（逗号分隔）", DEFAULT_JOINT_NAMES),
        namespace="",  # official setup.ask rejects a blank default
        gripper_joint_name=setup.ask("夹爪关节名", "gripper_joint"),
        gripper_command_topic=setup.ask("夹爪位置命令话题", "/gripper_joint/position_command"),
        gripper_controller_name=setup.ask("夹爪控制器名", "gripper_controller"),
    )


def arm_plugin():
    return ArmPlugin(
        options=(ComponentOption("FaSim ROS 2 机械臂", FasimArmConfig),),
        create=FasimArm,
        create_feedback=FasimFeedback,
        configure=configure,
        setup_notes=(
            "控制点是 tool0（法兰中心、Z 向外）。请确认运控发布的位姿是 tool0，而不是 ROS-Industrial flange。",
        ),
    )


class FasimArm:
    def __init__(self, cfg, tool):
        self.cfg = cfg
        self.tool = tool
        self._session = None
        self._error = None
        self._live = False

    @property
    def live(self) -> bool:
        return bool(self._live and self._session is not None and self._error is None)

    @property
    def last_error(self) -> str | None:
        return self._error

    def connect(self, *, calibration=False):
        from .session import acquire_robot, release_robot, wait_until

        if self._session is not None:
            return
        try:
            self._session = acquire_robot(self.cfg)
            if not calibration:
                self._session.prepare_control()
            wait_until(
                lambda: self._session.interface.left_arm_handler.get_pose() is not None,
                5.0,
                f"no pose on {self.cfg.pose_topic}",
            )
            self._error = None
            self._live = True
        except Exception as exc:
            self._error = str(exc)
            self._live = False
            if self._session is not None:
                release_robot()
                self._session = None
            raise

    def disconnect(self):
        from .session import release_robot

        self._live = False
        if self._session is not None:
            self._session = None
            release_robot()
        self._error = None

    def get_joints(self):
        from .session import joint_positions

        names = [name.strip() for name in self.cfg.joint_names.split(",") if name.strip()]
        return joint_positions(self._require().interface, names)

    def get_eef_pose(self):
        from .session import pose_to_wxyz

        pose = self._require().interface.left_arm_handler.get_pose()
        if pose is None:
            raise RuntimeError(f"no measured pose on {self.cfg.pose_topic}")
        return pose_to_wxyz(pose)

    def servo_pose(self, pose7, twist=None, *, position_only=False):
        del twist, position_only
        from .session import pose7_to_msg

        session = self._require()
        session.prepare_control()
        # ocs2_arm_controller follows unstamped /left_target (Pose in base_link).
        session.interface.left_arm_handler.send_target(pose7_to_msg(pose7))

    def hold(self):
        self.servo_pose(self.get_eef_pose())

    def validate_calibration(self, device):
        from .session import device_identity, require_same_device

        require_same_device(device, device_identity(self.cfg))

    def recover(self, should_stop=None):
        del should_stop
        session = self._require()
        session._control_ready = False
        session.prepare_control()
        self._error = None
        self._live = True

    def _require(self):
        if self._session is None:
            raise RuntimeError("fasim arm is not connected")
        return self._session


class FasimFeedback:
    """Measured flange feedback, available without taking motion control."""

    def __init__(self, cfg):
        self.cfg = cfg
        self._session = None
        self._error = None
        self._live = False
        self._last_pose = None
        self._stamp = 0.0

    @property
    def live(self) -> bool:
        return bool(self._live and self._session is not None and self._error is None)

    @property
    def last_error(self) -> str | None:
        return self._error

    def connect(self):
        from .session import acquire_robot, release_robot, pose_to_wxyz, wait_until

        if self._session is not None:
            return
        try:
            self._session = acquire_robot(self.cfg)
            wait_until(
                lambda: self._session.interface.left_arm_handler.get_pose() is not None,
                5.0,
                f"no pose on {self.cfg.pose_topic}",
            )
            pose = pose_to_wxyz(self._session.interface.left_arm_handler.get_pose())
            self._last_pose = pose
            self._stamp = 0.0
            self._error = None
            self._live = True
        except Exception as exc:
            self._error = str(exc)
            self._live = False
            if self._session is not None:
                release_robot()
                self._session = None
            raise

    def disconnect(self):
        from .session import release_robot

        self._live = False
        if self._session is not None:
            self._session = None
            release_robot()
        self._error = None
        self._last_pose = None
        self._stamp = 0.0

    def snapshot(self):
        from .session import device_identity, pose_to_wxyz

        if self._session is None:
            raise RuntimeError("fasim feedback is not connected")
        pose_msg = self._session.interface.left_arm_handler.get_pose()
        if pose_msg is None:
            raise RuntimeError(f"no measured pose on {self.cfg.pose_topic}")
        pose = pose_to_wxyz(pose_msg)
        if self._last_pose is None or not _same_pose(self._last_pose, pose):
            import time

            self._last_pose = pose
            self._stamp = time.monotonic()
        return {
            "pose": pose.tolist(),
            "timestamp": self._stamp,
            "device": device_identity(self.cfg),
            "has_errors": False,
        }

    def validate_device(self, device):
        from .session import device_identity, require_same_device

        require_same_device(device, device_identity(self.cfg))


def _same_pose(left, right, atol=1e-9) -> bool:
    import numpy as np

    return bool(np.allclose(left, right, rtol=0.0, atol=atol))
