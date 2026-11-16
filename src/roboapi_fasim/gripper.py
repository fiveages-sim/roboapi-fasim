"""fasim single-DOF parallel gripper; opening 0=closed, 1=open."""

from __future__ import annotations

from dataclasses import dataclass
import time

from roboapi.hardware import ComponentOption, ComponentPlugin, GripperConfig


@dataclass
class FasimGripperConfig(GripperConfig):
    type: str = "fasim"
    joint_name: str = "gripper_joint"
    command_topic: str = "/gripper_joint/position_command"
    controller_name: str = "gripper_controller"
    joint_states_topic: str = "/joint_states"
    closed_position: float = 0.8
    open_position: float = 0.0
    namespace: str = ""
    full_travel_s: float = 1.0

    @property
    def connection_id(self) -> str:
        return f"{self.command_topic}|{self.joint_name}"

    def validate(self) -> None:
        super().validate()
        if not self.joint_name.strip() or not self.command_topic.strip():
            raise ValueError("gripper.joint_name and gripper.command_topic are required")
        if self.closed_position == self.open_position:
            raise ValueError("gripper.closed_position and gripper.open_position must differ")


def configure(option, setup):
    arm = getattr(getattr(setup, "config", None), "arm", None)
    joint_name = getattr(arm, "gripper_joint_name", "gripper_joint") if arm is not None else "gripper_joint"
    command_topic = (
        getattr(arm, "gripper_command_topic", "/gripper_joint/position_command")
        if arm is not None else "/gripper_joint/position_command"
    )
    joint_states_topic = (
        getattr(arm, "joint_states_topic", "/joint_states") if arm is not None else "/joint_states"
    )
    namespace = getattr(arm, "namespace", "") if arm is not None else ""
    closed = getattr(arm, "gripper_closed", 0.8) if arm is not None else 0.8
    opened = getattr(arm, "gripper_open", 0.0) if arm is not None else 0.0
    return option.config_type(
        joint_name=setup.ask("夹爪关节名", joint_name),
        command_topic=setup.ask("夹爪位置命令话题", command_topic),
        controller_name=setup.ask("夹爪控制器名", getattr(arm, "gripper_controller_name", "gripper_controller") if arm is not None else "gripper_controller"),
        joint_states_topic=setup.ask("关节状态话题", joint_states_topic),
        closed_position=float(setup.ask("闭合关节位置（2F-85 为 0.8）", str(closed))),
        open_position=float(setup.ask("张开关节位置（2F-85 为 0.0）", str(opened))),
        namespace=namespace,  # official setup.ask rejects a blank default
    )


def gripper_plugin():
    return ComponentPlugin(
        options=(ComponentOption("FaSim ROS 2 平行夹爪", FasimGripperConfig),),
        create=FasimGripper,
        configure=configure,
        setup_notes=(
            "RoboAPI 开度 0 为闭合、1 为张开。官方 2F-85 的 gripper_joint 是 0 张开、0.8 闭合，驱动会自动换算。",
        ),
    )


def _arm_cfg_from_gripper(cfg: FasimGripperConfig):
    from .arm import FasimArmConfig

    # Infer side from gripper topics when arm config is not shared yet.
    command = (cfg.command_topic or "").lower()
    arm_side = "right" if "right" in command else "left"
    return FasimArmConfig(
        arm_side=arm_side,
        joint_states_topic=cfg.joint_states_topic,
        gripper_joint_name=cfg.joint_name,
        gripper_command_topic=cfg.command_topic,
        gripper_controller_name=cfg.controller_name,
        gripper_closed=cfg.closed_position,
        gripper_open=cfg.open_position,
        namespace=cfg.namespace,
        # Keep pose topics coherent with the inferred side for session identity.
        pose_topic=f"/{arm_side}_current_pose",
        target_topic=f"/{arm_side}_target",
        current_target_topic=f"/{arm_side}_current_target",
    )


class FasimGripper:
    def __init__(self, cfg):
        self.cfg = cfg
        self._session = None
        self._error = None
        self._live = False
        self._last_ratio = None

    @property
    def live(self) -> bool:
        return bool(self._live and self._session is not None and self._error is None)

    @property
    def last_error(self) -> str | None:
        return self._error

    def connect(self):
        from .session import acquire_robot, named_joint, ratio_from_position, release_robot, wait_until

        if self._session is not None:
            return
        try:
            self._session = acquire_robot(_arm_cfg_from_gripper(self.cfg))
            wait_until(
                lambda: _try_named_joint(self._session.interface, self.cfg.joint_name) is not None,
                5.0,
                f"no {self.cfg.joint_name} in joint_states",
            )
            position = named_joint(self._session.interface, self.cfg.joint_name)
            self._last_ratio = ratio_from_position(
                position, self.cfg.closed_position, self.cfg.open_position
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

    def get_ratio(self):
        from .session import named_joint, ratio_from_position

        position = named_joint(self._require().interface, self.cfg.joint_name)
        return ratio_from_position(position, self.cfg.closed_position, self.cfg.open_position)

    def set_ratio(self, ratio):
        from .session import active_gripper_handler, position_from_ratio

        session = self._require()
        target = position_from_ratio(ratio, self.cfg.closed_position, self.cfg.open_position)
        active_gripper_handler(session).send_joint_positions(target)
        self._last_ratio = float(ratio)

    def hold(self):
        self.set_ratio(self.get_ratio())

    def move_blocking(self, ratio, should_stop=None):
        self.set_ratio(ratio)
        deadline = time.monotonic() + float(self.cfg.full_travel_s) + 0.5
        while time.monotonic() < deadline:
            if should_stop is not None and should_stop():
                self.hold()
                return
            if abs(self.get_ratio() - float(ratio)) <= 0.02:
                return
            time.sleep(0.02)
        raise TimeoutError(f"gripper did not reach ratio {ratio}")

    def recover(self):
        self._error = None
        self._live = self._session is not None

    def _require(self):
        if self._session is None:
            raise RuntimeError("fasim gripper is not connected")
        return self._session


def _try_named_joint(interface, name: str):
    from .session import named_joint

    try:
        return named_joint(interface, name)
    except Exception:
        return None
