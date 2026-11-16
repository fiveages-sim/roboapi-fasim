"""ROS 2 graph probing helpers for Fasim RoboAPI setup defaults.

Uses ``ros2_robot_interface`` discovery when available; falls back silently
so ``roboapi init`` still works offline.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Iterable, List, Optional, Sequence, Tuple

TopicsAndTypes = List[Tuple[str, List[str]]]

DEFAULT_ARM_SIDE = "left"
DEFAULT_POSE_TOPIC = "/left_current_pose"
DEFAULT_TARGET_TOPIC = "/left_target"
DEFAULT_CURRENT_TARGET_TOPIC = "/left_current_target"
DEFAULT_JOINT_STATES_TOPIC = "/joint_states"
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_JOINT_NAMES = (
    "shoulder_pan_joint,shoulder_lift_joint,elbow_joint,"
    "wrist_1_joint,wrist_2_joint,wrist_3_joint"
)
DEFAULT_GRIPPER_JOINT = "gripper_joint"
DEFAULT_GRIPPER_COMMAND = "/gripper_joint/position_command"
DEFAULT_GRIPPER_CONTROLLER = "gripper_controller"
DEFAULT_CAMERA_TOPIC = "/wrist_camera/rgb"

# Per-side topic presets. Left also accepts unprefixed single-arm names.
_SIDE_PRESETS = {
    "left": {
        "pose": ("/left_current_pose", "/current_pose"),
        "target": ("/left_target", "/target"),
        "current_target": ("/left_current_target", "/current_target"),
        "gripper_command": (
            "/left_gripper_joint/position_command",
            "/gripper_joint/position_command",
        ),
        "gripper_controller": ("left_gripper_controller", "gripper_controller"),
        "gripper_joint": ("left_gripper_joint", "gripper_joint"),
    },
    "right": {
        "pose": ("/right_current_pose",),
        "target": ("/right_target",),
        "current_target": ("/right_current_target",),
        "gripper_command": ("/right_gripper_joint/position_command",),
        "gripper_controller": ("right_gripper_controller",),
        "gripper_joint": ("right_gripper_joint",),
    },
}

_JOINT_STATES_CANDIDATES = (
    "/joint_states",
)
_GRIPPER_COMMAND_SUFFIXES = (
    "/position_command",
)
_IMAGE_TYPE_MARKERS = (
    "sensor_msgs/msg/Image",
    "sensor_msgs/Image",
)
_CAMERA_NAME_HINTS = (
    "wrist",
    "acepro",
    "action6",
    "rgb",
    "camera",
    "hand",
)


@dataclass
class ArmProbeHints:
    arm_side: str = DEFAULT_ARM_SIDE
    available_sides: List[str] = field(default_factory=lambda: [DEFAULT_ARM_SIDE])
    pose_topic: str = DEFAULT_POSE_TOPIC
    target_topic: str = DEFAULT_TARGET_TOPIC
    current_target_topic: str = DEFAULT_CURRENT_TARGET_TOPIC
    joint_states_topic: str = DEFAULT_JOINT_STATES_TOPIC
    base_frame: str = DEFAULT_BASE_FRAME
    joint_names: str = DEFAULT_JOINT_NAMES
    gripper_joint_name: str = DEFAULT_GRIPPER_JOINT
    gripper_command_topic: str = DEFAULT_GRIPPER_COMMAND
    gripper_controller_name: str = DEFAULT_GRIPPER_CONTROLLER
    ok: bool = False
    notes: List[str] = field(default_factory=list)


def normalize_arm_side(side: str | None) -> str:
    value = (side or DEFAULT_ARM_SIDE).strip().lower()
    if value not in _SIDE_PRESETS:
        raise ValueError(f"arm_side must be 'left' or 'right', got {side!r}")
    return value


def detect_available_sides(names: Sequence[str]) -> List[str]:
    name_set = set(names)
    sides: List[str] = []
    if any(t in name_set for t in ("/left_current_pose", "/left_target")):
        sides.append("left")
    if any(t in name_set for t in ("/right_current_pose", "/right_target")):
        sides.append("right")
    if not sides:
        # No side-tagged Cartesian topics: treat as single left arm (legacy UR5e).
        sides = ["left"]
    return sides


@dataclass
class CameraProbeHints:
    topics: List[str] = field(default_factory=list)
    preferred: str = DEFAULT_CAMERA_TOPIC
    ok: bool = False
    notes: List[str] = field(default_factory=list)


_GRAPH_CACHE: Optional[Tuple[TopicsAndTypes, Optional[str]]] = None


def clear_graph_cache() -> None:
    global _GRAPH_CACHE
    _GRAPH_CACHE = None


def discover_graph(
    max_attempts: int = 12,
    *,
    use_cache: bool = True,
) -> Tuple[TopicsAndTypes, Optional[str]]:
    """Return ``(topics_and_types, error)``. ``error`` is set when discovery fails."""
    global _GRAPH_CACHE
    if use_cache and _GRAPH_CACHE is not None:
        return _GRAPH_CACHE

    try:
        from ros2_robot_interface.utils.discovery import discover_topics_with_types
    except Exception as exc:  # pragma: no cover - optional dependency path
        result: Tuple[TopicsAndTypes, Optional[str]] = ([], f"ros2_robot_interface 不可用: {exc}")
        _GRAPH_CACHE = result
        return result

    try:
        topics = discover_topics_with_types(max_attempts=max_attempts)
    except Exception as exc:
        result = ([], f"ROS 2 话题探测失败: {exc}")
        _GRAPH_CACHE = result
        return result
    if not topics:
        result = ([], "未发现 ROS 2 话题（请确认已 source 且仿真/驱动在跑）")
        _GRAPH_CACHE = result
        return result

    result = (topics, None)
    _GRAPH_CACHE = result
    return result


def _topic_names(topics_and_types: TopicsAndTypes) -> List[str]:
    return [name for name, _ in topics_and_types]


def _first_present(names: Sequence[str], candidates: Iterable[str]) -> Optional[str]:
    name_set = set(names)
    for candidate in candidates:
        if candidate in name_set:
            return candidate
    return None


def _endswith_present(names: Sequence[str], suffix: str) -> List[str]:
    return sorted(name for name in names if name.endswith(suffix))


def _pick_gripper_command(names: Sequence[str], side: str) -> Optional[str]:
    preferred = _SIDE_PRESETS[side]["gripper_command"]
    hit = _first_present(names, preferred)
    if hit:
        return hit
    matches = []
    for name in names:
        lower = name.lower()
        if side not in lower and side != "left":
            continue
        if side == "left" and "right" in lower:
            continue
        for suffix in _GRIPPER_COMMAND_SUFFIXES:
            if "gripper" in lower and name.endswith(suffix):
                matches.append(name)
                break
    return sorted(matches)[0] if matches else None


def _pick_gripper_controller(names: Sequence[str], side: str) -> Optional[str]:
    preferred = _SIDE_PRESETS[side]["gripper_controller"]
    for controller in preferred:
        if any(n.endswith(f"/{controller}/target_percent") or n.endswith(f"/{controller}/target_command") for n in names):
            return controller
    # Fallback: infer from any matching percent topic on this side.
    for name in names:
        lower = name.lower()
        if side == "left" and "right" in lower:
            continue
        if side == "right" and "right" not in lower:
            continue
        if name.endswith("/target_percent") and "gripper" in lower:
            parts = [p for p in name.strip("/").split("/") if p]
            if len(parts) >= 2:
                return parts[-2]
    return None


def _is_image_topic(types: Sequence[str]) -> bool:
    return any(any(marker in t for marker in _IMAGE_TYPE_MARKERS) for t in types)


def _camera_rank(topic: str) -> tuple:
    lower = topic.lower()
    hint_score = min(
        (i for i, hint in enumerate(_CAMERA_NAME_HINTS) if hint in lower),
        default=len(_CAMERA_NAME_HINTS),
    )
    # Prefer shorter, leaf-looking rgb topics.
    return (hint_score, len(topic), topic)


def list_image_topics(topics_and_types: TopicsAndTypes) -> List[str]:
    topics = [name for name, types in topics_and_types if _is_image_topic(types)]
    return sorted(topics, key=_camera_rank)


def sample_joint_state_names(topic: str, timeout_s: float = 3.0) -> Optional[List[str]]:
    """Briefly subscribe to ``sensor_msgs/JointState`` and return joint names."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except Exception:
        return None

    if not rclpy.ok():
        rclpy.init()

    names: List[str] = []
    node = Node("roboapi_fasim_probe_joints")
    done = {"ok": False}

    def _cb(msg: JointState):
        if msg.name and not done["ok"]:
            names[:] = list(msg.name)
            done["ok"] = True

    sub = node.create_subscription(JointState, topic, _cb, 10)
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline and not done["ok"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
    return names or None


def sample_pose_frame_id(topic: str, timeout_s: float = 3.0) -> Optional[str]:
    """Read ``header.frame_id`` from the first Pose / PoseStamped on ``topic``."""
    try:
        import rclpy
        from geometry_msgs.msg import Pose, PoseStamped
        from rclpy.node import Node
    except Exception:
        return None

    if not rclpy.ok():
        rclpy.init()

    frame: List[str] = []
    node = Node("roboapi_fasim_probe_pose")
    done = {"ok": False}

    def _from_stamped(msg: PoseStamped):
        if msg.header.frame_id and not done["ok"]:
            frame[:] = [msg.header.frame_id]
            done["ok"] = True

    def _from_pose(_msg: Pose):
        # Unstamped Pose has no frame_id; ignore.
        return

    sub_stamped = node.create_subscription(PoseStamped, topic, _from_stamped, 10)
    # Some stacks publish plain Pose on the same name; keep a no-op sub so DDS matches.
    sub_pose = node.create_subscription(Pose, topic, _from_pose, 10)
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline and not done["ok"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(sub_stamped)
        node.destroy_subscription(sub_pose)
        node.destroy_node()
    return frame[0] if frame else None


def _arm_joint_names_from_state(names: Sequence[str], side: str) -> str:
    other = "right" if side == "left" else "left"

    def _is_gripper_like(name: str) -> bool:
        lower = name.lower()
        return "gripper" in lower or "finger" in lower or "hand" in lower

    sided = [
        name
        for name in names
        if side in name.lower() and other not in name.lower() and not _is_gripper_like(name)
    ]
    if sided:
        return ",".join(sided)

    filtered = [
        name
        for name in names
        if other not in name.lower() and not _is_gripper_like(name)
    ]
    use = filtered or list(names)
    return ",".join(use) if use else DEFAULT_JOINT_NAMES


def _gripper_joint_from_state(names: Sequence[str], side: str) -> Optional[str]:
    preferred = _SIDE_PRESETS[side]["gripper_joint"]
    hit = _first_present(names, preferred)
    if hit:
        return hit
    other = "right" if side == "left" else "left"
    for name in names:
        lower = name.lower()
        if ("gripper" in lower or "finger" in lower) and other not in lower:
            if side == "right" and side not in lower:
                continue
            return name
    return None


def side_topic_defaults(side: str) -> dict:
    """Static defaults for a side when the ROS graph has no matching topics."""
    side = normalize_arm_side(side)
    preset = _SIDE_PRESETS[side]
    return {
        "arm_side": side,
        "pose_topic": preset["pose"][0],
        "target_topic": preset["target"][0],
        "current_target_topic": preset["current_target"][0],
        "gripper_command_topic": preset["gripper_command"][0],
        "gripper_controller_name": preset["gripper_controller"][0],
        "gripper_joint_name": preset["gripper_joint"][0],
    }


def probe_arm_hints(
    *,
    arm_side: str | None = None,
    sample_live: bool = True,
    max_attempts: int = 12,
) -> ArmProbeHints:
    """Build arm setup defaults from the live ROS 2 graph when possible."""
    topics_and_types, error = discover_graph(max_attempts=max_attempts)
    hints = ArmProbeHints()
    if error:
        hints.notes.append(error)
        if arm_side:
            hints.arm_side = normalize_arm_side(arm_side)
            hints.available_sides = [hints.arm_side]
            hints.__dict__.update({k: v for k, v in side_topic_defaults(hints.arm_side).items() if k != "arm_side"})
        return hints

    names = _topic_names(topics_and_types)
    available = detect_available_sides(names)
    hints.available_sides = available
    if arm_side is None:
        side = "left" if "left" in available else available[0]
    else:
        side = normalize_arm_side(arm_side)
        if side not in available:
            hints.notes.append(f"图上未看到 {side} 臂话题，仍按 {side} 预设填写")
    hints.arm_side = side

    # Start from side presets, then overwrite with live matches.
    for key, value in side_topic_defaults(side).items():
        if key != "arm_side":
            setattr(hints, key, value)

    notes: List[str] = [f"已发现 {len(names)} 个话题", f"控制臂: {side}（可选: {', '.join(available)}）"]
    preset = _SIDE_PRESETS[side]

    pose = _first_present(names, preset["pose"])
    if pose:
        hints.pose_topic = pose
        notes.append(f"位姿话题: {pose}")

    target = _first_present(names, preset["target"])
    if target:
        hints.target_topic = target
        notes.append(f"目标话题: {target}")

    current_target = _first_present(names, preset["current_target"])
    if current_target:
        hints.current_target_topic = current_target
        notes.append(f"当前目标话题: {current_target}")

    joint_states = _first_present(names, _JOINT_STATES_CANDIDATES)
    if joint_states is None:
        prefixed = _endswith_present(names, "/joint_states")
        if prefixed:
            joint_states = prefixed[0]
    if joint_states:
        hints.joint_states_topic = joint_states
        notes.append(f"关节状态: {joint_states}")

    gripper_cmd = _pick_gripper_command(names, side)
    if gripper_cmd:
        hints.gripper_command_topic = gripper_cmd
        notes.append(f"夹爪命令: {gripper_cmd}")
        parts = [p for p in gripper_cmd.strip("/").split("/") if p]
        if len(parts) >= 2 and parts[-1] == "position_command":
            hints.gripper_joint_name = parts[-2]

    gripper_ctrl = _pick_gripper_controller(names, side)
    if gripper_ctrl:
        hints.gripper_controller_name = gripper_ctrl
        notes.append(f"夹爪控制器: {gripper_ctrl}")

    if sample_live and joint_states:
        state_names = sample_joint_state_names(joint_states)
        if state_names:
            hints.joint_names = _arm_joint_names_from_state(state_names, side)
            notes.append(f"关节名来自 {joint_states}（{len(hints.joint_names.split(','))} 轴）")
            gripper_joint = _gripper_joint_from_state(state_names, side)
            if gripper_joint:
                hints.gripper_joint_name = gripper_joint
                notes.append(f"夹爪关节: {gripper_joint}")

    if sample_live and hints.pose_topic:
        stamped = (
            f"{hints.pose_topic}/stamped"
            if f"{hints.pose_topic}/stamped" in set(names)
            else hints.pose_topic
        )
        frame_id = sample_pose_frame_id(stamped)
        if frame_id:
            hints.base_frame = frame_id
            notes.append(f"基坐标系来自 {stamped}: {frame_id}")

    hints.ok = True
    hints.notes = notes
    return hints


def probe_camera_hints(*, max_attempts: int = 12) -> CameraProbeHints:
    """List Image topics and rank likely wrist/scene camera feeds."""
    topics_and_types, error = discover_graph(max_attempts=max_attempts)
    hints = CameraProbeHints()
    if error:
        hints.notes.append(error)
        return hints

    image_topics = list_image_topics(topics_and_types)
    if not image_topics:
        hints.notes.append("未发现 sensor_msgs/Image 话题")
        return hints

    hints.topics = image_topics
    hints.preferred = image_topics[0]
    hints.ok = True
    hints.notes.append(f"发现 {len(image_topics)} 个 Image 话题，建议: {hints.preferred}")
    return hints


def print_probe_notes(title: str, notes: Sequence[str]) -> None:
    print(f">>> {title}")
    for note in notes:
        print(f"    {note}")
