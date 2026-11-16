"""ROS 2 graph probing helpers for Fasim RoboAPI setup defaults.

Uses ``ros2_robot_interface`` discovery when available; falls back silently
so ``roboapi init`` still works offline.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Iterable, List, Optional, Sequence, Tuple

TopicsAndTypes = List[Tuple[str, List[str]]]

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

_POSE_CANDIDATES = (
    "/left_current_pose",
    "/right_current_pose",
    "/current_pose",
)
_TARGET_CANDIDATES = (
    "/left_target",
    "/right_target",
    "/target",
)
_CURRENT_TARGET_CANDIDATES = (
    "/left_current_target",
    "/right_current_target",
    "/current_target",
)
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


def _pick_gripper_command(names: Sequence[str]) -> Optional[str]:
    preferred = (
        "/gripper_joint/position_command",
        "/left_gripper_joint/position_command",
        "/right_gripper_joint/position_command",
    )
    hit = _first_present(names, preferred)
    if hit:
        return hit
    matches = []
    for name in names:
        for suffix in _GRIPPER_COMMAND_SUFFIXES:
            if "gripper" in name and name.endswith(suffix):
                matches.append(name)
                break
    return sorted(matches)[0] if matches else None


def _pick_gripper_controller(names: Sequence[str]) -> Optional[str]:
    for name in names:
        if name.endswith("/gripper_controller/target_percent"):
            # /gripper_controller/target_percent -> gripper_controller
            parts = [p for p in name.strip("/").split("/") if p]
            if len(parts) >= 2:
                return parts[-2]
        if name.endswith("/left_gripper_controller/target_percent"):
            return "left_gripper_controller"
        if name.endswith("/right_gripper_controller/target_percent"):
            return "right_gripper_controller"
    if any(n.endswith("/gripper_controller/target_command") for n in names):
        return "gripper_controller"
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


def _arm_joint_names_from_state(names: Sequence[str]) -> str:
    filtered = [
        name
        for name in names
        if "gripper" not in name.lower()
        and "finger" not in name.lower()
        and "hand" not in name.lower()
    ]
    use = filtered or list(names)
    return ",".join(use) if use else DEFAULT_JOINT_NAMES


def _gripper_joint_from_state(names: Sequence[str]) -> Optional[str]:
    for name in names:
        lower = name.lower()
        if "gripper" in lower or "finger" in lower:
            return name
    return None


def probe_arm_hints(*, sample_live: bool = True, max_attempts: int = 12) -> ArmProbeHints:
    """Build arm setup defaults from the live ROS 2 graph when possible."""
    topics_and_types, error = discover_graph(max_attempts=max_attempts)
    hints = ArmProbeHints()
    if error:
        hints.notes.append(error)
        return hints

    names = _topic_names(topics_and_types)
    notes: List[str] = [f"已发现 {len(names)} 个话题"]

    pose = _first_present(names, _POSE_CANDIDATES)
    if pose:
        hints.pose_topic = pose
        notes.append(f"位姿话题: {pose}")

    target = _first_present(names, _TARGET_CANDIDATES)
    if target:
        hints.target_topic = target
        notes.append(f"目标话题: {target}")

    current_target = _first_present(names, _CURRENT_TARGET_CANDIDATES)
    if current_target:
        hints.current_target_topic = current_target
        notes.append(f"当前目标话题: {current_target}")

    joint_states = _first_present(names, _JOINT_STATES_CANDIDATES)
    if joint_states is None:
        # Prefixed joint_states, e.g. /ur5e/joint_states
        prefixed = _endswith_present(names, "/joint_states")
        if prefixed:
            joint_states = prefixed[0]
    if joint_states:
        hints.joint_states_topic = joint_states
        notes.append(f"关节状态: {joint_states}")

    gripper_cmd = _pick_gripper_command(names)
    if gripper_cmd:
        hints.gripper_command_topic = gripper_cmd
        notes.append(f"夹爪命令: {gripper_cmd}")
        # Derive joint leaf from topic when possible: /gripper_joint/position_command
        parts = [p for p in gripper_cmd.strip("/").split("/") if p]
        if len(parts) >= 2 and parts[-1] == "position_command":
            hints.gripper_joint_name = parts[-2]

    gripper_ctrl = _pick_gripper_controller(names)
    if gripper_ctrl:
        hints.gripper_controller_name = gripper_ctrl
        notes.append(f"夹爪控制器: {gripper_ctrl}")

    if sample_live and joint_states:
        state_names = sample_joint_state_names(joint_states)
        if state_names:
            hints.joint_names = _arm_joint_names_from_state(state_names)
            notes.append(f"关节名来自 {joint_states}（{len(hints.joint_names.split(','))} 轴）")
            gripper_joint = _gripper_joint_from_state(state_names)
            if gripper_joint:
                hints.gripper_joint_name = gripper_joint
                notes.append(f"夹爪关节: {gripper_joint}")

    if sample_live and pose:
        # Prefer stamped companion if present.
        stamped = f"{pose}/stamped" if f"{pose}/stamped" in set(names) else pose
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
