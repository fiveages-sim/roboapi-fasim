"""fasim wrist camera, native transport converted to raw-view RGB."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from roboapi.hardware import CameraConfig, ComponentOption, ComponentPlugin

from .probe import DEFAULT_CAMERA_TOPIC, print_probe_notes, probe_camera_hints


@dataclass
class FasimCameraConfig(CameraConfig):
    type: str = "fasim"
    topic: str = DEFAULT_CAMERA_TOPIC
    namespace: str = ""
    stale_after_s: float = 0.5

    @property
    def connection_id(self) -> str:
        return self.topic

    def validate(self) -> None:
        super().validate()
        if not self.topic.strip():
            raise ValueError("camera.topic is required")


def _choose_camera_topic(setup, hints) -> str:
    if not hints.ok or not hints.topics:
        return setup.ask("腕部相机 Image 话题", hints.preferred or DEFAULT_CAMERA_TOPIC)

    if len(hints.topics) == 1:
        return setup.ask("腕部相机 Image 话题", hints.topics[0])

    choices = list(hints.topics) + ["手动输入..."]
    index = setup.choose("腕部相机 Image 话题（已探测）", choices)
    if index >= len(hints.topics):
        return setup.ask("腕部相机 Image 话题", hints.preferred)
    return hints.topics[index]


def configure(option, setup):
    hints = probe_camera_hints()
    if hints.ok:
        print_probe_notes("已探测 sensor_msgs/Image 话题", hints.notes)
    else:
        print_probe_notes("未能探测相机话题，使用内置默认值", hints.notes or ["无详情"])

    return option.config_type(
        topic=_choose_camera_topic(setup, hints),
        namespace="",  # official setup.ask rejects a blank default
    )


def camera_plugin():
    return ComponentPlugin(
        options=(ComponentOption("FaSim ROS 2 腕部相机", FasimCameraConfig),),
        create=FasimCamera,
        configure=configure,
        setup_notes=(
            "需要仿真或驱动向该话题发布 1280×720、约 30 FPS 的 RGB 图像；不要裁剪、补边或镜像。",
            "configure 会扫描 Image 话题（如 /acepro2/rgb、/action6/rgb），供向导选择。",
        ),
    )


class FasimCamera:
    def __init__(self, cfg):
        self.cfg = cfg
        self._node = None
        self._subscription = None
        self._lock = threading.Lock()
        self._frame = None
        self._stamp = 0.0
        self._read_started_at = 0.0
        self._error = None
        self._live = False

    @property
    def live(self) -> bool:
        if not self._live or self._error is not None:
            return False
        with self._lock:
            if self._frame is None:
                return False
            return (time.monotonic() - self._stamp) < float(self.cfg.stale_after_s)

    @property
    def last_error(self) -> str | None:
        if self._error:
            return self._error
        if self._live and self._frame is not None and not self.live:
            return f"camera frames are stale (>{self.cfg.stale_after_s}s)"
        return None

    def connect(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image

        from .session import spinner, wait_until

        if self._node is not None:
            return
        if not rclpy.ok():
            rclpy.init()
        self._error = None
        self._node = Node("roboapi_fasim_camera", namespace=self.cfg.namespace or "")
        self._subscription = self._node.create_subscription(
            Image, self.cfg.topic, self._on_image, 10
        )
        spinner().add(self._node)
        try:
            wait_until(lambda: self._frame is not None, 5.0, f"no image on {self.cfg.topic}")
            self._live = True
        except Exception as exc:
            self._error = str(exc)
            self.disconnect()
            raise

    def disconnect(self):
        from .session import spinner

        self._live = False
        if self._node is not None:
            spinner().remove(self._node)
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
            self._subscription = None
        with self._lock:
            self._frame = None
            self._stamp = self._read_started_at = 0.0
        self._error = None

    def read(self):
        with self._lock:
            if self._frame is None:
                raise RuntimeError(self._error or f"no frame yet from {self.cfg.topic}")
            return self._frame, time.monotonic() - self._stamp

    def read_after(self, after, timeout_s, should_stop=None):
        deadline = time.monotonic() + timeout_s
        while True:
            if should_stop is not None and should_stop():
                return None
            if after is None:
                return self.read()
            with self._lock:
                if self._error:
                    raise RuntimeError(self._error)
                if self._frame is not None and self._read_started_at > after:
                    return self._frame, time.monotonic() - self._stamp
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("camera produced no new frame after the motion boundary")
            if not self._live and self._node is None:
                raise RuntimeError("camera disconnected while waiting for a new frame")
            time.sleep(min(0.005, remaining))

    def _on_image(self, msg):
        started_at = time.monotonic()
        try:
            image = _image_to_rgb(msg)
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
            return
        with self._lock:
            self._frame = image
            self._stamp = time.monotonic()
            self._read_started_at = started_at
            self._error = None


def _image_to_rgb(msg):
    import numpy as np

    try:
        from cv_bridge import CvBridge

        if not hasattr(_image_to_rgb, "_bridge"):
            _image_to_rgb._bridge = CvBridge()
        bgr = _image_to_rgb._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        import cv2

        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    except Exception:
        encoding = (msg.encoding or "").lower()
        height, width = int(msg.height), int(msg.width)
        if encoding in ("rgb8", "bgr8"):
            data = np.frombuffer(msg.data, dtype=np.uint8)
            step = int(msg.step) if msg.step else width * 3
            image = np.ndarray((height, width, 3), dtype=np.uint8)
            packed = data.reshape((height, step))[:, : width * 3].reshape((height, width, 3))
            if encoding == "bgr8":
                image[:] = packed[:, :, ::-1]
            else:
                image[:] = packed
            return image
        raise RuntimeError(f"unsupported image encoding {msg.encoding!r}")
