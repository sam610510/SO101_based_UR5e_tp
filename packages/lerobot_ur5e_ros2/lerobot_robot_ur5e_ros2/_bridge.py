from __future__ import annotations

import importlib
import os
import sys
import threading
from pathlib import Path

_SO100_SRC = os.environ.get("SO100_CONTROL_SRC") or str(
    Path(__file__).resolve().parents[2] / "so100_control"
)
if os.path.isdir(_SO100_SRC) and _SO100_SRC not in sys.path:
    sys.path.insert(0, _SO100_SRC)

teleop = importlib.import_module("so100_control.so101_ur5e_teleop_node")

UR5eTeleopNode = teleop.UR5eTeleopNode
GripperLink = teleop.GripperLink
scs = teleop.scs

UR5E_JOINT_NAMES = teleop.UR5E_JOINT_NAMES
UR5E_HOME_Q = teleop.UR5E_HOME_Q

ur5e_fk_full = teleop.ur5e_fk_full
ur5e_fk_pos = teleop.ur5e_fk_pos
map_gripper_pos = teleop.map_gripper_pos
is_folded_unsafe = teleop.is_folded_unsafe

UR5E_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
MOTOR_TO_JOINT = dict(zip(UR5E_MOTORS, UR5E_JOINT_NAMES))

import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402


class RosSpinner:
    _inst: "RosSpinner | None" = None

    def __init__(self) -> None:
        self._executor: MultiThreadedExecutor | None = None
        self._thread: threading.Thread | None = None
        self._nodes: set = set()
        self._lock = threading.Lock()

    @classmethod
    def instance(cls) -> "RosSpinner":
        if cls._inst is None:
            cls._inst = cls()
        return cls._inst

    def ensure_init(self) -> None:
        if not rclpy.ok():
            rclpy.init()

    def add_node(self, node) -> None:
        self.ensure_init()
        with self._lock:
            if self._executor is None:
                self._executor = MultiThreadedExecutor()
                self._thread = threading.Thread(
                    target=self._executor.spin, name="ros-spinner", daemon=True
                )
                self._thread.start()
            self._executor.add_node(node)
            self._nodes.add(node)

    def remove_node(self, node) -> None:
        with self._lock:
            if self._executor is not None and node in self._nodes:
                self._executor.remove_node(node)
            self._nodes.discard(node)


class ArmedSync:
    _inst: "ArmedSync | None" = None

    def __init__(self) -> None:
        self._armed_time: float | None = None
        self._consumed = True

    @classmethod
    def instance(cls) -> "ArmedSync":
        if cls._inst is None:
            cls._inst = cls()
        return cls._inst

    def mark_armed(self, t: float) -> None:
        self._armed_time = t
        self._consumed = False

    def take(self) -> float | None:
        if self._consumed:
            return None
        self._consumed = True
        return self._armed_time
