from __future__ import annotations

import time
from typing import Any

import numpy as np
from rclpy.node import Node

from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator

from ._bridge import (
    UR5E_HOME_Q,
    UR5E_MOTORS,
    ArmedSync,
    RosSpinner,
    UR5eTeleopNode,
    scs,
)
from .configuration_so101_ur5e import SO101UR5eLeaderConfig

_BAUD = 1_000_000


class _LeaderNode(UR5eTeleopNode):
    def __init__(self, cfg: SO101UR5eLeaderConfig):
        Node.__init__(self, "so101_ur5e_leader_node")

        self.max_jump = float(cfg.max_jump)
        self.filter_alpha = float(cfg.filter_alpha)
        self.raw_alpha = float(cfg.raw_filter_alpha)
        self.pose_alpha = float(cfg.pose_filter_alpha)

        self.calib = [2078, 2060, 1985, 2042, 2047, 2136]
        self.leader_ids = [1, 2, 3, 4, 5, 6]

        self.port = scs.PortHandler(cfg.leader_port)
        self.packet = scs.protocol_packet_handler(self.port, 0)
        if not self.port.openPort() or not self.port.setBaudRate(_BAUD):
            raise RuntimeError(f"Cannot open SO101 leader port {cfg.leader_port} @ {_BAUD}")
        for lid in self.leader_ids:
            self.packet.write1ByteTxRx(lid, 19, 2)
            self.packet.write1ByteTxRx(lid, 40, 0)
        self.get_logger().info("SO101 leader connected (torque released)")

        self._armed = False
        self._last_pos = np.array([0.10, 0.0, 0.15])
        self._last_q = UR5E_HOME_Q.copy()
        self._first = True
        self._fallback_count = 0
        self._armed_time = None
        self._quat_filtered = None
        self._rads_filtered = None

        self._captured_q = UR5E_HOME_Q.copy()
        self._captured_grip = 0.0

    def _publish(self, q) -> None:  # noqa: D401
        self._captured_q = np.array(q, dtype=float).copy()

    def _update_gripper(self, target_pos) -> None:
        self._captured_grip = float(np.clip(target_pos, 0, 255)) / 255.0

    def close_port(self) -> None:
        try:
            self.port.closePort()
        except Exception:
            pass


class SO101UR5eLeader(Teleoperator):
    config_class = SO101UR5eLeaderConfig
    name = "so101_ur5e_leader"

    def __init__(self, config: SO101UR5eLeaderConfig):
        super().__init__(config)
        self.config = config
        self._node: _LeaderNode | None = None
        self._connected = False

    @property
    def action_features(self) -> dict:
        return {f"{m}.pos": float for m in [*UR5E_MOTORS, "gripper"]}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            raise RuntimeError(f"{self} already connected")
        RosSpinner.instance().ensure_init()
        self._node = _LeaderNode(self.config)
        self._connected = True
        while not self._node._armed:
            self._node._loop()
            time.sleep(1.0 / 30.0)
        if self._node._armed_time is not None:
            ArmedSync.instance().mark_armed(self._node._armed_time)

    def disconnect(self) -> None:
        if not self._connected:
            return
        if self._node is not None:
            self._node.close_port()
            self._node.destroy_node()
            self._node = None
        self._connected = False

    def get_action(self) -> RobotAction:
        if not self._connected or self._node is None:
            raise RuntimeError(f"{self} is not connected")
        was_armed = self._node._armed
        self._node._loop()
        if not was_armed and self._node._armed:
            ArmedSync.instance().mark_armed(self._node._armed_time)
        q = self._node._captured_q
        action: RobotAction = {f"{UR5E_MOTORS[i]}.pos": float(q[i]) for i in range(6)}
        action["gripper.pos"] = float(self._node._captured_grip)
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass
