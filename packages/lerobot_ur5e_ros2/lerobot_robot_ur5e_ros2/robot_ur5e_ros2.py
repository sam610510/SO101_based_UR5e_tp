from __future__ import annotations

import threading
import time

import numpy as np
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.robot import Robot

from ._bridge import (
    UR5E_HOME_Q,
    UR5E_JOINT_NAMES,
    UR5E_MOTORS,
    ArmedSync,
    GripperLink,
    RosSpinner,
    UR5eTeleopNode,
    is_folded_unsafe,
)
from .configuration_ur5e_ros2 import UR5eROS2FollowerConfig


class _NoopGripper:
    def set_target(self, pos: int) -> None:  # noqa: D401
        self.last = int(pos)

    def close(self) -> None:
        pass


class _FollowerNode(UR5eTeleopNode):
    def __init__(self, cfg: UR5eROS2FollowerConfig):
        Node.__init__(self, "ur5e_ros2_follower_node")

        if not cfg.mock_gripper and cfg.robot_ip == "None":
            raise RuntimeError(
                "robot_ip is required (Robotiq TCP-Modbus socket). "
                "Pass --robot.robot_ip=<UR5e IP> (or --robot.mock_gripper=true)."
            )

        self.declare_parameter("loop_hz", float(cfg.loop_hz))
        self.declare_parameter("max_home_deg_per_sec", float(cfg.max_home_deg_per_sec))

        self.max_dq = float(cfg.max_dq_per_cmd)
        self.armed_settle_sec = float(cfg.armed_settle_sec)
        self.armed_settle_scale = float(cfg.armed_settle_dq_scale)
        self.max_joint_vel = float(cfg.max_joint_vel)
        self._armed_time = None
        self._prev_pub_q = None
        self._prev_pub_time = None
        self._homed = False
        self._last_q = UR5E_HOME_Q.copy()

        self.urdf_limits = {n: (-2 * np.pi, 2 * np.pi) for n in UR5E_JOINT_NAMES}
        self._usub = self.create_subscription(String, "/robot_description", self._urdf_cb, 10)

        self.ACTION_NAME = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
        self._traj_pub = self.create_publisher(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
        )
        self._traj_action = ActionClient(self, FollowJointTrajectory, self.ACTION_NAME)

        self._gripper = _NoopGripper() if cfg.mock_gripper else GripperLink(
            cfg.robot_ip, logger=self.get_logger()
        )

        self._state_lock = threading.Lock()
        self._latest_q: np.ndarray | None = None
        self._latest_grip: float | None = None
        self._last_grip_cmd: float = 0.0
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(Float64, "/teleop/gripper_state", self._grip_cb, 10)

    def _js_cb(self, msg: JointState) -> None:
        q = [None] * 6
        for i, name in enumerate(UR5E_JOINT_NAMES):
            if name in msg.name:
                q[i] = msg.position[msg.name.index(name)]
        if all(v is not None for v in q):
            with self._state_lock:
                self._latest_q = np.array(q, dtype=float)

    def _grip_cb(self, msg: Float64) -> None:
        with self._state_lock:
            self._latest_grip = float(np.clip(msg.data, 0.0, 1.0))


class UR5eROS2Follower(Robot):
    config_class = UR5eROS2FollowerConfig
    name = "ur5e_ros2_follower"

    def __init__(self, config: UR5eROS2FollowerConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self._node: _FollowerNode | None = None
        self._connected = False

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in [*UR5E_MOTORS, "gripper"]}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self._connected and all(c.is_connected for c in self.cameras.values())

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

        spinner = RosSpinner.instance()
        spinner.ensure_init()
        self._node = _FollowerNode(self.config)

        if not self._node._traj_action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                f"Action server {self._node.ACTION_NAME} not available -- is the "
                "UR driver up with scaled_joint_trajectory_controller active?"
            )
        if not self._node._goto_home_blocking(duration=self.config.home_duration):
            raise RuntimeError("Homing failed (see log). Aborting connect().")

        if self.config.settle_ramp_on_connect:
            self._node._armed_time = time.time()

        spinner.add_node(self._node)

        for cam in self.cameras.values():
            cam.connect()

        self._connected = True

    def disconnect(self) -> None:
        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except Exception:
                pass
        if self._node is not None:
            RosSpinner.instance().remove_node(self._node)
            try:
                self._node._gripper.close()
            except Exception:
                pass
            self._node.destroy_node()
            self._node = None
        self._connected = False

    def get_observation(self) -> RobotObservation:
        if not self._connected or self._node is None:
            raise RuntimeError(f"{self} is not connected")

        deadline = time.time() + self.config.obs_timeout_s
        while time.time() < deadline:
            with self._node._state_lock:
                q = None if self._node._latest_q is None else self._node._latest_q.copy()
            if q is not None:
                break
            time.sleep(0.005)
        if q is None:
            raise RuntimeError("No /joint_states received within obs_timeout_s")

        with self._node._state_lock:
            grip = self._node._latest_grip
        if grip is None:
            grip = self._node._last_grip_cmd

        obs: RobotObservation = {f"{UR5E_MOTORS[i]}.pos": float(q[i]) for i in range(6)}
        obs["gripper.pos"] = float(grip)
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.async_read()
        return obs

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self._connected or self._node is None:
            raise RuntimeError(f"{self} is not connected")

        q = np.array([float(action[f"{m}.pos"]) for m in UR5E_MOTORS], dtype=float)

        unsafe, reason = is_folded_unsafe(q)
        if unsafe:
            self._node.get_logger().warn(
                f"Policy target unsafe ({reason}), freezing pose (action not executed)",
                throttle_duration_sec=0.5,
            )
            q = self._node._last_q.copy()

        armed_t = ArmedSync.instance().take()
        if armed_t is not None:
            self._node._armed_time = armed_t

        self._node._publish(q)
        sent_q = (
            self._node._prev_pub_q.copy()
            if self._node._prev_pub_q is not None
            else q
        )
        self._node._last_q = sent_q.copy()

        grip = float(np.clip(action.get("gripper.pos", self._node._last_grip_cmd), 0.0, 1.0))
        self._node._gripper.set_target(int(round(grip * 255)))
        self._node._last_grip_cmd = grip

        sent: RobotAction = {f"{UR5E_MOTORS[i]}.pos": float(sent_q[i]) for i in range(6)}
        sent["gripper.pos"] = grip
        return sent
