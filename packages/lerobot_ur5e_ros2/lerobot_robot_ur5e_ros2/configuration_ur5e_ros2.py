from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig

from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("ur5e_ros2_follower")
@dataclass
class UR5eROS2FollowerConfig(RobotConfig):
    robot_ip: str = "None"
    mock_gripper: bool = False

    loop_hz: float = 30.0
    home_duration: float = 10.0
    max_home_deg_per_sec: float = 15.0
    max_dq_per_cmd: float = 0.25
    armed_settle_sec: float = 1.5
    armed_settle_dq_scale: float = 0.2
    max_joint_vel: float = 1.0

    settle_ramp_on_connect: bool = True

    obs_timeout_s: float = 2.0

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "top": RealSenseCameraConfig(
                serial_number_or_name="944122072848",
                fps=30,
                width=640,
                height=480,
            )
        }
    )
