from __future__ import annotations

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("so101_ur5e_leader")
@dataclass
class SO101UR5eLeaderConfig(TeleoperatorConfig):
    leader_port: str = "/dev/ttyACM0"

    max_jump: float = 0.5
    filter_alpha: float = 0.7
    raw_filter_alpha: float = 0.3
    pose_filter_alpha: float = 0.3
