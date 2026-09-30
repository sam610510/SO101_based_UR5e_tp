#!/usr/bin/env python
import argparse
import time

import numpy as np

from lerobot_robot_ur5e_ros2 import UR5eROS2Follower, UR5eROS2FollowerConfig
from lerobot_robot_ur5e_ros2._bridge import UR5E_HOME_Q, UR5E_MOTORS


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", action="store_true", help="also open the RealSense")
    args = ap.parse_args()

    cams = None if args.camera else {}
    cfg = UR5eROS2FollowerConfig(
        id="mocktest",
        mock_gripper=True,
        home_duration=4.0,
        **({} if cams is None else {"cameras": cams}),
    )
    robot = UR5eROS2Follower(cfg)

    print("connecting (this runs the smooth homing routine)...")
    robot.connect(calibrate=False)
    print("connected:", robot.is_connected)

    obs = robot.get_observation()
    q0 = np.array([obs[f"{m}.pos"] for m in UR5E_MOTORS])
    print("observation q (rad):", np.round(q0, 4))
    print("  home target      :", np.round(UR5E_HOME_Q, 4))
    print("  |q - home| max   :", float(np.max(np.abs(q0 - UR5E_HOME_Q))))
    print("gripper.pos:", obs["gripper.pos"])
    for k, v in obs.items():
        if hasattr(v, "shape"):
            print(f"  image {k}: {v.shape} {v.dtype}")

    target = UR5E_HOME_Q.copy()
    target[0] += 0.4
    print("\nsending 10 commands toward home + 0.4rad on shoulder_pan ...")
    for i in range(10):
        sent = robot.send_action(
            {**{f"{m}.pos": float(target[j]) for j, m in enumerate(UR5E_MOTORS)},
             "gripper.pos": 0.5}
        )
        time.sleep(0.1)
        cur = np.array([robot.get_observation()[f"{m}.pos"] for m in UR5E_MOTORS])
        print(f"  step {i}: sent pan={sent['shoulder_pan.pos']:+.4f}  meas pan={cur[0]:+.4f}")

    print("\ndisconnecting ...")
    robot.disconnect()
    print("done. connected:", robot.is_connected)


if __name__ == "__main__":
    main()
