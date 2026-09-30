#!/usr/bin/env python
import argparse
import time

from lerobot_robot_ur5e_ros2 import SO101UR5eLeader, SO101UR5eLeaderConfig


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--n", type=int, default=200)
    args = ap.parse_args()

    teleop = SO101UR5eLeader(SO101UR5eLeaderConfig(id="mocktest", leader_port=args.port))
    teleop.connect(calibrate=False)
    print("connected:", teleop.is_connected)
    print("action_features:", list(teleop.action_features))

    node = teleop._node
    prev_armed = None
    for i in range(args.n):
        act = teleop.get_action()
        if node._armed != prev_armed:
            print(f"[{i}] armed -> {node._armed}")
            prev_armed = node._armed
        if i % 20 == 0:
            pretty = " ".join(f"{k}={v:+.3f}" for k, v in act.items())
            print(f"[{i}] armed={node._armed} fb={node._fallback_count}  {pretty}")
        time.sleep(0.03)

    teleop.disconnect()
    print("done. connected:", teleop.is_connected)


if __name__ == "__main__":
    main()
