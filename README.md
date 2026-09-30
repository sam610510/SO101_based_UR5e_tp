# SO101-based UR5e Teleoperation

Teleoperate a UR5e (+ Robotiq 2F-140 gripper) follower arm from an SO101 leader arm, with a from-scratch analytic IK/FK pipeline, layered safety checks, and a LeRobot integration for recording demonstrations and training/evaluating imitation-learning policies (ACT, Diffusion Policy) via ROS2.

This repo does not include any trained models or recorded datasets -- only the code needed to run teleoperation, record data, and drive training/eval through LeRobot.

## How it works

- `so101_ur5e_teleop_node.py` reads the SO101 leader's joint encoders, computes its end-effector pose via a hand-written forward kinematics chain, linearly maps that pose from the SO101 leader's workspace into the (larger) UR5e workspace, then solves UR5e inverse kinematics (real-time warm-start, falling back to a multi-seed solver when the warm start fails to converge) and streams the result to `scaled_joint_trajectory_controller`.
- Before every publish, the target pose is checked against `is_folded_unsafe()` -- a set of 3D-distance checks (end-effector, elbow, and flange distance to the base) that freeze the arm at its last known-safe pose instead of moving toward a self-collision-prone configuration. A per-command joint-delta clamp (`max_dq_per_cmd`) and a ramped "settle" period right after arming bound how fast the arm can move at all.
- The node can run standalone (`ros2 launch so100_control ur5e_teleop.launch.py`) or be reused, unmodified, by a LeRobot `Robot`/`Teleoperator` pair (`packages/lerobot_ur5e_ros2`) so the exact same kinematics and safety logic apply whether a human is driving the leader arm or a trained policy is driving the follower directly.

## Repository layout

```
packages/
  so100_control/          Core teleop node (ROS2 Python package)
    so100_control/so101_ur5e_teleop_node.py   SO101 FK, workspace mapping, UR5e IK, safety checks, ROS2 node
    so100_control/scservo_sdk/                Feetech servo SDK (vendor code, used to read the SO101 leader)
    launch/ur5e_teleop.launch.py              Standalone launch: robot_state_publisher + teleop node + RViz

  lerobot_ur5e_ros2/       LeRobot integration (Python package, pip-installable)
    lerobot_robot_ur5e_ros2/_bridge.py             Imports so101_ur5e_teleop_node.py unmodified, re-exports shared pieces
    lerobot_robot_ur5e_ros2/robot_ur5e_ros2.py      LeRobot Robot (follower): send_action() -> is_folded_unsafe() check -> original _publish()
    lerobot_robot_ur5e_ros2/teleop_so101_ur5e.py    LeRobot Teleoperator (leader): get_action() runs the original _loop() and captures its output
    lerobot_robot_ur5e_ros2/configuration_*.py      Dataclass configs (--robot.* / --teleop.* CLI fields)
    scripts/record_episodes.sh                      Records N episodes, one lerobot-record invocation per episode
    scripts/eval_episodes.sh                         Same pattern for policy evaluation
    scripts/test_leader.py                           Standalone smoke test for the leader wrapper
    scripts/test_follower_mock.py                    Standalone smoke test for the follower wrapper (mock hardware)

  ur5e_description/       UR5e + Robotiq 2F-140 URDF/xacro and meshes, for RViz visualization
  so101_description/      SO101 leader URDF/xacro (reference only -- the teleop node's kinematics are
                           computed directly in Python, not loaded from this URDF)
```

## Prerequisites

- Ubuntu 22.04, ROS2 Humble
- [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) built in a workspace (referred to below as `~/ur_ws`)
- [LeRobot](https://github.com/huggingface/lerobot) installed in a Python (conda or venv) environment, referred to below as the `lerobot` env (this repo was developed and tested against the [Seeed-Projects/lerobot](https://github.com/Seeed-Projects/lerobot) fork; verify `register_third_party_plugins()` and the `RecordConfig`/`TrainPipelineConfig` CLI shape if using a different fork or a much older/newer commit)
- SO101 leader arm connected over USB serial (default `/dev/ttyACM0`)
- UR5e with a Robotiq 2F-140 gripper on a Robotiq TCP-Modbus interface (or `mock_gripper=true` for testing without one)
- A RealSense camera for the `top` view used during recording/eval (edit `configuration_ur5e_ros2.py` or override `--robot.cameras` if your setup differs)

## Setup

1. Copy `packages/so100_control`, `packages/ur5e_description`, and `packages/so101_description` into a colcon workspace `src/`, then:
   ```bash
   colcon build --packages-select so100_control ur5e_description so101_description --symlink-install
   ```
2. Install the LeRobot integration into your `lerobot` environment:
   ```bash
   source /opt/ros/humble/setup.bash
   <path-to-lerobot-env>/bin/pip install -e packages/lerobot_ur5e_ros2
   ```
   Verify: `<path-to-lerobot-env>/bin/python -c "import rclpy, lerobot, lerobot_robot_ur5e_ros2; print('ok')"`
   The distribution name (`lerobot_robot_ur5e_ros2` in `pyproject.toml`) must keep the `lerobot_robot_` prefix -- LeRobot's `register_third_party_plugins()` uses that prefix to auto-import it and register the `ur5e_ros2_follower` / `so101_ur5e_leader` CLI choices; renaming it breaks that discovery.
3. Get a UR5e calibration file (`ur_calibration` on the real robot, or use the driver's built-in default for mock hardware) and note its path -- it is passed as `kinematics_params_file` when launching the real driver. This repo does not ship a calibration file since it is specific to each physical robot.
4. **Recalibrate the reference constants for your own physical setup.** `UR5E_HOME_Q`, `SO101_ARM_REF`, `SO101_WS`, and `UR5E_WS` in `so101_ur5e_teleop_node.py` were measured for one specific SO101/UR5e mounting arrangement and table height. If your leader and follower are positioned differently, recompute these before relying on the safety thresholds derived from them (see "Safety mechanisms" below) -- otherwise the "arm" reference point and workspace mapping will not match your physical setup, and the distance-based safety checks may not carry the same margin they were verified with here.

## Every new terminal

```bash
source /opt/ros/humble/setup.bash
source ~/ur_ws/install/setup.bash
source <path-to-your-colcon-ws>/install/setup.bash
```

Run `lerobot-record` / `lerobot-train` with the `lerobot` env's own interpreter, e.g. `<lerobot-env>/bin/lerobot-record`, or `conda activate`/`source activate` that env first.

## Quick smoke tests

Before a full recording session, these two scripts independently sanity-check each half of the LeRobot wrapper:

```bash
cd packages/lerobot_ur5e_ros2/scripts

# Follower: needs mock (or real) UR5e hardware up (see below). Also proves the
# per-command joint-delta clamp actually limits a deliberately oversized step.
<lerobot-env>/bin/python test_follower_mock.py [--camera]

# Leader: needs the SO101 leader connected and powered.
<lerobot-env>/bin/python test_leader.py
```

## Mock testing (do this before touching real hardware)

Terminal 1 -- fake UR5e driver:
```bash
source /opt/ros/humble/setup.bash
source ~/ur_ws/install/setup.bash
ros2 launch ur_robot_driver ur5e.launch.py \
  robot_ip:=0.0.0.0 use_fake_hardware:=true launch_rviz:=true \
  initial_joint_controller:=scaled_joint_trajectory_controller \
  description_package:=ur5e_description description_file:=ur5e_gripper.urdf.xacro
```
`description_package`/`description_file` renders the gripper mesh in RViz so the visualized end-effector matches what `is_folded_unsafe()` actually checks (tool0 including the 25cm gripper extension), instead of a bare arm that hides how close the gripper tip really is to the base.

Terminal 2 -- standalone teleop node (or run the LeRobot smoke tests / `lerobot-record` from here instead):
```bash
ros2 launch so100_control ur5e_teleop.launch.py robot_ip:=127.0.0.1 use_rviz:=false
```
For a mock gripper without any real Robotiq hardware, point `robot_ip` at a host running a fake Modbus/TCP responder on port 63352, or use `--robot.mock_gripper=true` on the LeRobot side.

The node starts by moving the UR5e smoothly to a fixed home pose, then waits ("standby") until the SO101 leader is moved within `ARM_THRESH` (default 0.02m) of `SO101_ARM_REF` before it arms and starts following.

## Real hardware pre-flight checklist

- Emergency stop within reach and known to work
- Workspace clear of people/obstacles within the arm's reach
- UR pendant: External Control program running, IP matches `--robot.robot_ip` / `robot_ip:=`
- UR5e's own safety limits (speed/force/momentum) left at their configured values -- do not loosen them for testing
- Start with small, slow leader motions and confirm direction/scale before doing anything larger
- A second person present, able to reach the e-stop
- Robotiq gripper activated once via the pendant's URCap page (Installation > URCaps > Robotiq Gripper > Activate) after every power cycle

Then the same two-terminal pattern as above, with `use_fake_hardware:=false`, a real `robot_ip:=<UR5e IP>`, and `kinematics_params_file:=<path to your calibration file>` on the driver launch.

## Recording demonstrations

```bash
cd packages/lerobot_ur5e_ros2/scripts
./record_episodes.sh <repo_id> <num_episodes> "<task description>" [episode_time_s=8] [mock_gripper=true] [robot_ip] [reset_time_s=6]
```

This invokes `lerobot-record` once per episode (rather than once for the whole session) so that every episode gets its own `connect()` -> smooth home -> wait for the SO101 leader to re-arm -> record -> disconnect cycle. `SO101UR5eLeader.connect()` blocks until the leader is armed before returning, so the recorded window always starts already in following mode -- no leading frames of the arm sitting still at home.

## Training

Standard `lerobot-train`, e.g.:
```bash
<lerobot-env>/bin/lerobot-train \
  --policy.type=diffusion --dataset.repo_id=<repo_id> \
  --output_dir=outputs/<run_name> --steps=100000 --batch_size=8 --save_freq=10000 \
  --policy.push_to_hub=false
```
Swap `--policy.type=act` for ACT. LeRobot does not hold out a validation split by itself; to do so, pass `--dataset.episodes='[0,1,...]'` listing only the training episode indices, and evaluate held-out episodes' loss against saved checkpoints yourself if you want to pick a checkpoint by validation performance rather than just taking the last one (see "Known limitations").

## Evaluating a trained policy

```bash
cd packages/lerobot_ur5e_ros2/scripts
./eval_episodes.sh <policy_path> <repo_id> <num_episodes> "<task description>" [episode_time_s=15] [mock_gripper=true] [robot_ip]
```
`repo_id`'s last path segment must start with `eval_` (a LeRobot requirement whenever `--policy.path` is set). `policy_path` is a `pretrained_model` directory, e.g. `outputs/<run_name>/checkpoints/last/pretrained_model`.

Let a policy-driven episode run to completion rather than stopping it early -- especially for Diffusion Policy, which re-plans in chunks (`n_action_steps`) and can look like it is doing nothing useful in the first few seconds before the motion it actually learned becomes visible.

## Safety mechanisms

All of the following live in `is_folded_unsafe()` / `_publish()` in `so101_ur5e_teleop_node.py`, and apply identically whether a human is driving the leader or a policy is driving the follower directly (`UR5eROS2Follower.send_action()` runs the same `is_folded_unsafe()` check before publishing a policy's raw output):

- End-effector, elbow, and flange 3D distance to the base origin, each with its own threshold, tuned empirically against a scan of the legitimate workspace to avoid both false positives (freezing during normal reach motions) and false negatives (missing real folded-arm configurations)
- End-effector height floor, with a small buffer below the workspace's own lower bound so normal operation at the bottom of the reach envelope doesn't trip the freeze
- Per-command joint-delta clamp (`max_dq_per_cmd`), measured against the last value actually published (not the last requested value) so it cannot become a no-op
- A ramped "settle" window right after arming, so a large warm-start jump right when the leader comes into range doesn't move the arm at full speed
- The EMA-blended pose that is actually published is re-checked against `is_folded_unsafe()` after blending, since blending two individually-safe joint configurations in joint space does not guarantee every point on that path is itself safe

None of this checks for collisions with external objects (the table, a workpiece, a person) -- only self-folding / getting too close to the arm's own base. Treat the UR5e's own built-in protective-stop behavior and a human operator's judgment as the layer that covers everything else.

## Known limitations

- No train/validation split by default (see "Training" above for a manual workaround); a low training loss does not by itself indicate the policy will complete the task on real hardware.
- Diffusion Policy's default `n_action_steps` re-planning cadence can produce visibly discontinuous motion at each re-plan boundary; reducing `num_inference_steps` (faster inference, so re-planning happens closer to the control loop's intended rate) is a more direct fix than changing `n_action_steps` itself.
- The real-time IK solver has no automatic recovery if repeated warm-start attempts fail to converge under fast leader motion; it falls back to a slower multi-seed solve, which increases loop latency rather than failing outright.
- `UR5E_HOME_Q`, `SO101_ARM_REF`, and the workspace bounding boxes are specific to one physical mounting arrangement (see "Setup" above).

## External dependency

The Robotiq 2F-140 mesh/macro under `packages/ur5e_description/urdf/` originates from [ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper); only the description assets are vendored here, not the ros2_control driver packages from that repo.
