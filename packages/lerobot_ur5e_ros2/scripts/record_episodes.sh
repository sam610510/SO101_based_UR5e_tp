#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 3 ]; then
  echo "usage: $0 <repo_id> <num_episodes> <single_task> [episode_time_s=8] [mock_gripper=true] [robot_ip] [reset_time_s=6]" >&2
  exit 1
fi

REPO_ID="$1"
NUM_EPISODES="$2"
SINGLE_TASK="$3"
EPISODE_TIME_S="${4:-8}"
MOCK_GRIPPER="${5:-true}"
ROBOT_IP="${6:-}"
RESET_TIME_S="${7:-6}"

DEFAULT_LEROBOT_ENV="$HOME/miniconda3/envs/lerobot/bin"
if [ -n "${LEROBOT_RECORD_BIN:-}" ]; then
  LEROBOT="$LEROBOT_RECORD_BIN"
elif [ -x "$DEFAULT_LEROBOT_ENV/lerobot-record" ]; then
  LEROBOT="$DEFAULT_LEROBOT_ENV/lerobot-record"
else
  LEROBOT="$(command -v lerobot-record)"
fi
if [ -n "${LEROBOT_PYTHON_BIN:-}" ]; then
  PY="$LEROBOT_PYTHON_BIN"
elif [ -x "$DEFAULT_LEROBOT_ENV/python" ]; then
  PY="$DEFAULT_LEROBOT_ENV/python"
else
  PY="$(command -v python3)"
fi

DATASET_DIR="$HOME/.cache/huggingface/lerobot/$REPO_ID"
if [ -d "$DATASET_DIR" ]; then
  TOTAL_EPISODES=$("$PY" -c "
import json
try:
    print(json.load(open('$DATASET_DIR/meta/info.json')).get('total_episodes', 0))
except Exception:
    print(0)
" 2>/dev/null)
  if [ "${TOTAL_EPISODES:-0}" = "0" ]; then
    echo "Detected '$REPO_ID' as a stale empty dataset skeleton (0 episodes); removing it and starting over."
    rm -rf "$DATASET_DIR"
  else
    echo "repo_id '$REPO_ID' already has $TOTAL_EPISODES episode(s) of real data. To avoid overwriting it, pick a different repo_id (or delete it yourself after checking)." >&2
    exit 1
  fi
fi

ROBOT_ARGS=(--robot.type=ur5e_ros2_follower --robot.id=ur5e01
  --robot.cameras='{ top: {type: intelrealsense, serial_number_or_name: "944122072848", fps: 30, width: 640, height: 480}}')
if [ "$MOCK_GRIPPER" = "true" ]; then
  ROBOT_ARGS+=(--robot.mock_gripper=true)
else
  if [ -z "$ROBOT_IP" ]; then
    echo "mock_gripper=false requires <robot_ip> as the 6th argument" >&2
    exit 1
  fi
  ROBOT_ARGS+=(--robot.robot_ip="$ROBOT_IP")
fi

for i in $(seq 1 "$NUM_EPISODES"); do
  echo "=== episode $i / $NUM_EPISODES : connect -> home (not recorded) -> wait for arm -> record ==="
  RESUME_FLAGS=()
  if [ "$i" -gt 1 ]; then
    RESUME_FLAGS=(--resume=true)
  fi
  "$LEROBOT" \
    "${ROBOT_ARGS[@]}" \
    --teleop.type=so101_ur5e_leader --teleop.leader_port=/dev/ttyACM0 --teleop.id=so101 \
    --dataset.repo_id="$REPO_ID" --dataset.num_episodes=1 \
    --dataset.single_task="$SINGLE_TASK" --dataset.episode_time_s="$EPISODE_TIME_S" --dataset.reset_time_s="$RESET_TIME_S" \
    --dataset.push_to_hub=false --display_data=false \
    "${RESUME_FLAGS[@]}"
done

echo "All $NUM_EPISODES episode(s) recorded: $REPO_ID"
