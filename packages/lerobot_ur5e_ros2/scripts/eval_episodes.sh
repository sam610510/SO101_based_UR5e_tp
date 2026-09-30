#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 4 ]; then
  echo "usage: $0 <policy_path> <repo_id> <num_episodes> <single_task> [episode_time_s=15] [mock_gripper=true] [robot_ip]" >&2
  exit 1
fi

POLICY_PATH="$1"
REPO_ID="$2"
NUM_EPISODES="$3"
SINGLE_TASK="$4"
EPISODE_TIME_S="${5:-15}"
MOCK_GRIPPER="${6:-true}"
ROBOT_IP="${7:-}"

LAST_SEGMENT="${REPO_ID##*/}"
case "$LAST_SEGMENT" in
  eval_*) ;;
  *)
    echo "repo_id's last segment must start with 'eval_' (got '$LAST_SEGMENT') -- lerobot-record requires this whenever --policy.path is set." >&2
    exit 1
    ;;
esac

if [ ! -d "$POLICY_PATH" ]; then
  echo "policy_path does not exist: $POLICY_PATH" >&2
  exit 1
fi

LEROBOT="$HOME/miniconda3/envs/lerobot/bin/lerobot-record"
PY="$HOME/miniconda3/envs/lerobot/bin/python"

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
    echo "repo_id '$REPO_ID' already has $TOTAL_EPISODES episode(s) of real eval data. To avoid overwriting results from a different policy/checkpoint, pick a different repo_id (or delete it yourself after checking)." >&2
    exit 1
  fi
fi

ROBOT_ARGS=(--robot.type=ur5e_ros2_follower --robot.id=ur5e01
  --robot.cameras='{ top: {type: intelrealsense, serial_number_or_name: "944122072848", fps: 30, width: 640, height: 480}}')
if [ "$MOCK_GRIPPER" = "true" ]; then
  ROBOT_ARGS+=(--robot.mock_gripper=true)
else
  if [ -z "$ROBOT_IP" ]; then
    echo "mock_gripper=false requires <robot_ip> as the 7th argument" >&2
    exit 1
  fi
  ROBOT_ARGS+=(--robot.robot_ip="$ROBOT_IP")
fi

for i in $(seq 1 "$NUM_EPISODES"); do
  echo "=== eval episode $i / $NUM_EPISODES : connect -> home (not recorded) -> policy drives -> record ==="
  RESUME_FLAGS=()
  if [ "$i" -gt 1 ]; then
    RESUME_FLAGS=(--resume=true)
  fi
  "$LEROBOT" \
    "${ROBOT_ARGS[@]}" \
    --policy.path="$POLICY_PATH" \
    --dataset.repo_id="$REPO_ID" --dataset.num_episodes=1 \
    --dataset.single_task="$SINGLE_TASK" --dataset.episode_time_s="$EPISODE_TIME_S" --dataset.reset_time_s=1 \
    --dataset.push_to_hub=false --display_data=false \
    "${RESUME_FLAGS[@]}"
done

echo "All $NUM_EPISODES eval episode(s) complete: $REPO_ID"
