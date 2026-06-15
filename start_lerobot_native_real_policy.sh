#!/usr/bin/env bash
set -euo pipefail

# Native LeRobot Bring-Your-Own-Hardware rollout for the GELLO/ZMQ Panda stack.
# Start the normal GELLO robot/camera ZMQ servers first, then run this script in
# the LeRobot environment. The first run installs this repo and the LeRobot robot
# plugin in editable mode so the official lerobot-record CLI can discover it.

source "${CONDA_SH:-/home/tim/miniconda3/etc/profile.d/conda.sh}"
conda activate "${LEROBOT_ENV:-/home/tim/miniconda3/envs/lerobot}"

REPO_DIR="${GELLO_REPO_DIR:-/home/tim/gello_software}"
cd "$REPO_DIR"

python -m pip install -e .
python -m pip install -e lerobot_robot_gello

export DATASET_REPO_ID="${DATASET_REPO_ID:-TimR643/eval_left_right_native_gello}"
export DATASET_ROOT="${DATASET_ROOT:-/home/tim/lerobot_data/eval_left_right_native_gello}"
export CKPT="${CKPT:-/home/tim/lerobot_outputs/train/smolvla_left_right_wrist/checkpoints/last/pretrained_model}"

: "${TASK:=Move right when the red block is visible, otherwise move left.}"
: "${FPS:=5}"
: "${EPISODE_TIME_S:=50}"
: "${NUM_EPISODES:=1}"
: "${RESET_TIME_S:=5}"
: "${WARMUP_TIME_S:=5}"

mkdir -p "$DATASET_ROOT"
test -d "$CKPT" || { echo "FEHLT: CKPT=$CKPT"; exit 1; }

echo "Using native LeRobot CLI with robot.type=gello_zmq"
echo "DATASET_REPO_ID=$DATASET_REPO_ID"
echo "DATASET_ROOT=$DATASET_ROOT"
echo "CKPT=$CKPT"

lerobot-record \
  --robot.type=gello_zmq \
  --robot.robot_host="${ROBOT_HOST:-127.0.0.1}" \
  --robot.robot_port="${ROBOT_PORT:-6001}" \
  --robot.camera_host="${CAMERA_HOST:-${ROBOT_HOST:-127.0.0.1}}" \
  --robot.wrist_camera_port="${WRIST_CAMERA_PORT:-5000}" \
  --robot.cameras='("wrist",)' \
  --robot.max_joint_delta="${MAX_JOINT_DELTA:-0.015}" \
  --robot.max_gripper_delta="${MAX_GRIPPER_DELTA:-0.03}" \
  --robot.action_mode="${ACTION_MODE:-absolute_joint_position}" \
  --fps="$FPS" \
  --root="$DATASET_ROOT" \
  --repo-id="$DATASET_REPO_ID" \
  --task="$TASK" \
  --warmup-time-s="$WARMUP_TIME_S" \
  --episode-time-s="$EPISODE_TIME_S" \
  --reset-time-s="$RESET_TIME_S" \
  --num-episodes="$NUM_EPISODES" \
  --push-to-hub=0 \
  --policy.path="$CKPT"
