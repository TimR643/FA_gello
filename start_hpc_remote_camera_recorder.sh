#!/bin/bash
set -e

# HPC-side recorder for the remote-camera workflow.
# It receives only robot state/action from the laptop and pulls wrist/base RGB
# frames from the laptop camera ZMQ servers.

PROJECT_DIR="${PROJECT_DIR:-$HOME/gello_software}"
CONDA_SETUP="${CONDA_SETUP:-$HOME/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-lerobot}"

BIND_HOSTNAME="${BIND_HOSTNAME:-0.0.0.0}"
RECORD_STREAM_PORT="${RECORD_STREAM_PORT:-7000}"

# Set this to the IP/hostname that the HPC can use to reach the laptop.
HPC_CAMERA_HOST="${HPC_CAMERA_HOST:-127.0.0.1}"
WRIST_PORT="${WRIST_PORT:-5000}"
BASE_PORT="${BASE_PORT:-5001}"

LEROBOT_ROOT="${LEROBOT_ROOT:-$HOME/lerobot_data/go_left_right_even}"
LEROBOT_REPO_ID="${LEROBOT_REPO_ID:-local/go_left_right_even}"
LEROBOT_FPS="${LEROBOT_FPS:-10}"
LEROBOT_TASK="${LEROBOT_TASK:-go right if a red block is detected, go left if a green block is detected}"
LEROBOT_ROBOT_TYPE="${LEROBOT_ROBOT_TYPE:-panda_gello}"
LEROBOT_BATCH_ENCODING_SIZE="${LEROBOT_BATCH_ENCODING_SIZE:-1}"
CAMERA_TIMEOUT_MS="${CAMERA_TIMEOUT_MS:-3000}"

cat <<EOF
Starting HPC remote-camera recorder
  Project dir:       $PROJECT_DIR
  Bind:              $BIND_HOSTNAME:$RECORD_STREAM_PORT
  Laptop camera host:$HPC_CAMERA_HOST
  Wrist/Base ports:  $WRIST_PORT / $BASE_PORT
  Camera timeout:    ${CAMERA_TIMEOUT_MS} ms
  Dataset root:      $LEROBOT_ROOT
  Repo id:           $LEROBOT_REPO_ID
  FPS:               $LEROBOT_FPS
  Task:              $LEROBOT_TASK
EOF

source "$CONDA_SETUP"
conda activate "$CONDA_ENV"
cd "$PROJECT_DIR"

python experiments/record_lerobot_stream_with_remote_cameras.py \
  --bind-hostname "$BIND_HOSTNAME" \
  --port "$RECORD_STREAM_PORT" \
  --camera-hostname "$HPC_CAMERA_HOST" \
  --wrist-camera-port "$WRIST_PORT" \
  --base-camera-port "$BASE_PORT" \
  --lerobot-root "$LEROBOT_ROOT" \
  --lerobot-repo-id "$LEROBOT_REPO_ID" \
  --lerobot-fps "$LEROBOT_FPS" \
  --lerobot-task "$LEROBOT_TASK" \
  --lerobot-robot-type "$LEROBOT_ROBOT_TYPE" \
  --cameras wrist base \
  --lerobot-streaming-encoding \
  --lerobot-batch-encoding-size "$LEROBOT_BATCH_ENCODING_SIZE" \
  --camera-timeout-ms "$CAMERA_TIMEOUT_MS"
