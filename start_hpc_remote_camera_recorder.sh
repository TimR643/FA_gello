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

LEROBOT_ROOT="${LEROBOT_ROOT:-$HOME/lerobot_data/cam_sync_test}"
LEROBOT_REPO_ID="${LEROBOT_REPO_ID:-local/cam_sync_test}"
LEROBOT_FPS="${LEROBOT_FPS:-10}"
# go_left_right_even_...: "go right if a red block is detected, go left if a green bock is detected"
LEROBOT_TASK="${LEROBOT_TASK:-pick up the red rectangle and go above the necessary hight}"
LEROBOT_ROBOT_TYPE="${LEROBOT_ROBOT_TYPE:-panda_gello}"
LEROBOT_BATCH_ENCODING_SIZE="${LEROBOT_BATCH_ENCODING_SIZE:-1}"
CAMERA_TIMEOUT_MS="${CAMERA_TIMEOUT_MS:-3000}"
CAMERA_SYNC_MODE="${CAMERA_SYNC_MODE:-on_frame}"
CAMERA_POLL_FPS="${CAMERA_POLL_FPS:-10}"
CAMERA_MAX_AGE_MS="${CAMERA_MAX_AGE_MS:-250}"
CAMERA_FRAME_DELAY_MS="${CAMERA_FRAME_DELAY_MS:-100}"
CAMERA_FRAME_DELAY_FRAMES="${CAMERA_FRAME_DELAY_FRAMES:-1}"
CAMERA_BUFFER_SECONDS="${CAMERA_BUFFER_SECONDS:-2}"
LOG_SYNC_EVERY="${LOG_SYNC_EVERY:-50}"

cat <<EOF
Starting HPC remote-camera recorder
  Project dir:       $PROJECT_DIR
  Bind:              $BIND_HOSTNAME:$RECORD_STREAM_PORT
  Laptop camera host:$HPC_CAMERA_HOST
  Wrist/Base ports:  $WRIST_PORT / $BASE_PORT
  Camera timeout:    ${CAMERA_TIMEOUT_MS} ms
  Camera sync mode:  ${CAMERA_SYNC_MODE}
  Camera poll FPS:   ${CAMERA_POLL_FPS}
  Camera delay:      ${CAMERA_FRAME_DELAY_MS} ms
  Camera frame delay:${CAMERA_FRAME_DELAY_FRAMES} frame(s)
  Camera buffer:     ${CAMERA_BUFFER_SECONDS} s
  Camera stale warn: ${CAMERA_MAX_AGE_MS} ms
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
  --camera-timeout-ms "$CAMERA_TIMEOUT_MS" \
  --camera-sync-mode "$CAMERA_SYNC_MODE" \
  --camera-poll-fps "$CAMERA_POLL_FPS" \
  --camera-max-age-ms "$CAMERA_MAX_AGE_MS" \
  --camera-frame-delay-ms "$CAMERA_FRAME_DELAY_MS" \
  --camera-frame-delay-frames "$CAMERA_FRAME_DELAY_FRAMES" \
  --camera-buffer-seconds "$CAMERA_BUFFER_SECONDS" \
  --log-sync-every "$LOG_SYNC_EVERY"
