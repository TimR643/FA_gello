#!/bin/bash
set -e

# Recorder for the safe Franka workflow.
# It receives only robot state/action from the Franka laptop and captures wrist/base
# cameras locally on this recording machine by default. Do not pull cameras from
# the Franka/Polymetis realtime laptop during robot control.

PROJECT_DIR="${PROJECT_DIR:-$HOME/gello_software}"
CONDA_SETUP="${CONDA_SETUP:-$HOME/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-lerobot}"

BIND_HOSTNAME="${BIND_HOSTNAME:-0.0.0.0}"
RECORD_STREAM_PORT="${RECORD_STREAM_PORT:-7000}"

CAMERA_SOURCE="${CAMERA_SOURCE:-local_realsense}"
HPC_CAMERA_HOST="${HPC_CAMERA_HOST:-127.0.0.1}"
WRIST_PORT="${WRIST_PORT:-5000}"
BASE_PORT="${BASE_PORT:-5001}"
WRIST_CAMERA_ID="${WRIST_CAMERA_ID:-6CD1460304A5}"
BASE_CAMERA_ID="${BASE_CAMERA_ID:-318122303303}"

LEROBOT_ROOT="${LEROBOT_ROOT:-$HOME/lerobot_data/basecam_test}"
LEROBOT_REPO_ID="${LEROBOT_REPO_ID:-local/basecam_test}"
LEROBOT_FPS="${LEROBOT_FPS:-10}"
LEROBOT_TASK="${LEROBOT_TASK:-go right if a red block is detected, go left if a green bock is detected}"
LEROBOT_ROBOT_TYPE="${LEROBOT_ROBOT_TYPE:-panda_gello}"
LEROBOT_BATCH_ENCODING_SIZE="${LEROBOT_BATCH_ENCODING_SIZE:-1}"
CAMERA_TIMEOUT_MS="${CAMERA_TIMEOUT_MS:-3000}"

cat <<EOF
Starting HPC remote-camera recorder
  Project dir:       $PROJECT_DIR
  Bind:              $BIND_HOSTNAME:$RECORD_STREAM_PORT
  Camera source:     $CAMERA_SOURCE
  Remote camera host:$HPC_CAMERA_HOST
  Wrist/Base ports:  $WRIST_PORT / $BASE_PORT
  Wrist/Base IDs:    $WRIST_CAMERA_ID / $BASE_CAMERA_ID
  Camera timeout:    ${CAMERA_TIMEOUT_MS} ms
  Dataset root:      $LEROBOT_ROOT
  Repo id:           $LEROBOT_REPO_ID
  FPS:               $LEROBOT_FPS
  Task:              $LEROBOT_TASK
EOF

source "$CONDA_SETUP"
conda activate "$CONDA_ENV"
cd "$PROJECT_DIR"

python - <<PY
import socket
import sys

host = "${BIND_HOSTNAME}"
port = int("${RECORD_STREAM_PORT}")
bind_host = "" if host in {"0.0.0.0", "*"} else host
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    try:
        sock.bind((bind_host, port))
    except OSError as exc:
        print(
            f"ERROR: recording stream port {host}:{port} is already in use. "
            "Stop the old recorder or set RECORD_STREAM_PORT to a free port "
            "on both this recorder and the Franka laptop.",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc
PY

if [ "$CAMERA_SOURCE" = "local_realsense" ]; then
  python scripts/check_realsense_cameras.py \
    --wrist-camera-id "$WRIST_CAMERA_ID" \
    --base-camera-id "$BASE_CAMERA_ID"
fi

python experiments/record_lerobot_stream_with_remote_cameras.py \
  --bind-hostname "$BIND_HOSTNAME" \
  --port "$RECORD_STREAM_PORT" \
  --camera-source "$CAMERA_SOURCE" \
  --camera-hostname "$HPC_CAMERA_HOST" \
  --wrist-camera-port "$WRIST_PORT" \
  --base-camera-port "$BASE_PORT" \
  --wrist-camera-id "$WRIST_CAMERA_ID" \
  --base-camera-id "$BASE_CAMERA_ID" \
  --lerobot-root "$LEROBOT_ROOT" \
  --lerobot-repo-id "$LEROBOT_REPO_ID" \
  --lerobot-fps "$LEROBOT_FPS" \
  --lerobot-task "$LEROBOT_TASK" \
  --lerobot-robot-type "$LEROBOT_ROBOT_TYPE" \
  --cameras wrist base \
  --lerobot-streaming-encoding \
  --lerobot-batch-encoding-size "$LEROBOT_BATCH_ENCODING_SIZE" \
  --camera-timeout-ms "$CAMERA_TIMEOUT_MS"
