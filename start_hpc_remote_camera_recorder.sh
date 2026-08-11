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

LEROBOT_ROOT="${LEROBOT_ROOT:-$HOME/lerobot_data/precision_peg_in_hole_v3}"
LEROBOT_REPO_ID="${LEROBOT_REPO_ID:-local/precision_peg_in_hole_v3}"
LEROBOT_FPS="${LEROBOT_FPS:-10}"
# go_left_right_even_...: "go right if a red block is detected, go left if a green bock is detected"
# pick up the red rectangle and go above the necessary hight
#cube pushing: Push the white cube straight forward. -> geradeaus
#              Push the white cube forward, then guide it right through the opening. -> nach rechts
LEROBOT_TASK="${LEROBOT_TASK:-Put the peg in the designated hole}"
LEROBOT_ROBOT_TYPE="${LEROBOT_ROBOT_TYPE:-panda_gello}"
LEROBOT_BATCH_ENCODING_SIZE="${LEROBOT_BATCH_ENCODING_SIZE:-1}"
CAMERA_TIMEOUT_MS="${CAMERA_TIMEOUT_MS:-3000}"
H5_LOG_ENABLED="${H5_LOG_ENABLED:-true}"
H5_LOG_DIR="${H5_LOG_DIR:-${LEROBOT_ROOT}_h5}"
H5_LOG_BASENAME="${H5_LOG_BASENAME:-teleoperation}"
H5_FLUSH_EVERY="${H5_FLUSH_EVERY:-1}"

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
  H5 logging:        $H5_LOG_ENABLED
  H5 directory:      $H5_LOG_DIR
  H5 basename:       $H5_LOG_BASENAME
EOF

source "$CONDA_SETUP"
conda activate "$CONDA_ENV"
cd "$PROJECT_DIR"

if [[ "$H5_LOG_ENABLED" == "true" ]]; then
  python -c 'import h5py' || {
    echo "FEHLT: h5py ist im Conda-Environment '$CONDA_ENV' nicht installiert." >&2
    echo "Installiere es mit: python -m pip install h5py" >&2
    exit 1
  }
  H5_LOG_ARG="--h5-log-enabled"
else
  H5_LOG_ARG="--no-h5-log-enabled"
fi

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
  "$H5_LOG_ARG" \
  --h5-log-dir "$H5_LOG_DIR" \
  --h5-log-basename "$H5_LOG_BASENAME" \
  --h5-flush-every "$H5_FLUSH_EVERY"
