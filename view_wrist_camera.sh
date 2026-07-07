#!/usr/bin/env bash
set -euo pipefail

# Low-rate preview for the wrist ZMQ camera. Use for setup/alignment; stop it
# before recording or policy rollout if camera/control timing matters.

source "${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
conda activate "${LEROBOT_ENV:-$HOME/miniconda3/envs/lerobot}"

REPO_DIR="${GELLO_REPO_DIR:-$HOME/gello_software}"
cd "$REPO_DIR"

python scripts/view_zmq_camera.py \
  --host "${CAMERA_HOST:-${ROBOT_HOST:-127.0.0.1}}" \
  --port "${WRIST_CAMERA_PORT:-5000}" \
  --width "${CAMERA_WIDTH:-640}" \
  --height "${CAMERA_HEIGHT:-480}" \
  --fps "${PREVIEW_FPS:-0.5}" \
  --timeout-ms "${ZMQ_TIMEOUT_MS:-3000}" \
  --window-name "${WINDOW_NAME:-GELLO wrist camera}" \
  --lerobot-key "${LEROBOT_KEY:-observation.images.wrist}" \
  "$@"
