#!/usr/bin/env bash
set -euo pipefail

# Show a known-good LeRobot wrist frame next to the live wrist ZMQ camera so a
# moved physical camera can be adjusted back to the recorded viewpoint.

source "${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
conda activate "${LEROBOT_ENV:-$HOME/miniconda3/envs/lerobot}"

REPO_DIR="${GELLO_REPO_DIR:-$HOME/gello_software}"
cd "$REPO_DIR"

DATASET_ROOT="${DATASET_ROOT:-${LEROBOT_ROOT:-$HOME/lerobot_data/pick_red_green_lego}}"

python scripts/align_zmq_camera_to_lerobot_frame.py \
  --dataset-root "$DATASET_ROOT" \
  --camera wrist \
  --episode-index "${REFERENCE_EPISODE_INDEX:-0}" \
  --frame-index "${REFERENCE_FRAME_INDEX:-0}" \
  --host "${CAMERA_HOST:-${ROBOT_HOST:-127.0.0.1}}" \
  --port "${WRIST_CAMERA_PORT:-5000}" \
  --width "${CAMERA_WIDTH:-640}" \
  --height "${CAMERA_HEIGHT:-480}" \
  --fps "${PREVIEW_FPS:-2}" \
  --timeout-ms "${ZMQ_TIMEOUT_MS:-3000}" \
  "$@"
