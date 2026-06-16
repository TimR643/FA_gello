#!/usr/bin/env bash
set -euo pipefail

# Check whether the live GELLO/Panda ZMQ state is close to the desired recording
# start pose. Start the normal GELLO robot ZMQ server / SSH tunnel first.

source "${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
conda activate "${LEROBOT_ENV:-$HOME/miniconda3/envs/lerobot}"

REPO_DIR="${GELLO_REPO_DIR:-$HOME/gello_software}"
cd "$REPO_DIR"

python scripts/check_gello_start_position.py \
  --robot-host "${ROBOT_HOST:-127.0.0.1}" \
  --robot-port "${ROBOT_PORT:-6001}" \
  --timeout-ms "${ZMQ_TIMEOUT_MS:-3000}" \
  --target-deg "${START_JOINTS_DEG:-0,-90,90,-90,-90,0,0}" \
  --target-gripper "${START_GRIPPER:-1.0}" \
  --arm-tolerance-rad "${START_ARM_TOLERANCE_RAD:-0.035}" \
  --gripper-tolerance "${START_GRIPPER_TOLERANCE:-0.08}" \
  "$@"
