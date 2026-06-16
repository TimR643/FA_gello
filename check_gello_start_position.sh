#!/usr/bin/env bash
set -euo pipefail

# Check whether the live GELLO/Panda ZMQ state is close to the desired recording
# start pose. Start the normal GELLO robot ZMQ server / SSH tunnel first.

source "${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
conda activate "${LEROBOT_ENV:-$HOME/miniconda3/envs/lerobot}"

REPO_DIR="${GELLO_REPO_DIR:-$HOME/gello_software}"
cd "$REPO_DIR"

TARGET_ARGS=(--target-rad "${START_JOINTS_RAD:--0.0905,-0.0038,-0.0327,-2.2140,-0.0262,2.1220,-0.9494}")
if [[ -n "${START_JOINTS_DEG:-}" ]]; then
  TARGET_ARGS=(--target-deg "$START_JOINTS_DEG")
fi

python scripts/check_gello_start_position.py \
  --robot-host "${ROBOT_HOST:-127.0.0.1}" \
  --robot-port "${ROBOT_PORT:-6001}" \
  --timeout-ms "${ZMQ_TIMEOUT_MS:-3000}" \
  "${TARGET_ARGS[@]}" \
  --target-gripper "${START_GRIPPER:-0.8868}" \
  --arm-tolerance-rad "${START_ARM_TOLERANCE_RAD:-0.035}" \
  --gripper-tolerance "${START_GRIPPER_TOLERANCE:-0.08}" \
  "$@"
