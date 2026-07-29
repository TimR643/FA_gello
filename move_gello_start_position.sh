#!/usr/bin/env bash
set -euo pipefail

# Move the live GELLO/Panda ZMQ robot to the three-Lego-block start pose without
# starting GELLO. The normal robot ZMQ server / SSH tunnel must already be up.

source "${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
conda activate "${LEROBOT_ENV:-$HOME/miniconda3/envs/lerobot}"

REPO_DIR="${GELLO_REPO_DIR:-$HOME/gello_software}"
cd "$REPO_DIR"

# precision: "0.0419,-0.1546,-0.0701,-2.2596,-0.0772,2.052,-0.7830"
# pick rectangle: -0.0831,-0.1316,-0.1535,-2.4256,-0.0616,2.2407,-0.790
#sorting algorithm: "0.0178,-0.0764,-0.1099,-1.6782,-0.0508,1.5962,+0.7516"
#TARGET_ARGS=(--target-rad "${START_JOINTS_RAD:-0.0419,-0.1546,-0.0701,-2.2596,-0.0772,2.052,-0.7830}")
TARGET_ARGS=(
  --target-rad
  "0.0178,-0.0764,-0.1099,-1.6782,-0.0508,1.5962,+0.7516"
)
if [[ -n "${START_JOINTS_DEG:-}" ]]; then
  TARGET_ARGS=(--target-deg "$START_JOINTS_DEG")
fi

python scripts/move_gello_start_position.py \
  --robot-host "${ROBOT_HOST:-127.0.0.1}" \
  --robot-port "${ROBOT_PORT:-6001}" \
  --timeout-ms "${ZMQ_TIMEOUT_MS:-3000}" \
  "${TARGET_ARGS[@]}" \
  --target-gripper "${START_GRIPPER:-0.1}" \
  --arm-tolerance-rad "${START_ARM_TOLERANCE_RAD:-0.03}" \
  --gripper-tolerance "${START_GRIPPER_TOLERANCE:-0.08}" \
  --steps "${MOVE_STEPS:-80}" \
  --period-s "${MOVE_PERIOD_S:-0.04}" \
  --hold-s "${MOVE_HOLD_S:-0.5}" \
  "$@"
