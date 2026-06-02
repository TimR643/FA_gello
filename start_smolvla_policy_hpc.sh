#!/usr/bin/env bash
set -euo pipefail

# HPC launcher for the SmolVLA left/right policy.
# All values can be overridden from the shell, e.g.:
#   CKPT=/path/to/pretrained_model ./start_smolvla_policy_hpc.sh

HPC_USER_HOME="${HPC_USER_HOME:-/home/tim_st179133}"
CONDA_SH="${CONDA_SH:-$HPC_USER_HOME/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-$HPC_USER_HOME/miniconda3/envs/lerobot}"
GELLO_ROOT="${GELLO_ROOT:-$HPC_USER_HOME/gello_software}"

DATASET_REPO_ID="${DATASET_REPO_ID:-TimR643/left_right}"
DATASET_ROOT="${DATASET_ROOT:-$HPC_USER_HOME/lerobot_data/left_right_test}"
CKPT="${CKPT:-$HPC_USER_HOME/lerobot_outputs/smolvla_left_right/checkpoints/040000/pretrained_model}"

ROBOT_HOST="${ROBOT_HOST:-127.0.0.1}"
CAMERA_HOST="${CAMERA_HOST:-$ROBOT_HOST}"
ROBOT_PORT="${ROBOT_PORT:-6001}"
WRIST_CAMERA_PORT="${WRIST_CAMERA_PORT:-5000}"
DURATION="${DURATION:-10.0}"
HZ="${HZ:-2.0}"
MAX_JOINT_DELTA="${MAX_JOINT_DELTA:-0.005}"
MAX_GRIPPER_DELTA="${MAX_GRIPPER_DELTA:-0.01}"
GRIPPER_MODE="${GRIPPER_MODE:-hold}"
MAX_JOINT_DISTANCE_FROM_START="${MAX_JOINT_DISTANCE_FROM_START:-0.25}"
REPLAN_EVERY_STEP="${REPLAN_EVERY_STEP:-1}"
TASK="${TASK:-Move right when the red block is visible, otherwise move left.}"
EXECUTE="${EXECUTE:-1}"
REQUIRE_ENTER="${REQUIRE_ENTER:-1}"

source "$CONDA_SH"
conda activate "$CONDA_ENV"
cd "$GELLO_ROOT"

printf 'DATASET_REPO_ID=%s\nDATASET_ROOT=%s\nCKPT=%s\nGELLO_ROOT=%s\n' \
  "$DATASET_REPO_ID" "$DATASET_ROOT" "$CKPT" "$GELLO_ROOT"

test -f "$DATASET_ROOT/meta/info.json" || { echo "FEHLT: $DATASET_ROOT/meta/info.json"; exit 1; }
test -d "$CKPT" || { echo "FEHLT: $CKPT"; exit 1; }
if [[ ! -f "$CKPT/model.safetensors" && ! -f "$CKPT/pytorch_model.bin" ]]; then
  echo "FEHLT: weder $CKPT/model.safetensors noch $CKPT/pytorch_model.bin gefunden"
  exit 1
fi

cmd=(
  python -u experiments/run_smolvla_real_robot_hpc.py
  --checkpoint "$CKPT"
  --dataset-root "$DATASET_ROOT"
  --repo-id "$DATASET_REPO_ID"
  --robot-host "$ROBOT_HOST"
  --camera-host "$CAMERA_HOST"
  --robot-port "$ROBOT_PORT"
  --wrist-camera-port "$WRIST_CAMERA_PORT"
  --cameras wrist
  --duration "$DURATION"
  --hz "$HZ"
  --max-joint-delta "$MAX_JOINT_DELTA"
  --max-gripper-delta "$MAX_GRIPPER_DELTA"
  --gripper-mode "$GRIPPER_MODE"
  --max-joint-distance-from-start "$MAX_JOINT_DISTANCE_FROM_START"
  --task "$TASK"
)

if [[ "$EXECUTE" == "1" ]]; then
  cmd+=(--execute)
fi
if [[ "$REPLAN_EVERY_STEP" != "1" ]]; then
  cmd+=(--no-replan-every-step)
fi
if [[ "$REQUIRE_ENTER" != "1" ]]; then
  cmd+=(--no-require-enter)
fi

exec "${cmd[@]}"
