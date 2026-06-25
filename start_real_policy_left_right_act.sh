#!/usr/bin/env bash
set -e

source /home/tim/miniconda3/etc/profile.d/conda.sh
conda activate /home/tim/miniconda3/envs/lerobot

cd /home/tim/gello_software

export DATASET_REPO_ID="TimR643/left_right"
export DATASET_ROOT="/home/tim/lerobot_data/left_right/left_right_test"
export CKPT="/home/tim/lerobot_outputs/train/act_left_right/checkpoints/last/pretrained_model"

EXTRA_ARGS=()
if [[ "${RECORD_LEROBOT:-0}" == "1" || "${RECORD_LEROBOT:-}" == "true" ]]; then
  EXTRA_ARGS+=(
    --record-lerobot
    --lerobot-record-root "${LEROBOT_RECORD_ROOT:-/home/tim/lerobot_data/act_left_right_rollouts}"
    --lerobot-record-repo-id "${LEROBOT_RECORD_REPO_ID:-local/act_left_right_rollouts}"
    --lerobot-record-task "${LEROBOT_RECORD_TASK:-ACT left-right policy rollout.}"
  )
fi

python experiments/run_lerobot_real_robot_act.py \
  --checkpoint "$CKPT" \
  --dataset-root "$DATASET_ROOT" \
  --repo-id "$DATASET_REPO_ID" \
  --robot-host 127.0.0.1 \
  --robot-port 6001 \
  --wrist-camera-port 5000 \
  --cameras wrist \
  --duration 50.0 \
  --hz 2.0 \
  --max-joint-delta 0.05 \
  --max-gripper-delta 0.01 \
  --execute \
  "${EXTRA_ARGS[@]}"
