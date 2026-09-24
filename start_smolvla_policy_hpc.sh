#!/usr/bin/env bash
set -e

source /home/tim_st179133/miniconda3/etc/profile.d/conda.sh
conda activate /home/tim_st179133/miniconda3/envs/lerobot

cd /home/tim_st179133/gello_software

export DATASET_REPO_ID="TimR643/left_right"
export DATASET_ROOT="/home/tim_st179133/lerobot_data/left_right_test"

# WICHTIG:
# Nimm genau den Checkpoint-Ordner, der bei dir existiert.
# Dein Terminal hat zuletzt diesen Pfad geladen:
export CKPT="/home/tim_st179133/lerobot_outputs/smolvla_left_right/checkpoints/040000/pretrained_model"

echo "DATASET_REPO_ID=$DATASET_REPO_ID"
echo "DATASET_ROOT=$DATASET_ROOT"
echo "CKPT=$CKPT"

test -f "$DATASET_ROOT/meta/info.json" || { echo "FEHLT: $DATASET_ROOT/meta/info.json"; exit 1; }
test -d "$CKPT" || { echo "FEHLT: $CKPT"; exit 1; }
test -f "$CKPT/model.safetensors" || { echo "FEHLT: $CKPT/model.safetensors"; exit 1; }

python experiments/run_smolvla_real_robot_hpc.py \
  --checkpoint "$CKPT" \
  --dataset-root "$DATASET_ROOT" \
  --repo-id "$DATASET_REPO_ID" \
  --robot-host 127.0.0.1 \
  --robot-port 6001 \
  --wrist-camera-port 5000 \
  --cameras wrist \
  --duration 50.0 \
  --hz 10.0 \
  --max-joint-delta 0.2 \
  --max-gripper-delta 1.0 \
  --task "Move right when the red block is visible, otherwise move left." \
  --execute