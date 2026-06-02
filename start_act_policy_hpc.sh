#!/usr/bin/env bash
set -e

source /home/tim_st179133/miniconda3/etc/profile.d/conda.sh
conda activate /home/tim_st179133/miniconda3/envs/lerobot

cd /home/tim_st179133/gello_software

export DATASET_REPO_ID="TimR643/left_right"
export DATASET_ROOT="/home/tim_st179133/lerobot_data/left_right_test"

# Passe diesen Pfad an deinen ACT-Checkpoint auf dem HPC an.
# Falls du act_left_right vom Laptop kopiert hast, sollte er ungefähr so liegen:
export CKPT="/home/tim_st179133/lerobot_outputs/train/act_left_right/checkpoints/last/pretrained_model"

echo "DATASET_REPO_ID=$DATASET_REPO_ID"
echo "DATASET_ROOT=$DATASET_ROOT"
echo "CKPT=$CKPT"

test -f "$DATASET_ROOT/meta/info.json" || { echo "FEHLT: $DATASET_ROOT/meta/info.json"; exit 1; }
test -d "$CKPT" || {
  echo "FEHLT: $CKPT"
  echo
  echo "Gefundene ACT-Kandidaten:"
  find /home/tim_st179133/lerobot_outputs -type d -path "*pretrained_model*" 2>/dev/null | grep -Ei "act|left_right|panda|sorting" || true
  exit 1
}

python experiments/run_lerobot_real_robot_act.py \
  --checkpoint "$CKPT" \
  --dataset-root "$DATASET_ROOT" \
  --repo-id "$DATASET_REPO_ID" \
  --robot-host 127.0.0.1 \
  --robot-port 6001 \
  --wrist-camera-port 5000 \
  --cameras wrist \
  --duration 10.0 \
  --hz 2.0 \
  --max-joint-delta 0.005 \
  --max-gripper-delta 0.01 \
  --execute