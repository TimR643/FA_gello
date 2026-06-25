#!/usr/bin/env bash
set -euo pipefail

# Convenience launcher for ACT checkpoints trained with the legacy GELLO/LeRobot
# feature names.  The ACT checkpoint reported by Tim expects the visual feature
# `observation.images.wrist`, so this wrapper overrides the generic SmolVLA
# camera defaults before delegating to the shared native rollout script.

# Prefer the ACT checkpoint from this task, but fall back to other known ACT
# output layouts on Tim's HPC so running this ACT-specific launcher without a
# CKPT override still starts an ACT model when one is present.
ACT_CKPT_CANDIDATES=(
  "$HOME/lerobot_outputs/smolvla_left_right/checkpoints/100000/pretrained_model"
)

has_lerobot_config() {
  local candidate="$1"
  [[ -f "$candidate/config.json" || -f "$candidate/pretrained_model/config.json" ]]
}

if [[ -z "${CKPT:-}" ]]; then
  for candidate in "${ACT_CKPT_CANDIDATES[@]}"; do
    if has_lerobot_config "$candidate"; then
      export CKPT="$candidate"
      break
    fi
  done
fi

if [[ -z "${CKPT:-}" ]]; then
  export CKPT="${ACT_CKPT_CANDIDATES[0]}"
  cat >&2 <<EOF
WARNUNG: Kein ACT-Checkpoint aus den bekannten Standardpfaden gefunden.
Das ACT-Skript versucht jetzt: $CKPT
Wenn dein ACT-Modell woanders liegt, starte z.B. so:
  CKPT=/pfad/zu/checkpoints/last/pretrained_model ./start_lerobot_native_act_policy.sh
EOF
fi
export CAMERA_NAMES="${CAMERA_NAMES:-wrist}"
export POLICY_CAMERA_NAMES="${POLICY_CAMERA_NAMES:-wrist}"
export FPS="${FPS:-8}"
export MAX_JOINT_DELTA="${MAX_JOINT_DELTA:-0.20}"
export MAX_GRIPPER_DELTA="${MAX_GRIPPER_DELTA:-1.00}"
export RETURN_TO_INITIAL_POSITION="${RETURN_TO_INITIAL_POSITION:-false}"

exec "$(dirname "$0")/start_lerobot_native_real_policy.sh"
