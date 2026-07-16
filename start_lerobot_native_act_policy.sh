#!/usr/bin/env bash
set -euo pipefail

# Convenience launcher for ACT checkpoints trained with GELLO/LeRobot.
#
# Dieses Skript ist für ACT-Modelle mit zwei Kameras:
#
#   observation.images.wrist
#   observation.images.base
#
# Die echten Live-Kameras sind ebenfalls:
#
#   wrist,base
#
# Für ältere ACT-Modelle mit nur einer Kamera kannst du unten die
# One-Camera-Variante wieder aktivieren.

ACT_CKPT_CANDIDATES=(
  "$HOME/lerobot_outputs/train/push_white_cube_30-30_act/checkpoints/140000/pretrained_model"
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

# ---------------------------------------------------------------------
# ACT Kamera-Konfiguration: zwei Kameras
# ---------------------------------------------------------------------
#
# Für ACT mit wrist + base:
export CAMERA_NAMES="${CAMERA_NAMES:-wrist,base}"
export POLICY_CAMERA_NAMES="${POLICY_CAMERA_NAMES:-wrist,base}"
#
# Für ältere ACT-One-Camera-Modelle stattdessen diese Variante nutzen:
#
# export CAMERA_NAMES="${CAMERA_NAMES:-wrist}"
# export POLICY_CAMERA_NAMES="${POLICY_CAMERA_NAMES:-wrist}"
#
# ---------------------------------------------------------------------

export FPS="${FPS:-8}"
export MAX_JOINT_DELTA="${MAX_JOINT_DELTA:-0.20}"
export MAX_GRIPPER_DELTA="${MAX_GRIPPER_DELTA:-1.00}"
export RETURN_TO_INITIAL_POSITION="${RETURN_TO_INITIAL_POSITION:-false}"
export ACTION_MODE="${ACTION_MODE:-absolute_joint_position}"

exec "$(dirname "$0")/start_lerobot_native_real_policy.sh" "$@"