#!/usr/bin/env bash
set -euo pipefail

# Convenience launcher for ACT checkpoints trained with GELLO/LeRobot.
#
# Periodischer Ablauf:
#   1. ACT-Rollout für EPISODE_TIME_S Sekunden
#   2. RESET_TIME_S Sekunden Pause für manuellen Reset
#   3. nächste Episode
#
# Jede Episode startet start_lerobot_native_real_policy.sh separat.
# Dadurch werden Policy und Rollout-Zustand zwischen Episoden vollständig neu geladen.

# "$HOME/lerobot_outputs/train/bottle_task_act/act/140000/pretrained_model"


ACT_CKPT_CANDIDATES=(
  "$HOME/lerobot_outputs/train/sorting_algorithm_3_pos_final1_act/checkpoints/140000/pretrained_model"
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

  CKPT=/pfad/zu/checkpoints/last/pretrained_model $0
EOF
fi

# ---------------------------------------------------------------------
# ACT Kamera-Konfiguration: zwei Kameras
# ---------------------------------------------------------------------

export CAMERA_NAMES="${CAMERA_NAMES:-wrist,base}"
export POLICY_CAMERA_NAMES="${POLICY_CAMERA_NAMES:-wrist,base}"

# Für ein ACT-One-Camera-Modell stattdessen:
#
# export CAMERA_NAMES="${CAMERA_NAMES:-wrist}"
# export POLICY_CAMERA_NAMES="${POLICY_CAMERA_NAMES:-wrist}"

# ---------------------------------------------------------------------
# Robot-/Rollout-Konfiguration
# ---------------------------------------------------------------------

export FPS="${FPS:-8}"
export MAX_JOINT_DELTA="${MAX_JOINT_DELTA:-0.20}"
export MAX_GRIPPER_DELTA="${MAX_GRIPPER_DELTA:-1.00}"
export RETURN_TO_INITIAL_POSITION="${RETURN_TO_INITIAL_POSITION:-false}"
export ACTION_MODE="${ACTION_MODE:-absolute_joint_position}"

# ---------------------------------------------------------------------
# Periodische Ausführung
# ---------------------------------------------------------------------

: "${NUM_EPISODES:=20}"
: "${EPISODE_TIME_S:=35}"
: "${RESET_TIME_S:=15}"
: "${STOP_ON_ERROR:=true}"

REAL_POLICY_SCRIPT="$(dirname "$0")/start_lerobot_native_real_policy.sh"

if [[ ! -x "$REAL_POLICY_SCRIPT" ]]; then
  echo "FEHLT oder nicht ausführbar: $REAL_POLICY_SCRIPT" >&2
  echo "Ausführungsrecht setzen mit:" >&2
  echo "  chmod +x \"$REAL_POLICY_SCRIPT\"" >&2
  exit 1
fi

if ! [[ "$NUM_EPISODES" =~ ^[1-9][0-9]*$ ]]; then
  echo "FEHLT: NUM_EPISODES muss eine positive Ganzzahl sein: $NUM_EPISODES" >&2
  exit 2
fi

python - "$EPISODE_TIME_S" "$RESET_TIME_S" <<'PY_VALIDATE'
import sys

episode_time = float(sys.argv[1])
reset_time = float(sys.argv[2])

if episode_time <= 0:
    raise SystemExit("FEHLT: EPISODE_TIME_S muss größer als 0 sein.")
if reset_time < 0:
    raise SystemExit("FEHLT: RESET_TIME_S darf nicht negativ sein.")
PY_VALIDATE

cat <<EOF

ACT Multi-Episode-Ausführung
----------------------------
Checkpoint:       $CKPT
Kameras:          $CAMERA_NAMES
Policy-Kameras:   $POLICY_CAMERA_NAMES
Episoden:         $NUM_EPISODES
Episodendauer:    $EPISODE_TIME_S s
Reset-Pause:      $RESET_TIME_S s
FPS:              $FPS
Stop bei Fehler:  $STOP_ON_ERROR

Während der Reset-Pause wird kein ACT-Rollout ausgeführt.
Mit Ctrl+C kann die Serie beendet werden.

EOF

for ((episode = 1; episode <= NUM_EPISODES; episode++)); do
  echo
  echo "============================================================"
  echo "Starte ACT-Episode $episode/$NUM_EPISODES"
  echo "Dauer: $EPISODE_TIME_S s"
  echo "============================================================"

  episode_exit=0

  # NUM_EPISODES=1 verhindert eine zweite, verschachtelte Episodenschleife
  # im generischen start_lerobot_native_real_policy.sh.
  if NUM_EPISODES=1 \
     DURATION="$EPISODE_TIME_S" \
     "$REAL_POLICY_SCRIPT" "$@"; then
    echo "ACT-Episode $episode/$NUM_EPISODES erfolgreich abgeschlossen."
  else
    episode_exit=$?
    echo "FEHLER: ACT-Episode $episode/$NUM_EPISODES endete mit Code $episode_exit." >&2

    if [[ "$STOP_ON_ERROR" == "true" ]]; then
      exit "$episode_exit"
    fi
  fi

  if (( episode == NUM_EPISODES )); then
    break
  fi

  echo
  echo "Manueller Reset: $RESET_TIME_S Sekunden bis Episode $((episode + 1))."
  echo "Während dieser Pause kannst du die Szene zurücksetzen."

  sleep "$RESET_TIME_S"
done

echo
echo "Alle $NUM_EPISODES ACT-Episoden wurden abgearbeitet."
