#!/usr/bin/env bash
set -euo pipefail

# Native LeRobot Bring-Your-Own-Hardware policy rollout for the GELLO/ZMQ Panda stack.
# Start the normal GELLO robot/camera ZMQ servers first, then run this script in
# the LeRobot environment.

source "${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
conda activate "${LEROBOT_ENV:-$HOME/miniconda3/envs/lerobot}"

REPO_DIR="${GELLO_REPO_DIR:-$HOME/gello_software}"
cd "$REPO_DIR"

export CKPT="${CKPT:-/home/tim_st179133/lerobot_outputs/train/go_left_right_even_10eps/checkpoints/040000/pretrained_model}"
: "${TASK:=go right if a red block is detected, go left if a green block is detected}"
: "${DURATION:=50}"
: "${NUM_EPISODES:=1}"
: "${FPS:=8}"
: "${RETURN_TO_INITIAL_POSITION:=false}"
: "${AUTO_CAMERA_CONFIG:=true}"

RECORD_LEROBOT="${RECORD_LEROBOT:-false}"
LOG_ROLLOUT="${LOG_ROLLOUT:-false}"

usage() {
  cat <<EOF_USAGE
Usage: $0 [--record-lerobot] [--no-record-lerobot] [--log-rollout] [--no-log-rollout]

Environment overrides:
  CKPT                         Policy checkpoint path
  TASK                         Task prompt
  DURATION                     Rollout duration in seconds
  NUM_EPISODES                 Number of rollout runs
  FPS                          Control loop FPS
  RETURN_TO_INITIAL_POSITION   true/false
  AUTO_CAMERA_CONFIG           true/false, infer rollout cameras from policy config (default: true)

LeRobot recording:
  LEROBOT_RECORD_ROOT          Dataset root (default: ~/lerobot_data/native_policy_rollouts)
  LEROBOT_RECORD_REPO_ID       Dataset repo id (default: local/native_policy_rollouts)
  LEROBOT_RECORD_TASK          Task stored in dataset (default: TASK)
  LEROBOT_RECORD_FPS           Dataset fps (default: FPS)
  LEROBOT_RECORD_PUSH_TO_HUB   Push dataset to Hugging Face Hub (default: false)

Rollout logging:
  ROLLOUT_LOG_DIR              Log directory for --log-rollout (default: logs/rollouts)

Robot/ZMQ:
  ROBOT_HOST                   default: 127.0.0.1
  ROBOT_PORT                   default: 6001
  CAMERA_HOST                  default: ROBOT_HOST
  WRIST_CAMERA_PORT            default: 5000
  BASE_CAMERA_PORT             default: 5001
  CAMERA_NAMES                 Live GELLO cameras, default inferred from checkpoint then wrist,base
  POLICY_CAMERA_NAMES          Policy image names, default inferred from checkpoint

Example:
  LEROBOT_RECORD_ROOT=/home/tim_st179133/lerobot_inferences/go_left_right_even_10eps \\
  LEROBOT_RECORD_REPO_ID=local/go_left_right_even_10eps \\
  ROLLOUT_LOG_DIR=/home/tim_st179133/logs/go_left_right_even_10eps \\
  $0 --record-lerobot --log-rollout
EOF_USAGE
}

while [[ $# -gt 0 ]]; do
  # Defensive cleanup for copy/paste accidents in terminals (for example a
  # trailing Ctrl-C byte after "--log-rollout"). Without this, the usage text can
  # claim that an otherwise supported argument is unknown.
  arg="$(printf '%s' "$1" | tr -d '[:cntrl:]')"
  case "$arg" in
    --record-lerobot|--record|--lerobot-record) RECORD_LEROBOT=true ;;
    --no-record-lerobot|--no-record|--no-lerobot-record) RECORD_LEROBOT=false ;;
    --log-rollout|--logger) LOG_ROLLOUT=true ;;
    --no-log-rollout|--no-logger) LOG_ROLLOUT=false ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %q\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

resolve_policy_path() {
  local candidate="$1"
  if [[ -f "$candidate/config.json" ]]; then
    printf '%s\n' "$candidate"
    return 0
  fi
  if [[ -f "$candidate/pretrained_model/config.json" ]]; then
    printf '%s\n' "$candidate/pretrained_model"
    return 0
  fi
  return 1
}

if ! CKPT="$(resolve_policy_path "$CKPT")"; then
  cat >&2 <<EOF_MISSING
FEHLT: CKPT=$CKPT

CKPT muss auf ein LeRobot pretrained_model-Verzeichnis mit config.json zeigen.

Beispiele:
  .../checkpoints/last/pretrained_model
  .../checkpoints/<step>/pretrained_model

Wenn du nur .../checkpoints/<step> angibst, akzeptiert das Skript das ebenfalls,
sofern darunter pretrained_model/config.json existiert.
EOF_MISSING
  exit 1
fi
export CKPT

infer_policy_image_names() {
  python - "$CKPT/config.json" <<'PY'
import json
import sys
from pathlib import Path

config = json.loads(Path(sys.argv[1]).read_text())
keys = []

def walk(value):
    if isinstance(value, dict):
        for key, item in value.items():
            if isinstance(key, str) and key.startswith("observation.images."):
                keys.append(key.removeprefix("observation.images."))
            walk(item)
    elif isinstance(value, list):
        for item in value:
            walk(item)

walk(config)
seen = []
for key in keys:
    if key not in seen:
        seen.append(key)
print(",".join(seen))
PY
}

live_cameras_for_policy_names() {
  local policy_names="$1"
  python - "$policy_names" <<'PY'
import sys
policy_names = [name.strip() for name in sys.argv[1].split(',') if name.strip()]
real_names = [name for name in policy_names if not name.startswith('empty_camera')]
if not real_names:
    print('wrist')
elif real_names == ['wrist']:
    print('wrist')
elif real_names == ['base']:
    print('base')
elif set(real_names).issubset({'wrist', 'base'}):
    print(','.join(name for name in ('wrist', 'base') if name in real_names))
elif len(real_names) == 1:
    print('wrist')
else:
    print('wrist,base')
PY
}

INFERRED_POLICY_CAMERA_NAMES="$(infer_policy_image_names)"
if [[ "$AUTO_CAMERA_CONFIG" == "true" && -n "$INFERRED_POLICY_CAMERA_NAMES" ]]; then
  POLICY_CAMERA_NAMES_RESOLVED="${POLICY_CAMERA_NAMES:-$INFERRED_POLICY_CAMERA_NAMES}"
  CAMERA_NAMES_RESOLVED="${CAMERA_NAMES:-${CAMERAS:-$(live_cameras_for_policy_names "$POLICY_CAMERA_NAMES_RESOLVED")}}"
else
  POLICY_CAMERA_NAMES_RESOLVED="${POLICY_CAMERA_NAMES:-camera1,camera2,camera3}"
  CAMERA_NAMES_RESOLVED="${CAMERA_NAMES:-${CAMERAS:-wrist,base}}"
fi
LEROBOT_RECORD_CAMERA_NAMES_RESOLVED="${LEROBOT_RECORD_CAMERA_NAMES:-$CAMERA_NAMES_RESOLVED}"

if ! command -v lerobot-rollout >/dev/null 2>&1; then
  echo "FEHLT: lerobot-rollout wurde nicht gefunden." >&2
  echo "Bitte LeRobot auf eine Version mit lerobot-rollout aktualisieren." >&2
  exit 1
fi

warm_lerobot_imports() {
  echo "Warming up LeRobot rollout imports (datasets/pandas can take a while on first start; do not interrupt)..."
  python - <<'PY'
from lerobot.scripts.lerobot_rollout import main as _main
print("LeRobot rollout imports are ready.")
PY
}

warm_lerobot_imports

if [[ "$RECORD_LEROBOT" == "true" ]]; then
  ROLLOUT_STRATEGY="${STRATEGY_TYPE:-sentry}"
else
  ROLLOUT_STRATEGY="${STRATEGY_TYPE:-base}"
fi

LEROBOT_RECORD_ROOT_RESOLVED="${LEROBOT_RECORD_ROOT:-$HOME/lerobot_data/native_policy_rollouts}"
LEROBOT_RECORD_REPO_ID_RESOLVED="${LEROBOT_RECORD_REPO_ID:-local/native_policy_rollouts}"
LEROBOT_RECORD_TASK_RESOLVED="${LEROBOT_RECORD_TASK:-$TASK}"
LEROBOT_RECORD_FPS_RESOLVED="${LEROBOT_RECORD_FPS:-$FPS}"
LEROBOT_RECORD_PUSH_TO_HUB_RESOLVED="${LEROBOT_RECORD_PUSH_TO_HUB:-false}"
ROLLOUT_LOG_DIR_RESOLVED="${ROLLOUT_LOG_DIR:-logs/rollouts}"

cat <<EOF_CONFIG
Using native LeRobot rollout CLI with robot.type=gello_zmq
CKPT=$CKPT
TASK=$TASK
DURATION=$DURATION
NUM_EPISODES=$NUM_EPISODES
FPS=$FPS
RETURN_TO_INITIAL_POSITION=$RETURN_TO_INITIAL_POSITION
AUTO_CAMERA_CONFIG=$AUTO_CAMERA_CONFIG
INFERRED_POLICY_CAMERA_NAMES=${INFERRED_POLICY_CAMERA_NAMES:-<none>}
CAMERA_NAMES=$CAMERA_NAMES_RESOLVED
POLICY_CAMERA_NAMES=$POLICY_CAMERA_NAMES_RESOLVED
RECORD_LEROBOT=$RECORD_LEROBOT
LOG_ROLLOUT=$LOG_ROLLOUT
ROLLOUT_STRATEGY=$ROLLOUT_STRATEGY
EOF_CONFIG

if [[ "$RECORD_LEROBOT" == "true" ]]; then
  echo "LEROBOT_RECORD_ROOT=$LEROBOT_RECORD_ROOT_RESOLVED"
  echo "LEROBOT_RECORD_REPO_ID=$LEROBOT_RECORD_REPO_ID_RESOLVED"
  echo "LEROBOT_RECORD_TASK=$LEROBOT_RECORD_TASK_RESOLVED"
  echo "LEROBOT_RECORD_FPS=$LEROBOT_RECORD_FPS_RESOLVED"
  echo "LEROBOT_RECORD_PUSH_TO_HUB=$LEROBOT_RECORD_PUSH_TO_HUB_RESOLVED"
  echo "LEROBOT_RECORD_CAMERA_NAMES=$LEROBOT_RECORD_CAMERA_NAMES_RESOLVED"
  mkdir -p "$LEROBOT_RECORD_ROOT_RESOLVED"
fi

if [[ "$LOG_ROLLOUT" == "true" ]]; then
  echo "ROLLOUT_LOG_DIR=$ROLLOUT_LOG_DIR_RESOLVED"
  mkdir -p "$ROLLOUT_LOG_DIR_RESOLVED"
fi

run_rollout_episode() {
  local episode="$1"
  echo "Starting rollout episode $episode/$NUM_EPISODES"

  cmd=(
    lerobot-rollout
    --strategy.type="$ROLLOUT_STRATEGY"
    --policy.path="$CKPT"
    --fps="$FPS"
    --return_to_initial_position="$RETURN_TO_INITIAL_POSITION"
    --robot.type=gello_zmq
    --robot.robot_host="${ROBOT_HOST:-127.0.0.1}"
    --robot.robot_port="${ROBOT_PORT:-6001}"
    --robot.camera_host="${CAMERA_HOST:-${ROBOT_HOST:-127.0.0.1}}"
    --robot.wrist_camera_port="${WRIST_CAMERA_PORT:-5000}"
    --robot.base_camera_port="${BASE_CAMERA_PORT:-5001}"
    --robot.zmq_timeout_ms="${ZMQ_TIMEOUT_MS:-3000}"
    --robot.camera_names="$CAMERA_NAMES_RESOLVED"
    --robot.policy_camera_names="$POLICY_CAMERA_NAMES_RESOLVED"
    --robot.max_joint_delta="${MAX_JOINT_DELTA:-1.0}"
    --robot.max_gripper_delta="${MAX_GRIPPER_DELTA:-1.0}"
    --robot.action_mode="${ACTION_MODE:-absolute_joint_position}"
    --robot.joint_inference_log_dir="${JOINT_INFERENCE_LOG_DIR:-logs/inference_joint_logs}"
    --robot.joint_inference_log_enabled="${JOINT_INFERENCE_LOG_ENABLED:-true}"
    --task="$TASK"
    --duration="$DURATION"
  )

  if [[ "$RECORD_LEROBOT" == "true" ]]; then
    cmd+=(
      --dataset.root="$LEROBOT_RECORD_ROOT_RESOLVED"
      --dataset.repo_id="$LEROBOT_RECORD_REPO_ID_RESOLVED"
      --dataset.single_task="$LEROBOT_RECORD_TASK_RESOLVED"
      --dataset.fps="$LEROBOT_RECORD_FPS_RESOLVED"
      --dataset.push_to_hub="$LEROBOT_RECORD_PUSH_TO_HUB_RESOLVED"
    )
  fi

  "${cmd[@]}"
}

for episode in $(seq 1 "$NUM_EPISODES"); do
  if [[ "$LOG_ROLLOUT" == "true" ]]; then
    run_rollout_episode "$episode" 2>&1 | tee \
      "$ROLLOUT_LOG_DIR_RESOLVED/native_rollout_$(date +%Y%m%d_%H%M%S)_episode_${episode}.log"
  else
    run_rollout_episode "$episode"
  fi
done
