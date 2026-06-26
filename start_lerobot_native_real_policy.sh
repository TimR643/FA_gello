#!/usr/bin/env bash
set -euo pipefail

# Native LeRobot Bring-Your-Own-Hardware policy rollout for the GELLO/ZMQ Panda stack.
# Start the normal GELLO robot/camera ZMQ servers first, then run this script in
# the LeRobot environment. The first run installs this repo and the LeRobot robot
# plugin in editable mode so the official LeRobot rollout CLI can discover it.

source "${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
conda activate "${LEROBOT_ENV:-$HOME/miniconda3/envs/lerobot}"

REPO_DIR="${GELLO_REPO_DIR:-$HOME/gello_software}"
cd "$REPO_DIR"

export CKPT="${CKPT:-/home/tim_st179133/lerobot_outputs/pick_red_green_lego/checkpoints/last/pretrained_model}"
: "${TASK:=pick up the lego block, and go left if the block is green, go right if the block is red}"
: "${DURATION:=50}"
: "${NUM_EPISODES:=1}"
: "${FPS:=8}"
: "${RETURN_TO_INITIAL_POSITION:=false}"

RECORD_LEROBOT="${RECORD_LEROBOT:-false}"
LOG_ROLLOUT="${LOG_ROLLOUT:-false}"

usage() {
  cat <<EOF
Usage: $0 [--record-lerobot] [--no-record-lerobot] [--log-rollout] [--no-log-rollout]

Environment overrides for LeRobot recording:
  LEROBOT_RECORD_ROOT      Dataset root (default: ~/lerobot_data/native_policy_rollouts)
  LEROBOT_RECORD_REPO_ID   Dataset repo id (default: local/native_policy_rollouts)
  LEROBOT_RECORD_TASK      Task stored in recorded frames (default: TASK)
  LEROBOT_RECORD_ROBOT_TYPE Robot type metadata (default: panda_gello)
  LEROBOT_RECORD_CAMERA_NAMES Camera keys written to dataset (default: CAMERA_NAMES/CAMERAS)
  LEROBOT_RECORD_FPS       Dataset fps (default: FPS)
  ROLLOUT_LOG_DIR          Log directory for --log-rollout (default: logs/rollouts)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --record-lerobot|--record|--lerobot-record)
      RECORD_LEROBOT=true
      ;;
    --no-record-lerobot|--no-record|--no-lerobot-record)
      RECORD_LEROBOT=false
      ;;
    --log-rollout|--logger)
      LOG_ROLLOUT=true
      ;;
    --no-log-rollout|--no-logger)
      LOG_ROLLOUT=false
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
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
  cat >&2 <<EOF
FEHLT: CKPT=$CKPT
CKPT muss auf ein LeRobot pretrained_model-Verzeichnis mit config.json zeigen.
Beispiele:
  .../checkpoints/last/pretrained_model
  .../checkpoints/<step>/pretrained_model
Wenn du nur .../checkpoints/<step> angibst, akzeptiert das Skript das ebenfalls,
sofern darunter pretrained_model/config.json existiert.
EOF
  exit 1
fi
export CKPT

python -m pip install -e .
python -m pip install -e lerobot_robot_gello

if ! command -v lerobot-rollout >/dev/null 2>&1; then
  echo "FEHLT: lerobot-rollout wurde nicht gefunden."
  echo "Dein installiertes LeRobot erkennt zwar robot.type=gello_zmq, aber"
  echo "lerobot-record in LeRobot 0.5.x akzeptiert keine --policy.path Argumente."
  echo "Bitte LeRobot auf eine Version mit lerobot-rollout aktualisieren."
  exit 1
fi

echo "Using native LeRobot rollout CLI with robot.type=gello_zmq"
echo "CKPT=$CKPT"
echo "TASK=$TASK"
echo "DURATION=$DURATION"
echo "NUM_EPISODES=$NUM_EPISODES"
echo "FPS=$FPS"
echo "RETURN_TO_INITIAL_POSITION=$RETURN_TO_INITIAL_POSITION"
echo "CAMERA_NAMES=${CAMERA_NAMES:-${CAMERAS:-wrist,base}}"
echo "POLICY_CAMERA_NAMES=${POLICY_CAMERA_NAMES:-camera1,camera2,camera3}"
echo "RECORD_LEROBOT=$RECORD_LEROBOT"
echo "LOG_ROLLOUT=$LOG_ROLLOUT"
if [[ "$RECORD_LEROBOT" == "true" ]]; then
  echo "LEROBOT_RECORD_ROOT=${LEROBOT_RECORD_ROOT:-$HOME/lerobot_data/native_policy_rollouts}"
  echo "LEROBOT_RECORD_REPO_ID=${LEROBOT_RECORD_REPO_ID:-local/native_policy_rollouts}"
fi

run_rollout_episode() {
  local episode="$1"
  echo "Starting rollout episode $episode/$NUM_EPISODES"
  lerobot-rollout \
    --strategy.type="${STRATEGY_TYPE:-base}" \
    --policy.path="$CKPT" \
    --fps="$FPS" \
    --return_to_initial_position="$RETURN_TO_INITIAL_POSITION" \
    --robot.type=gello_zmq \
    --robot.robot_host="${ROBOT_HOST:-127.0.0.1}" \
    --robot.robot_port="${ROBOT_PORT:-6001}" \
    --robot.camera_host="${CAMERA_HOST:-${ROBOT_HOST:-127.0.0.1}}" \
    --robot.wrist_camera_port="${WRIST_CAMERA_PORT:-5000}" \
    --robot.base_camera_port="${BASE_CAMERA_PORT:-5001}" \
    --robot.zmq_timeout_ms="${ZMQ_TIMEOUT_MS:-3000}" \
    --robot.camera_names="${CAMERA_NAMES:-${CAMERAS:-wrist,base}}" \
    --robot.policy_camera_names="${POLICY_CAMERA_NAMES:-camera1,camera2,camera3}" \
    --robot.max_joint_delta="${MAX_JOINT_DELTA:-1.0}" \
    --robot.max_gripper_delta="${MAX_GRIPPER_DELTA:-1.0}" \
    --robot.action_mode="${ACTION_MODE:-absolute_joint_position}" \
    --robot.record_lerobot="$RECORD_LEROBOT" \
    --robot.lerobot_root="${LEROBOT_RECORD_ROOT:-$HOME/lerobot_data/native_policy_rollouts}" \
    --robot.lerobot_repo_id="${LEROBOT_RECORD_REPO_ID:-local/native_policy_rollouts}" \
    --robot.lerobot_fps="${LEROBOT_RECORD_FPS:-$FPS}" \
    --robot.lerobot_task="${LEROBOT_RECORD_TASK:-$TASK}" \
    --robot.lerobot_robot_type="${LEROBOT_RECORD_ROBOT_TYPE:-panda_gello}" \
    --robot.lerobot_camera_names="${LEROBOT_RECORD_CAMERA_NAMES:-${CAMERA_NAMES:-${CAMERAS:-wrist,base}}}" \
    --task="$TASK" \
    --duration="$DURATION"
}

for episode in $(seq 1 "$NUM_EPISODES"); do
  if [[ "$LOG_ROLLOUT" == "true" ]]; then
    mkdir -p "${ROLLOUT_LOG_DIR:-logs/rollouts}"
    run_rollout_episode "$episode" 2>&1 | tee "${ROLLOUT_LOG_DIR:-logs/rollouts}/native_rollout_$(date +%Y%m%d_%H%M%S)_episode_${episode}.log"
  else
    run_rollout_episode "$episode"
  fi
done
