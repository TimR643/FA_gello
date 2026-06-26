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
: "${RECORD_ROLLOUT:=false}"
: "${LEROBOT_PUSH_TO_HUB:=false}"
: "${LEROBOT_RESUME:=false}"
: "${RESET_TIME_S:=0}"
: "${EPISODIC_RESET_TO_INITIAL_POSITION:=$RETURN_TO_INITIAL_POSITION}"

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
echo "RECORD_ROLLOUT=$RECORD_ROLLOUT"

common_args=(
  --policy.path="$CKPT"
  --fps="$FPS"
  --robot.type=gello_zmq
  --robot.robot_host="${ROBOT_HOST:-127.0.0.1}"
  --robot.robot_port="${ROBOT_PORT:-6001}"
  --robot.camera_host="${CAMERA_HOST:-${ROBOT_HOST:-127.0.0.1}}"
  --robot.wrist_camera_port="${WRIST_CAMERA_PORT:-5000}"
  --robot.base_camera_port="${BASE_CAMERA_PORT:-5001}"
  --robot.zmq_timeout_ms="${ZMQ_TIMEOUT_MS:-3000}"
  --robot.camera_names="${CAMERA_NAMES:-${CAMERAS:-wrist,base}}"
  --robot.policy_camera_names="${POLICY_CAMERA_NAMES:-camera1,camera2,camera3}"
  --robot.max_joint_delta="${MAX_JOINT_DELTA:-1.0}"
  --robot.max_gripper_delta="${MAX_GRIPPER_DELTA:-1.0}"
  --robot.action_mode="${ACTION_MODE:-absolute_joint_position}"
  --task="$TASK"
)

record_enabled="$(printf '%s' "$RECORD_ROLLOUT" | tr '[:upper:]' '[:lower:]')"
if [[ "$record_enabled" == "1" || "$record_enabled" == "true" || "$record_enabled" == "yes" || "$record_enabled" == "on" ]]; then
  : "${LEROBOT_REPO_ID:?Set LEROBOT_REPO_ID, for example LEROBOT_REPO_ID=local/eval_pick_red_green_lego}"
  : "${LEROBOT_ROOT:?Set LEROBOT_ROOT to the local LeRobot dataset directory}"
  echo "Recording rollout episodes into LeRobot dataset"
  echo "LEROBOT_ROOT=$LEROBOT_ROOT"
  echo "LEROBOT_REPO_ID=$LEROBOT_REPO_ID"
  echo "LEROBOT_PUSH_TO_HUB=$LEROBOT_PUSH_TO_HUB"
  echo "LEROBOT_RESUME=$LEROBOT_RESUME"
  lerobot-rollout \
    "${common_args[@]}" \
    --strategy.type="${STRATEGY_TYPE:-episodic}" \
    --strategy.reset_to_initial_position="$EPISODIC_RESET_TO_INITIAL_POSITION" \
    --dataset.root="$LEROBOT_ROOT" \
    --dataset.repo_id="$LEROBOT_REPO_ID" \
    --dataset.single_task="$TASK" \
    --dataset.num_episodes="$NUM_EPISODES" \
    --dataset.episode_time_s="$DURATION" \
    --dataset.reset_time_s="$RESET_TIME_S" \
    --dataset.push_to_hub="$LEROBOT_PUSH_TO_HUB" \
    --resume="$LEROBOT_RESUME"
else
  for episode in $(seq 1 "$NUM_EPISODES"); do
    echo "Starting rollout episode $episode/$NUM_EPISODES"
    lerobot-rollout \
      "${common_args[@]}" \
      --strategy.type="${STRATEGY_TYPE:-base}" \
      --return_to_initial_position="$RETURN_TO_INITIAL_POSITION" \
      --duration="$DURATION"
  done
fi
