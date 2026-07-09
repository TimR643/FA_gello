#!/usr/bin/env bash
set -euo pipefail

# SmolVLA-specific native LeRobot BYOH rollout for the GELLO/ZMQ Panda stack.
#
# This launcher intentionally differs from start_lerobot_native_real_policy.sh:
# - defaults to non-recording base rollout for stable robot control,
# - defaults to RTC inference, which LeRobot recommends for slow VLA policies,
# - exposes only live policy camera keys through the robot plugin,
# - omits observation.images.empty_camera_* from robot observations so SmolVLA can
#   create its own masked dummy tensors for empty camera slots,
# - uses conservative motion limits by default.

source "${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
conda activate "${LEROBOT_ENV:-$HOME/miniconda3/envs/lerobot}"

REPO_DIR="${GELLO_REPO_DIR:-$HOME/gello_software}"
cd "$REPO_DIR"

: "${CKPT:=$HOME/lerobot_outputs/train/go_left_right_even_10eps_smolvlabase/checkpoints/050000/pretrained_model}"
: "${TASK:=go right if a red block is detected, go left if a green bock is detected}"
: "${DURATION:=50}"
: "${FPS:=10}"
: "${RETURN_TO_INITIAL_POSITION:=false}"
: "${DEVICE:=cuda}"
: "${ROBOT_HOST:=127.0.0.1}"
: "${ROBOT_PORT:=6001}"
: "${CAMERA_HOST:=$ROBOT_HOST}"
: "${WRIST_CAMERA_PORT:=5000}"
: "${BASE_CAMERA_PORT:=5001}"
: "${ZMQ_TIMEOUT_MS:=3000}"
: "${MAX_JOINT_DELTA:=0.2}"
: "${MAX_GRIPPER_DELTA:=1.0}"
: "${ACTION_MODE:=absolute_joint_position}"
: "${INFERENCE_TYPE:=rtc}"
: "${RTC_EXECUTION_HORIZON:=5}"
: "${RTC_MAX_GUIDANCE_WEIGHT:=5.0}"
: "${RTC_PREFIX_ATTENTION_SCHEDULE:=}"
: "${LOG_ACTION_DIAGNOSTICS_EVERY_N:=1}"
: "${JOINT_INFERENCE_LOG_ENABLED:=true}"
: "${JOINT_INFERENCE_LOG_DIR:=logs/smolvla_inference_joint_logs}"
: "${RECORD_LEROBOT:=false}"
: "${INCLUDE_EMPTY_CAMERAS_IN_ROBOT_OBS:=false}"
: "${INCLUDE_UNMAPPED_POLICY_CAMERAS_AS_BLACK:=false}"

usage() {
  cat <<EOF_USAGE
Usage:
  $0 [--record-lerobot] [--sync] [--rtc] [--help]

Environment overrides:
  CKPT       SmolVLA checkpoint/pretrained_model path
  TASK       Task prompt; default fixes the old "green bock" typo
  FPS        Default: 5
  DURATION   Default: 50
  DEVICE     Default: cuda

Camera mapping:
  LIVE_CAMERA_NAMES       Default inferred from live policy keys, usually wrist,base
  POLICY_CAMERA_NAMES     Override policy camera keys exposed by robot plugin
  INCLUDE_EMPTY_CAMERAS_IN_ROBOT_OBS=false
             Keep false to match LeRobot empty_cameras semantics: empty_camera_*
             keys are not emitted by the robot; SmolVLA should mask them itself.
  INCLUDE_UNMAPPED_POLICY_CAMERAS_AS_BLACK=false
             Keep false to avoid silently treating missing real cameras (e.g.
             camera3) as real black images. Set true only if that is what the
             checkpoint was trained with.

Inference:
  INFERENCE_TYPE          rtc or sync. Default: rtc
  RTC_EXECUTION_HORIZON   Default: 10
  RTC_MAX_GUIDANCE_WEIGHT Default: 10.0
  RTC_PREFIX_ATTENTION_SCHEDULE Optional; unset by default

Safety:
  MAX_JOINT_DELTA         Default: 0.2
  MAX_GRIPPER_DELTA       Default: 1.0
  ACTION_MODE             Default: absolute_joint_position
EOF_USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --record-lerobot|--record)
      RECORD_LEROBOT=true
      ;;
    --no-record-lerobot|--no-record)
      RECORD_LEROBOT=false
      ;;
    --sync)
      INFERENCE_TYPE=sync
      ;;
    --rtc)
      INFERENCE_TYPE=rtc
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown argument: %q\n' "$1" >&2
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
  echo "FEHLT: CKPT muss auf ein pretrained_model-Verzeichnis mit config.json zeigen: $CKPT" >&2
  exit 1
fi
export CKPT

infer_policy_image_names() {
  python - "$CKPT/config.json" <<'PY'
import json
import sys
from pathlib import Path

cfg = json.loads(Path(sys.argv[1]).read_text())
input_features = cfg.get("input_features", {})
names = []

def add_from_key(key):
    if isinstance(key, str) and key.startswith("observation.images."):
        name = key.removeprefix("observation.images.")
        if name not in names:
            names.append(name)

for key, feat in input_features.items():
    if key.startswith("observation.images."):
        if isinstance(feat, dict):
            typ = str(feat.get("type", ""))
            if feat.get("type") in ("VISUAL", "FeatureType.VISUAL") or typ.endswith("VISUAL"):
                add_from_key(key)
        else:
            add_from_key(key)

if not names:
    def walk(value):
        if isinstance(value, dict):
            for key, item in value.items():
                add_from_key(key)
                walk(item)
        elif isinstance(value, list):
            for item in value:
                walk(item)
    walk(cfg)

print(",".join(names))
PY
}

filter_policy_camera_names_for_robot() {
  python - "$1" "$INCLUDE_EMPTY_CAMERAS_IN_ROBOT_OBS" "$INCLUDE_UNMAPPED_POLICY_CAMERAS_AS_BLACK" <<'PY'
import sys

names = [name.strip() for name in sys.argv[1].split(",") if name.strip()]
include_empty = sys.argv[2].lower() == "true"
include_unmapped = sys.argv[3].lower() == "true"

# GELLO currently has live wrist/base cameras. SmolVLA rename_map convention:
# camera1 <- wrist, camera2 <- base. camera3 has no live GELLO source here.
live_mappable = {"camera1", "camera2", "wrist", "base"}
selected = []
omitted_empty = []
omitted_unmapped = []
for name in names:
    if name.startswith("empty_camera"):
        if include_empty:
            selected.append(name)
        else:
            omitted_empty.append(name)
    elif name in live_mappable:
        selected.append(name)
    else:
        if include_unmapped:
            selected.append(name)
        else:
            omitted_unmapped.append(name)

print(",".join(selected))
if omitted_empty:
    print("OMITTED_EMPTY=" + ",".join(omitted_empty), file=sys.stderr)
if omitted_unmapped:
    print("OMITTED_UNMAPPED=" + ",".join(omitted_unmapped), file=sys.stderr)
PY
}

infer_live_camera_names_from_policy_names() {
  python - "$1" <<'PY'
import sys
names = [name.strip() for name in sys.argv[1].split(",") if name.strip()]
live = []
for name in names:
    if name in {"camera1", "wrist"} and "wrist" not in live:
        live.append("wrist")
    elif name in {"camera2", "base"} and "base" not in live:
        live.append("base")
print(",".join(live) if live else "wrist")
PY
}

INFERRED_POLICY_IMAGE_NAMES="$(infer_policy_image_names)"
ROBOT_POLICY_CAMERA_NAMES_DEFAULT="$(filter_policy_camera_names_for_robot "$INFERRED_POLICY_IMAGE_NAMES")"
POLICY_CAMERA_NAMES_RESOLVED="${POLICY_CAMERA_NAMES:-$ROBOT_POLICY_CAMERA_NAMES_DEFAULT}"
LIVE_CAMERA_NAMES_RESOLVED="${LIVE_CAMERA_NAMES:-${CAMERA_NAMES:-$(infer_live_camera_names_from_policy_names "$POLICY_CAMERA_NAMES_RESOLVED")}}"

if [[ -z "$POLICY_CAMERA_NAMES_RESOLVED" ]]; then
  echo "FEHLT: Konnte keine Policy-Kameras aus $CKPT/config.json ableiten." >&2
  exit 1
fi

if [[ "$RECORD_LEROBOT" == "true" ]]; then
  ROLLOUT_STRATEGY="${STRATEGY_TYPE:-sentry}"
else
  ROLLOUT_STRATEGY="${STRATEGY_TYPE:-base}"
fi

if [[ "$RECORD_LEROBOT" == "true" && "$INFERENCE_TYPE" == "rtc" ]]; then
  echo "WARNUNG: Recording/Sentry kann SmolVLA stark verlangsamen. Fuer Debugging zuerst ohne --record-lerobot testen." >&2
fi

if [[ "$TASK" == *"bock"* ]]; then
  echo "WARNUNG: TASK enthaelt vermutlich Tippfehler 'bock'. Aktueller TASK: $TASK" >&2
fi

cat <<EOF_CONFIG
Using SmolVLA-focused native LeRobot rollout CLI with robot.type=gello_zmq
CKPT=$CKPT
TASK=$TASK
DURATION=$DURATION
FPS=$FPS
DEVICE=$DEVICE
ROLLOUT_STRATEGY=$ROLLOUT_STRATEGY
INFERENCE_TYPE=$INFERENCE_TYPE
INFERRED_POLICY_IMAGE_NAMES=${INFERRED_POLICY_IMAGE_NAMES:-<none>}
ROBOT_POLICY_CAMERA_NAMES=$POLICY_CAMERA_NAMES_RESOLVED
LIVE_CAMERA_NAMES=$LIVE_CAMERA_NAMES_RESOLVED
INCLUDE_EMPTY_CAMERAS_IN_ROBOT_OBS=$INCLUDE_EMPTY_CAMERAS_IN_ROBOT_OBS
INCLUDE_UNMAPPED_POLICY_CAMERAS_AS_BLACK=$INCLUDE_UNMAPPED_POLICY_CAMERAS_AS_BLACK
MAX_JOINT_DELTA=$MAX_JOINT_DELTA
MAX_GRIPPER_DELTA=$MAX_GRIPPER_DELTA
ACTION_MODE=$ACTION_MODE
JOINT_INFERENCE_LOG_ENABLED=$JOINT_INFERENCE_LOG_ENABLED
JOINT_INFERENCE_LOG_DIR=$JOINT_INFERENCE_LOG_DIR
EOF_CONFIG

cmd=(
  lerobot-rollout
  --strategy.type="$ROLLOUT_STRATEGY"
  --policy.path="$CKPT"
  --fps="$FPS"
  --duration="$DURATION"
  --device="$DEVICE"
  --return_to_initial_position="$RETURN_TO_INITIAL_POSITION"
  --robot.type=gello_zmq
  --robot.robot_host="$ROBOT_HOST"
  --robot.robot_port="$ROBOT_PORT"
  --robot.camera_host="$CAMERA_HOST"
  --robot.wrist_camera_port="$WRIST_CAMERA_PORT"
  --robot.base_camera_port="$BASE_CAMERA_PORT"
  --robot.zmq_timeout_ms="$ZMQ_TIMEOUT_MS"
  --robot.camera_names="$LIVE_CAMERA_NAMES_RESOLVED"
  --robot.policy_camera_names="$POLICY_CAMERA_NAMES_RESOLVED"
  --robot.max_joint_delta="$MAX_JOINT_DELTA"
  --robot.max_gripper_delta="$MAX_GRIPPER_DELTA"
  --robot.action_mode="$ACTION_MODE"
  --robot.log_action_diagnostics_every_n="$LOG_ACTION_DIAGNOSTICS_EVERY_N"
  --robot.joint_inference_log_enabled="$JOINT_INFERENCE_LOG_ENABLED"
  --robot.joint_inference_log_dir="$JOINT_INFERENCE_LOG_DIR"
  --task="$TASK"
)

if [[ "$INFERENCE_TYPE" == "rtc" ]]; then
  cmd+=(
    --inference.type=rtc
    --inference.rtc.execution_horizon="$RTC_EXECUTION_HORIZON"
    --inference.rtc.max_guidance_weight="$RTC_MAX_GUIDANCE_WEIGHT"
  )
  if [[ -n "$RTC_PREFIX_ATTENTION_SCHEDULE" ]]; then
    cmd+=(--inference.rtc.prefix_attention_schedule="$RTC_PREFIX_ATTENTION_SCHEDULE")
  fi
elif [[ "$INFERENCE_TYPE" == "sync" ]]; then
  cmd+=(--inference.type=sync)
else
  echo "Unsupported INFERENCE_TYPE=$INFERENCE_TYPE; expected rtc or sync" >&2
  exit 2
fi

if [[ "$RECORD_LEROBOT" == "true" ]]; then
  : "${INFERENCE_BASE_DIR:=$HOME/lerobot_inferences}"
  RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
  MODEL_NAME="$(python - "$CKPT" <<'PY'
import sys
from pathlib import Path
p = Path(sys.argv[1]).resolve()
parts = p.parts
if "train" in parts:
    i = parts.index("train")
    if i + 1 < len(parts):
        print(parts[i + 1])
        raise SystemExit
if "checkpoints" in parts:
    i = parts.index("checkpoints")
    if i - 1 >= 0:
        print(parts[i - 1])
        raise SystemExit
print(p.name)
PY
  )"
  LEROBOT_RECORD_ROOT_RESOLVED="${LEROBOT_RECORD_ROOT:-$INFERENCE_BASE_DIR/${MODEL_NAME}_smolvla_inference_${RUN_STAMP}}"
  LEROBOT_RECORD_REPO_ID_RESOLVED="${LEROBOT_RECORD_REPO_ID:-local/rollout_${MODEL_NAME}_smolvla_inference_${RUN_STAMP}}"
  LEROBOT_RECORD_FPS_RESOLVED="${LEROBOT_RECORD_FPS:-$FPS}"
  LEROBOT_RECORD_PUSH_TO_HUB_RESOLVED="${LEROBOT_RECORD_PUSH_TO_HUB:-false}"
  mkdir -p "$(dirname "$LEROBOT_RECORD_ROOT_RESOLVED")"
  cmd+=(
    --dataset.root="$LEROBOT_RECORD_ROOT_RESOLVED"
    --dataset.repo_id="$LEROBOT_RECORD_REPO_ID_RESOLVED"
    --dataset.single_task="$TASK"
    --dataset.fps="$LEROBOT_RECORD_FPS_RESOLVED"
    --dataset.push_to_hub="$LEROBOT_RECORD_PUSH_TO_HUB_RESOLVED"
  )
fi

printf 'Executing:'
printf ' %q' "${cmd[@]}"
printf '\n'

"${cmd[@]}"