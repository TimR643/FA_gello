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

: "${CKPT:=$HOME/lerobot_outputs/train/sorting_algorithm_cubes_smolvlabase/checkpoints/060000/pretrained_model}"
# Push the white cube straight forward.
# Push the white cube forward, then guide it right through the opening.
# put the peg in the designated hole
#Put all red objects into the red box and all other objects into the white box.
: "${TASK:=Put all red objects into the red box and all other objects into the white box.}"
: "${DURATION:=150}"
: "${NUM_EPISODES:=1}"
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
: "${RTC_EXECUTION_HORIZON:=8}"
: "${RTC_MAX_GUIDANCE_WEIGHT:=5.0}"
: "${RTC_PREFIX_ATTENTION_SCHEDULE:=}"
: "${LOG_ACTION_DIAGNOSTICS_EVERY_N:=1}"
: "${RECORD_LEROBOT:=false}"
: "${H5_LOG:=false}"
: "${H5_DELETE_INTERMEDIATE_CSV:=true}"
: "${INCLUDE_EMPTY_CAMERAS_IN_ROBOT_OBS:=false}"
: "${INCLUDE_UNMAPPED_POLICY_CAMERAS_AS_BLACK:=false}"

usage() {
  cat <<EOF_USAGE
Usage:
  $0 [--record-lerobot] [--h5-log] [--sync] [--rtc] [--help]

Environment overrides:
  CKPT       SmolVLA checkpoint/pretrained_model path
  TASK       Task prompt; default fixes the old "green bock" typo
  FPS        Default: 10
  DURATION   Default: 150
  NUM_EPISODES Number of rollout episodes; default: 1
  DEVICE     Default: cuda

H5 logging:
  H5_LOG_DIR                   Output directory
                               default: ~/lerobot_inferences/h5/MODEL_NAME
  H5_LOG_BASENAME              Filename without .h5
  H5_DELETE_INTERMEDIATE_CSV   Delete temporary CSV after conversion; default: true
  JOINT_INFERENCE_LOG_DIR      Temporary CSV directory
  JOINT_INFERENCE_LOG_ENABLED  Normally derived from --h5-log

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
    --h5-log|--hdf5-log)
      H5_LOG=true
      ;;
    --no-h5-log|--no-hdf5-log)
      H5_LOG=false
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

infer_model_name_from_ckpt() {
  python - "$CKPT" <<'PY'
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

MODEL_NAME="$(infer_model_name_from_ckpt)"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
INFERENCE_BASE_DIR="${INFERENCE_BASE_DIR:-$HOME/lerobot_inferences}"

LEROBOT_RECORD_ROOT_RESOLVED="${LEROBOT_RECORD_ROOT:-$INFERENCE_BASE_DIR/${MODEL_NAME}_smolvla_inference_${RUN_STAMP}}"
LEROBOT_RECORD_REPO_ID_RESOLVED="${LEROBOT_RECORD_REPO_ID:-local/rollout_${MODEL_NAME}_smolvla_inference_${RUN_STAMP}}"
LEROBOT_RECORD_FPS_RESOLVED="${LEROBOT_RECORD_FPS:-$FPS}"
LEROBOT_RECORD_PUSH_TO_HUB_RESOLVED="${LEROBOT_RECORD_PUSH_TO_HUB:-false}"

H5_LOG_DIR_RESOLVED="${H5_LOG_DIR:-$INFERENCE_BASE_DIR/h5/${MODEL_NAME}}"
H5_LOG_BASENAME="${H5_LOG_BASENAME:-${MODEL_NAME}_smolvla_inference_${RUN_STAMP}}"
H5_DELETE_INTERMEDIATE_CSV_RESOLVED="${H5_DELETE_INTERMEDIATE_CSV:-true}"

if [[ "$H5_LOG" == "true" ]]; then
  JOINT_INFERENCE_LOG_DIR_RESOLVED="${JOINT_INFERENCE_LOG_DIR:-$INFERENCE_BASE_DIR/joint_csv/${MODEL_NAME}_smolvla_inference_${RUN_STAMP}}"
else
  JOINT_INFERENCE_LOG_DIR_RESOLVED="${JOINT_INFERENCE_LOG_DIR:-logs/smolvla_inference_joint_logs}"
fi
JOINT_INFERENCE_LOG_ENABLED_RESOLVED="${JOINT_INFERENCE_LOG_ENABLED:-$H5_LOG}"

cat <<EOF_CONFIG
Using SmolVLA-focused native LeRobot rollout CLI with robot.type=gello_zmq
CKPT=$CKPT
MODEL_NAME=$MODEL_NAME
TASK=$TASK
DURATION=$DURATION
NUM_EPISODES=$NUM_EPISODES
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
RECORD_LEROBOT=$RECORD_LEROBOT
H5_LOG=$H5_LOG
JOINT_INFERENCE_LOG_ENABLED=$JOINT_INFERENCE_LOG_ENABLED_RESOLVED
JOINT_INFERENCE_LOG_DIR=$JOINT_INFERENCE_LOG_DIR_RESOLVED
EOF_CONFIG

if [[ "$RECORD_LEROBOT" == "true" ]]; then
  echo "LEROBOT_RECORD_ROOT=$LEROBOT_RECORD_ROOT_RESOLVED"
  echo "LEROBOT_RECORD_REPO_ID=$LEROBOT_RECORD_REPO_ID_RESOLVED"
  mkdir -p "$(dirname "$LEROBOT_RECORD_ROOT_RESOLVED")"
fi

if [[ "$H5_LOG" == "true" ]]; then
  echo "H5_LOG_DIR=$H5_LOG_DIR_RESOLVED"
  echo "H5_LOG_BASENAME=$H5_LOG_BASENAME"
  echo "H5_DELETE_INTERMEDIATE_CSV=$H5_DELETE_INTERMEDIATE_CSV_RESOLVED"
  mkdir -p "$H5_LOG_DIR_RESOLVED"
  mkdir -p "$JOINT_INFERENCE_LOG_DIR_RESOLVED"

  python - <<'PY'
import importlib.util
missing = [name for name in ["pandas", "h5py", "numpy"] if importlib.util.find_spec(name) is None]
if missing:
    raise SystemExit(
        "FEHLT: Python-Pakete für H5-Logging fehlen: "
        + ", ".join(missing)
        + "\nInstalliere einmalig mit: python -m pip install "
        + " ".join(missing)
    )
PY
fi

convert_latest_joint_csv_to_h5() {
  local episode="$1"

  if [[ "$H5_LOG" != "true" ]]; then
    return 0
  fi

  local latest_csv
  latest_csv="$(ls -t "$JOINT_INFERENCE_LOG_DIR_RESOLVED"/*.csv 2>/dev/null | head -n 1 || true)"

  if [[ -z "$latest_csv" ]]; then
    echo "WARNUNG: Kein Joint-CSV für H5-Konvertierung gefunden in $JOINT_INFERENCE_LOG_DIR_RESOLVED" >&2
    return 0
  fi

  # Wie bei act_real.h5 werden alle Episoden eines Laufs in eine gemeinsame
  # Datei geschrieben. frame_index und timestamps beginnen pro Episode bei 0.
  local h5_path="$H5_LOG_DIR_RESOLVED/${H5_LOG_BASENAME}.h5"

  python - \
    "$latest_csv" \
    "$h5_path" \
    "$CKPT" \
    "$TASK" \
    "$FPS" \
    "$episode" \
    "$LEROBOT_RECORD_REPO_ID_RESOLVED" <<'PY'
import sys
from pathlib import Path

import h5py
import numpy as np
import pandas as pd

csv_path = Path(sys.argv[1])
h5_path = Path(sys.argv[2])
ckpt = sys.argv[3]
task = sys.argv[4]
fps = float(sys.argv[5])
episode_number = int(sys.argv[6])
dataset_repo_id = sys.argv[7]

df = pd.read_csv(csv_path)
h5_path.parent.mkdir(parents=True, exist_ok=True)

if df.empty:
    raise RuntimeError(f"CSV enthält keine Messwerte: {csv_path}")

NUM_ARM_JOINTS = 7
NUM_DOFS = 8
GRIPPER_SOURCE_INDEX = 7

def numeric_column(*names: str) -> np.ndarray:
    """Read the first available numeric CSV column from names."""
    for name in names:
        if name in df.columns:
            values = pd.to_numeric(df[name], errors="coerce").to_numpy(
                dtype=np.float64
            )
            if values.shape != (len(df),):
                raise RuntimeError(f"Ungültige Spaltenform für {name}: {values.shape}")
            return values
    raise KeyError(
        "Keine der erwarteten CSV-Spalten vorhanden: " + ", ".join(names)
    )

def optional_numeric_column(*names: str, fill: float = np.nan) -> np.ndarray:
    for name in names:
        if name in df.columns:
            return pd.to_numeric(df[name], errors="coerce").to_numpy(
                dtype=np.float64
            )
    return np.full(len(df), fill, dtype=np.float64)

# ------------------------------------------------------------------
# Relative Zeit pro Episode
# ------------------------------------------------------------------
if "time_s" in df.columns:
    timestamps = pd.to_numeric(df["time_s"], errors="coerce").to_numpy(
        dtype=np.float64
    )
    if (
        not np.all(np.isfinite(timestamps))
        or np.any(np.diff(timestamps) < 0.0)
    ):
        timestamps = np.arange(len(df), dtype=np.float64) / fps
    else:
        timestamps = timestamps - timestamps[0]
else:
    timestamps = np.arange(len(df), dtype=np.float64) / fps

frame_index = np.arange(len(df), dtype=np.int64)

# ------------------------------------------------------------------
# 7 Panda-Armachsen + Joint 7 als expliziter Greifer
# ------------------------------------------------------------------
arm_positions = [
    numeric_column(f"joint_{index}_position_rad")
    for index in range(NUM_ARM_JOINTS)
]
gripper_state = numeric_column(
    "gripper_state",
    f"joint_{GRIPPER_SOURCE_INDEX}_position_rad",
)

arm_velocities = [
    numeric_column(f"joint_{index}_velocity_rad_s")
    for index in range(NUM_ARM_JOINTS)
]
gripper_velocity = numeric_column(
    "gripper_velocity",
    f"joint_{GRIPPER_SOURCE_INDEX}_velocity_rad_s",
)

arm_torques = [
    optional_numeric_column(f"joint_{index}_torque_nm")
    for index in range(NUM_ARM_JOINTS)
]
gripper_torque = optional_numeric_column(
    "gripper_torque",
    f"joint_{GRIPPER_SOURCE_INDEX}_torque_nm",
)

# act_real.h5 enthält diese drei Felder. Da hier keine Bildstörung injiziert
# wird, ist die korrekte Stärke jeweils 0.
noise_strength = np.zeros(len(df), dtype=np.float64)
blur_strength = np.zeros(len(df), dtype=np.float64)
brightness_strength = np.zeros(len(df), dtype=np.float64)

feature_names = (
    [f"joint_pos_{index}" for index in range(1, NUM_ARM_JOINTS + 1)]
    + ["gripper_state"]
    + [f"joint_vel_{index}" for index in range(1, NUM_ARM_JOINTS + 1)]
    + ["gripper_velocity"]
    + [f"joint_torque_{index}" for index in range(1, NUM_ARM_JOINTS + 1)]
    + ["gripper_torque"]
    + ["noise_strength", "blur_strength", "brightness_strength"]
)

features = np.column_stack(
    arm_positions
    + [gripper_state]
    + arm_velocities
    + [gripper_velocity]
    + arm_torques
    + [gripper_torque]
    + [noise_strength, blur_strength, brightness_strength]
).astype(np.float64, copy=False)

if features.shape != (len(df), len(feature_names)):
    raise RuntimeError(
        f"Feature-Form stimmt nicht: {features.shape}, "
        f"erwartet {(len(df), len(feature_names))}"
    )

# In act_real.h5 ist labels pro Episode konstant. Hier verwenden wir dafür
# den nullbasierten Episodenindex. Es existiert kein Klassifikator für
# predicted_labels; -1 bedeutet deshalb ausdrücklich 'nicht verfügbar'.
labels = np.full(len(df), episode_number - 1, dtype=np.int64)
predicted_labels = np.full(len(df), -1, dtype=np.int64)

def create_or_append(
    h5_file: h5py.File,
    name: str,
    values: np.ndarray,
) -> h5py.Dataset:
    values = np.asarray(values)
    if name not in h5_file:
        maxshape = (None,) + values.shape[1:]
        dataset = h5_file.create_dataset(
            name,
            data=values,
            maxshape=maxshape,
            chunks=True,
        )
        return dataset

    dataset = h5_file[name]
    if dataset.shape[1:] != values.shape[1:]:
        raise RuntimeError(
            f"{name}: bestehende Form {dataset.shape[1:]} passt nicht zu "
            f"{values.shape[1:]}"
        )
    old_length = dataset.shape[0]
    dataset.resize(old_length + len(values), axis=0)
    dataset[old_length:] = values
    return dataset

mode = "a" if h5_path.exists() else "w"
with h5py.File(h5_path, mode) as f:
    if "features" in f:
        existing_names = [
            value.decode("utf-8") if isinstance(value, bytes) else str(value)
            for value in f["features"].attrs["feature_names"]
        ]
        if existing_names != feature_names:
            raise RuntimeError(
                "Feature-Namen der bestehenden H5-Datei stimmen nicht überein.\n"
                f"Bestehend: {existing_names}\nNeu: {feature_names}"
            )

    features_dset = create_or_append(f, "features", features)
    create_or_append(f, "frame_index", frame_index)
    create_or_append(f, "labels", labels)
    create_or_append(f, "predicted_labels", predicted_labels)
    create_or_append(f, "timestamps", timestamps)

    string_dtype = h5py.string_dtype(encoding="utf-8")
    features_dset.attrs["feature_names"] = np.asarray(
        feature_names, dtype=string_dtype
    )

    # Metadaten analog zu act_real.h5 plus Roboter-spezifische Angaben.
    f.attrs["dataset_repo_id"] = dataset_repo_id
    f.attrs["fault_mode"] = "none"
    f.attrs["fault_strength_steps"] = 0
    f.attrs["fault_strength_strategy"] = "none"
    f.attrs["frame_alignment"] = (
        "frame_index resets to zero and aligns with each episode"
    )
    f.attrs["joint_position_unit"] = "radians"
    f.attrs["joint_velocity_unit"] = "radians_per_second"
    f.attrs["joint_torque_unit"] = "newton_meter"
    f.attrs["timestamp_origin"] = "episode_start"
    f.attrs["timestamp_unit"] = "seconds"
    f.attrs["num_arm_joints"] = NUM_ARM_JOINTS
    f.attrs["num_dofs"] = NUM_DOFS
    f.attrs["gripper_source_joint_index"] = GRIPPER_SOURCE_INDEX
    f.attrs["gripper_state_definition"] = (
        "normalized gripper width: 0=closed, 1=fully open"
    )
    f.attrs["labels_semantics"] = "zero_based_episode_index"
    f.attrs["predicted_labels_semantics"] = "-1 means unavailable"
    f.attrs["checkpoint"] = ckpt
    f.attrs["task"] = task
    f.attrs["fps"] = fps

print(
    f"H5 aktualisiert: {h5_path} | Episode {episode_number} | "
    f"{len(df)} Frames | features={features.shape[1]}"
)
PY

  if [[ "$H5_DELETE_INTERMEDIATE_CSV_RESOLVED" == "true" ]]; then
    rm -f "$latest_csv"
    echo "Temporäres Joint-CSV gelöscht: $latest_csv"
  fi
}

run_rollout_episode() {
  local episode="$1"
  echo "Starting SmolVLA rollout episode $episode/$NUM_EPISODES"

  local cmd=(
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
    --robot.joint_inference_log_enabled="$JOINT_INFERENCE_LOG_ENABLED_RESOLVED"
    --robot.joint_inference_log_dir="$JOINT_INFERENCE_LOG_DIR_RESOLVED"
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
    return 2
  fi

  if [[ "$RECORD_LEROBOT" == "true" ]]; then
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

  local exit_code=0
  "${cmd[@]}" || exit_code=$?

  # Auch nach Ctrl+C oder einem Rollout-Fehler konvertieren, falls ein CSV existiert.
  convert_latest_joint_csv_to_h5 "$episode"
  return "$exit_code"
}

for episode in $(seq 1 "$NUM_EPISODES"); do
  run_rollout_episode "$episode"
done
