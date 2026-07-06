#!/usr/bin/env bash
set -euo pipefail

# Native LeRobot Bring-Your-Own-Hardware policy rollout for the GELLO/ZMQ Panda stack.
# Start the normal GELLO robot/camera ZMQ servers first, then run this script in
# the LeRobot environment.

source "${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
conda activate "${LEROBOT_ENV:-$HOME/miniconda3/envs/lerobot}"

REPO_DIR="${GELLO_REPO_DIR:-$HOME/gello_software}"
cd "$REPO_DIR"

export CKPT="${CKPT:-/home/tim_st179133/lerobot_outputs/train/go_left_right_even_10eps/checkpoints/last/pretrained_model}"
: "${TASK:=go right if a red block is detected, go left if a green bock is detected}"
: "${DURATION:=50}"
: "${NUM_EPISODES:=1}"
: "${FPS:=10}"
: "${RETURN_TO_INITIAL_POSITION:=false}"
: "${AUTO_CAMERA_CONFIG:=true}"

RECORD_LEROBOT="${RECORD_LEROBOT:-false}"
H5_LOG="${H5_LOG:-false}"

usage() {
  cat <<EOF_USAGE
Usage:
  $0 [--record-lerobot] [--no-record-lerobot] [--h5-log] [--no-h5-log]

Examples:
  $0

  $0 --h5-log

  $0 --record-lerobot

  $0 --record-lerobot --h5-log

Environment overrides:
  CKPT                         Policy checkpoint path
  TASK                         Task prompt
  DURATION                     Rollout duration in seconds
  NUM_EPISODES                 Number of rollout runs
  FPS                          Control loop FPS
  RETURN_TO_INITIAL_POSITION   true/false
  AUTO_CAMERA_CONFIG           true/false, infer rollout cameras from policy config

LeRobot recording:
  INFERENCE_BASE_DIR           Base folder for inference datasets and H5 logs
                               default: ~/lerobot_inferences
  LEROBOT_RECORD_ROOT          Exact dataset root
                               default: INFERENCE_BASE_DIR/MODEL_NAME_TIMESTAMP
  LEROBOT_RECORD_REPO_ID       Dataset repo id
                               default: local/rollout_MODEL_NAME_TIMESTAMP
  LEROBOT_RECORD_TASK          Task stored in recorded rollout dataset
                               default: TASK
  LEROBOT_RECORD_FPS           Dataset fps
                               default: FPS
  LEROBOT_RECORD_PUSH_TO_HUB   Push dataset to Hugging Face Hub
                               default: false
  LEROBOT_RECORD_CAMERA_NAMES  Camera names stored in recorded rollout dataset
                               default: CAMERA_NAMES

H5 logging:
  H5_LOG_DIR                   H5 output directory
                               default: INFERENCE_BASE_DIR/h5/MODEL_NAME
  H5_LOG_BASENAME              H5 filename prefix
                               default: MODEL_NAME_TIMESTAMP
  H5_DELETE_INTERMEDIATE_CSV   Delete temporary joint CSV after H5 conversion
                               default: true
  JOINT_INFERENCE_LOG_DIR      Temporary CSV source directory from gello_zmq
                               default with --h5-log:
                                 INFERENCE_BASE_DIR/joint_csv/MODEL_NAME_TIMESTAMP
                               default without --h5-log:
                                 logs/inference_joint_logs
  JOINT_INFERENCE_LOG_ENABLED  Enable/disable temporary gello_zmq joint CSV
                               default: true with --h5-log, false without --h5-log

Robot/ZMQ:
  ROBOT_HOST                   default: 127.0.0.1
  ROBOT_PORT                   default: 6001
  CAMERA_HOST                  default: ROBOT_HOST
  WRIST_CAMERA_PORT            default: 5000
  BASE_CAMERA_PORT             default: 5001
  CAMERA_NAMES                 Live GELLO cameras, e.g. wrist,base
  POLICY_CAMERA_NAMES          Policy image names, e.g.
                                 wrist,base,empty_camera_0,empty_camera_1
                               or
                                 camera1,camera2,empty_camera_0
  MAX_JOINT_DELTA              default: 0.2
  MAX_GRIPPER_DELTA            default: 1.0
  ACTION_MODE                  default: absolute_joint_position
  ZMQ_TIMEOUT_MS               default: 3000

Example with LeRobot recording and H5 logging:
  INFERENCE_BASE_DIR=/home/tim_st179133/lerobot_inferences \\
  $0 --record-lerobot --h5-log
EOF_USAGE
}

while [[ $# -gt 0 ]]; do
  arg="$(printf '%s' "$1" | tr -d '[:cntrl:]')"
  case "$arg" in
    --record-lerobot|--record|--lerobot-record)
      RECORD_LEROBOT=true
      ;;
    --no-record-lerobot|--no-record|--no-lerobot-record)
      RECORD_LEROBOT=false
      ;;
    --h5-log|--hdf5-log)
      H5_LOG=true
      ;;
    --no-h5-log|--no-hdf5-log)
      H5_LOG=false
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

infer_model_name_from_ckpt() {
  python - "$CKPT" <<'PY'
import sys
from pathlib import Path

p = Path(sys.argv[1]).resolve()
parts = p.parts

# Typischer Pfad:
# /home/.../lerobot_outputs/train/MODELLNAME/checkpoints/last/pretrained_model
if "train" in parts:
    i = parts.index("train")
    if i + 1 < len(parts):
        print(parts[i + 1])
        raise SystemExit

# Fallback: Ordner direkt vor "checkpoints"
if "checkpoints" in parts:
    i = parts.index("checkpoints")
    if i - 1 >= 0:
        print(parts[i - 1])
        raise SystemExit

# Letzter Fallback
print(p.name)
PY
}

infer_policy_camera_names() {
  python - "$CKPT/config.json" <<'PY'
import json
import sys
from pathlib import Path

cfg = json.loads(Path(sys.argv[1]).read_text())
input_features = cfg.get("input_features", {})

names = []

for key, feat in input_features.items():
    if key.startswith("observation.images."):
        if isinstance(feat, dict):
            if feat.get("type") in ("VISUAL", "FeatureType.VISUAL") or str(feat.get("type")).endswith("VISUAL"):
                names.append(key.removeprefix("observation.images."))
        else:
            names.append(key.removeprefix("observation.images."))

# Fallback: falls input_features anders/nested gespeichert ist.
if not names:
    def walk(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if isinstance(key, str) and key.startswith("observation.images."):
                    names.append(key.removeprefix("observation.images."))
                walk(item)
        elif isinstance(value, list):
            for item in value:
                walk(item)

    walk(cfg)

seen = []
for name in names:
    if name not in seen:
        seen.append(name)

print(",".join(seen))
PY
}

infer_live_camera_names() {
  local policy_names="$1"

  python - "$policy_names" <<'PY'
import sys

policy_names = [name.strip() for name in sys.argv[1].split(",") if name.strip()]
real_policy_names = [name for name in policy_names if not name.startswith("empty_camera")]

# Fall 1: Policy erwartet echte GELLO-Namen.
# Beispiel:
#   wrist,base,empty_camera_0,empty_camera_1
if any(name in {"wrist", "base"} for name in real_policy_names):
    live = []
    if "wrist" in real_policy_names:
        live.append("wrist")
    if "base" in real_policy_names:
        live.append("base")
    print(",".join(live) if live else "wrist")
    raise SystemExit

# Fall 2: SmolVLA-Base-Layout nach rename_map.
# Beispiel:
#   camera1,camera2,empty_camera_0
#
# Annahme:
#   camera1 <- wrist
#   camera2 <- base
if any(name.startswith("camera") for name in real_policy_names):
    live = []
    if "camera1" in real_policy_names:
        live.append("wrist")
    if "camera2" in real_policy_names:
        live.append("base")

    if live:
        print(",".join(live))
    else:
        print("wrist,base")
    raise SystemExit

# Fallback.
if len(real_policy_names) <= 1:
    print("wrist")
else:
    print("wrist,base")
PY
}

# ---------------------------------------------------------------------
# Kamera-Konfiguration
# ---------------------------------------------------------------------
#
# Unterstützte Fälle:
#
# 1) Ältere/selbst trainierte SmolVLA-Checkpoints:
#
#      POLICY_CAMERA_NAMES=wrist,base,empty_camera_0,empty_camera_1
#      CAMERA_NAMES=wrist,base
#
# 2) SmolVLA-Base-Checkpoints mit rename_map:
#
#      Dataset:
#        observation.images.wrist -> observation.images.camera1
#        observation.images.base  -> observation.images.camera2
#
#      POLICY_CAMERA_NAMES kann dann z.B. sein:
#        camera1,camera2,empty_camera_0
#
#      CAMERA_NAMES bleibt:
#        wrist,base
#
# 3) Ältere Modelle mit camera1,camera2,camera3:
#
#      Du kannst manuell überschreiben:
#
#        CAMERA_NAMES=wrist,base \\
#        POLICY_CAMERA_NAMES=camera1,camera2,camera3 \\
#        ./start_lerobot_native_real_policy.sh
#
# ---------------------------------------------------------------------

INFERRED_POLICY_CAMERA_NAMES="$(infer_policy_camera_names)"

if [[ "$AUTO_CAMERA_CONFIG" == "true" && -n "$INFERRED_POLICY_CAMERA_NAMES" ]]; then
  POLICY_CAMERA_NAMES_RESOLVED="${POLICY_CAMERA_NAMES:-$INFERRED_POLICY_CAMERA_NAMES}"
  CAMERA_NAMES_RESOLVED="${CAMERA_NAMES:-${CAMERAS:-$(infer_live_camera_names "$POLICY_CAMERA_NAMES_RESOLVED")}}"
else
  POLICY_CAMERA_NAMES_RESOLVED="${POLICY_CAMERA_NAMES:-camera1,camera2,camera3}"
  CAMERA_NAMES_RESOLVED="${CAMERA_NAMES:-${CAMERAS:-wrist,base}}"
fi

LEROBOT_RECORD_CAMERA_NAMES_RESOLVED="${LEROBOT_RECORD_CAMERA_NAMES:-$CAMERA_NAMES_RESOLVED}"

# This part has to be done once to install the necessary GELLO/LeRobot plugin stuff.
# python -m pip install -e .
# python -m pip install -e lerobot_robot_gello

if ! command -v lerobot-rollout >/dev/null 2>&1; then
  echo "FEHLT: lerobot-rollout wurde nicht gefunden." >&2
  echo "Dein installiertes LeRobot erkennt zwar robot.type=gello_zmq, aber" >&2
  echo "lerobot-record in LeRobot 0.5.x akzeptiert keine --policy.path Argumente." >&2
  echo "Bitte LeRobot auf eine Version mit lerobot-rollout aktualisieren." >&2
  exit 1
fi

warm_lerobot_imports() {
  echo "Warming up LeRobot rollout imports..."
  python - <<'PY'
from lerobot.scripts.lerobot_rollout import main as _main
print("LeRobot rollout imports are ready.")
PY
}

if [[ "${WARM_LEROBOT_IMPORTS:-true}" == "true" ]]; then
  warm_lerobot_imports
fi

if [[ "$RECORD_LEROBOT" == "true" ]]; then
  ROLLOUT_STRATEGY="${STRATEGY_TYPE:-sentry}"
else
  ROLLOUT_STRATEGY="${STRATEGY_TYPE:-base}"
fi

MODEL_NAME="$(infer_model_name_from_ckpt)"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
INFERENCE_BASE_DIR="${INFERENCE_BASE_DIR:-$HOME/lerobot_inferences}"

# LeRobot-Aufnahme:
# Jeder Lauf bekommt standardmäßig einen neuen Dataset-Root.
# Wichtig: Dieser konkrete Root darf vorher noch NICHT existieren.
LEROBOT_RECORD_ROOT_RESOLVED="${LEROBOT_RECORD_ROOT:-$INFERENCE_BASE_DIR/${MODEL_NAME}_${RUN_STAMP}}"

# Repo-ID muss bei Rollout-Datasets mit rollout_ anfangen.
LEROBOT_RECORD_REPO_ID_RESOLVED="${LEROBOT_RECORD_REPO_ID:-local/rollout_${MODEL_NAME}_${RUN_STAMP}}"

LEROBOT_RECORD_TASK_RESOLVED="${LEROBOT_RECORD_TASK:-$TASK}"
LEROBOT_RECORD_FPS_RESOLVED="${LEROBOT_RECORD_FPS:-$FPS}"
LEROBOT_RECORD_PUSH_TO_HUB_RESOLVED="${LEROBOT_RECORD_PUSH_TO_HUB:-false}"

# H5-Logger:
H5_LOG_DIR_RESOLVED="${H5_LOG_DIR:-$INFERENCE_BASE_DIR/h5/${MODEL_NAME}}"
H5_LOG_BASENAME="${H5_LOG_BASENAME:-${MODEL_NAME}_${RUN_STAMP}}"
H5_DELETE_INTERMEDIATE_CSV_RESOLVED="${H5_DELETE_INTERMEDIATE_CSV:-true}"

# gello_zmq kann aktuell nur ein Joint-CSV schreiben. Dieses wird nach jeder Episode nach H5 konvertiert.
# Ohne --h5-log ist der Joint-CSV-Logger standardmäßig deaktiviert, damit der alte normale Logger entfernt ist.
if [[ "$H5_LOG" == "true" ]]; then
  JOINT_INFERENCE_LOG_DIR_RESOLVED="${JOINT_INFERENCE_LOG_DIR:-$INFERENCE_BASE_DIR/joint_csv/${MODEL_NAME}_${RUN_STAMP}}"
else
  JOINT_INFERENCE_LOG_DIR_RESOLVED="${JOINT_INFERENCE_LOG_DIR:-logs/inference_joint_logs}"
fi

JOINT_INFERENCE_LOG_ENABLED_RESOLVED="${JOINT_INFERENCE_LOG_ENABLED:-$H5_LOG}"

cat <<EOF_CONFIG
Using native LeRobot rollout CLI with robot.type=gello_zmq
CKPT=$CKPT
MODEL_NAME=$MODEL_NAME
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
H5_LOG=$H5_LOG
ROLLOUT_STRATEGY=$ROLLOUT_STRATEGY
MAX_JOINT_DELTA=${MAX_JOINT_DELTA:-0.2}
MAX_GRIPPER_DELTA=${MAX_GRIPPER_DELTA:-1.0}
ACTION_MODE=${ACTION_MODE:-absolute_joint_position}
JOINT_INFERENCE_LOG_DIR=$JOINT_INFERENCE_LOG_DIR_RESOLVED
JOINT_INFERENCE_LOG_ENABLED=$JOINT_INFERENCE_LOG_ENABLED_RESOLVED
EOF_CONFIG

if [[ "$RECORD_LEROBOT" == "true" ]]; then
  echo "LEROBOT_RECORD_ROOT=$LEROBOT_RECORD_ROOT_RESOLVED"
  echo "LEROBOT_RECORD_REPO_ID=$LEROBOT_RECORD_REPO_ID_RESOLVED"
  echo "LEROBOT_RECORD_TASK=$LEROBOT_RECORD_TASK_RESOLVED"
  echo "LEROBOT_RECORD_FPS=$LEROBOT_RECORD_FPS_RESOLVED"
  echo "LEROBOT_RECORD_PUSH_TO_HUB=$LEROBOT_RECORD_PUSH_TO_HUB_RESOLVED"
  echo "LEROBOT_RECORD_CAMERA_NAMES=$LEROBOT_RECORD_CAMERA_NAMES_RESOLVED"

  # Nicht den Dataset-Root selbst erstellen.
  # LeRobotDataset.create() erstellt diesen Ordner selbst und erwartet,
  # dass er vorher noch nicht existiert.
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
missing = []
for name in ["pandas", "h5py", "numpy"]:
    if importlib.util.find_spec(name) is None:
        missing.append(name)
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

  local h5_path="$H5_LOG_DIR_RESOLVED/${H5_LOG_BASENAME}_episode_${episode}.h5"

  python - "$latest_csv" "$h5_path" "$CKPT" "$TASK" "$FPS" <<'PY'
import sys
import re
from pathlib import Path

import h5py
import numpy as np
import pandas as pd

csv_path = Path(sys.argv[1])
h5_path = Path(sys.argv[2])
ckpt = sys.argv[3]
task = sys.argv[4]
fps = float(sys.argv[5])

df = pd.read_csv(csv_path)
h5_path.parent.mkdir(parents=True, exist_ok=True)

def safe_name(name: str) -> str:
    name = re.sub(r"[^A-Za-z0-9_]+", "_", str(name)).strip("_")
    return name or "unnamed"

def lower(col):
    return str(col).lower()

def contains_any(col, patterns):
    c = lower(col)
    return any(p in c for p in patterns)

def is_action_like(col):
    return contains_any(col, ["action", "target", "command", "sent", "policy"])

def is_time_col(col):
    return lower(col) in {
        "time", "t", "timestamp", "timestamp_s",
        "wall_time", "wall_time_s",
        "monotonic_time", "monotonic_time_s",
        "elapsed", "elapsed_s", "time_s"
    }

def numeric_cols(cols):
    out = []
    for col in cols:
        try:
            pd.to_numeric(df[col])
            out.append(col)
        except Exception:
            pass
    return out

time_cols = [c for c in df.columns if is_time_col(c)]

q_cols = [
    c for c in df.columns
    if not is_action_like(c)
    and contains_any(c, [
        "joint_position", "joint_pos", "position_rad",
        "actual_joint", "current_joint", "q_",
        "q.", "q[", "q"
    ])
]

dq_cols = [
    c for c in df.columns
    if not is_action_like(c)
    and contains_any(c, [
        "joint_velocity", "joint_vel", "velocity_rad",
        "vel_rad", "dq_", "dq.", "dq[", "dq"
    ])
]

tau_cols = [
    c for c in df.columns
    if not is_action_like(c)
    and contains_any(c, [
        "joint_torque", "torque", "tau_", "tau.",
        "tau[", "effort"
    ])
]

action_cols = [
    c for c in df.columns
    if is_action_like(c)
]

q_cols = numeric_cols(q_cols)
dq_cols = numeric_cols(dq_cols)
tau_cols = numeric_cols(tau_cols)
action_cols = numeric_cols(action_cols)

with h5py.File(h5_path, "w") as f:
    meta = f.create_group("meta")
    meta.attrs["source_csv"] = str(csv_path)
    meta.attrs["checkpoint"] = ckpt
    meta.attrs["task"] = task
    meta.attrs["fps"] = fps
    meta.attrs["note"] = (
        "All original CSV columns are stored under /raw_csv. "
        "Recognized joint position, velocity, torque and action columns "
        "are additionally grouped under /state and /action when detected. "
        "If velocity columns are identical to positions, they are rejected "
        "and /state/dq_estimated is computed from /state/q."
    )

    raw = f.create_group("raw_csv")
    for col in df.columns:
        values = df[col].to_numpy()
        dset_name = safe_name(col)

        if values.dtype.kind in {"O", "U"}:
            values = np.array([str(v).encode("utf-8") for v in values])
            raw.create_dataset(dset_name, data=values)
        else:
            raw.create_dataset(dset_name, data=values)

        raw[dset_name].attrs["original_column"] = str(col)

    if time_cols:
        t = pd.to_numeric(df[time_cols[0]], errors="coerce").to_numpy(dtype=np.float64)
        f.create_dataset("time", data=t)
        f["time"].attrs["source_column"] = str(time_cols[0])
        f["time"].attrs["unit"] = "s"
    else:
        t = np.arange(len(df), dtype=np.float64) / fps
        f.create_dataset("time", data=t)
        f["time"].attrs["source_column"] = "estimated_from_fps"
        f["time"].attrs["unit"] = "s"

    state = f.create_group("state")
    action = f.create_group("action")

    def write_matrix(group, name, cols, unit):
        if not cols:
            return None
        data = df[cols].apply(pd.to_numeric, errors="coerce").to_numpy(dtype=np.float64)
        dset = group.create_dataset(name, data=data)
        dset.attrs["columns"] = np.array([str(c).encode("utf-8") for c in cols])
        dset.attrs["unit"] = unit
        return data

    q_data = write_matrix(state, "q", q_cols, "rad")

    dq_raw_data = None
    dq_is_valid = False

    if dq_cols:
        dq_raw_data = df[dq_cols].apply(pd.to_numeric, errors="coerce").to_numpy(dtype=np.float64)
        dq_is_valid = True

    # Neue Erkenntnis:
    # In deinem aktuellen CSV sind joint_*_velocity_rad_s identisch zu joint_*_position_rad.
    # Solche Velocity-Spalten dürfen nicht als echte state/dq gespeichert werden.
    if q_data is not None and dq_raw_data is not None and q_data.shape == dq_raw_data.shape:
        if np.allclose(q_data, dq_raw_data, equal_nan=True):
            dq_is_valid = False
            state.attrs["dq_warning"] = (
                "Velocity columns were identical to position columns. "
                "They were not stored as state/dq. Use state/dq_estimated instead."
            )

            rejected = state.create_dataset("dq_rejected_identical_to_q", data=dq_raw_data)
            rejected.attrs["columns"] = np.array([str(c).encode("utf-8") for c in dq_cols])
            rejected.attrs["unit_claimed_by_csv"] = "rad/s"
            rejected.attrs["reason"] = "identical_to_state_q"
            rejected.attrs["do_not_use_as_velocity"] = True

    if dq_is_valid and dq_raw_data is not None:
        dset = state.create_dataset("dq", data=dq_raw_data)
        dset.attrs["columns"] = np.array([str(c).encode("utf-8") for c in dq_cols])
        dset.attrs["unit"] = "rad/s"
        dset.attrs["source"] = "csv_velocity_columns"

    write_matrix(state, "tau", tau_cols, "Nm")
    write_matrix(action, "values", action_cols, "policy_units_or_robot_units")

    # dq_estimated wird immer erzeugt, sobald q vorhanden ist.
    if q_data is not None and len(q_data) >= 2:
        t = f["time"][:]

        # Falls time absolute Unix-Zeit ist, ist das okay: np.gradient nutzt nur die Abstände.
        try:
            dq_est = np.gradient(q_data, t, axis=0)
            dq_source = "numerical_gradient_of_state_q_using_time"
        except Exception:
            dq_est = np.gradient(q_data, 1.0 / fps, axis=0)
            dq_source = "numerical_gradient_of_state_q_using_fps"

        dset = state.create_dataset("dq_estimated", data=dq_est)
        dset.attrs["unit"] = "rad/s"
        dset.attrs["source"] = dq_source
        dset.attrs["note"] = (
            "Estimated from state/q. Prefer real state/dq only if valid velocity "
            "columns are available and not identical to q."
        )

print(f"H5 geschrieben: {h5_path}")
PY

  if [[ "$H5_DELETE_INTERMEDIATE_CSV_RESOLVED" == "true" ]]; then
    rm -f "$latest_csv"
    echo "Temporäres Joint-CSV gelöscht: $latest_csv"
  fi
}

run_rollout_episode() {
  local episode="$1"

  echo "Starting rollout episode $episode/$NUM_EPISODES"

  local cmd=(
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
    --robot.max_joint_delta="${MAX_JOINT_DELTA:-0.2}"
    --robot.max_gripper_delta="${MAX_GRIPPER_DELTA:-1.0}"
    --robot.action_mode="${ACTION_MODE:-absolute_joint_position}"
    --robot.joint_inference_log_dir="$JOINT_INFERENCE_LOG_DIR_RESOLVED"
    --robot.joint_inference_log_enabled="$JOINT_INFERENCE_LOG_ENABLED_RESOLVED"
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

  local exit_code=0
  "${cmd[@]}" || exit_code=$?

  convert_latest_joint_csv_to_h5 "$episode"

  return "$exit_code"
}

for episode in $(seq 1 "$NUM_EPISODES"); do
  run_rollout_episode "$episode"
done