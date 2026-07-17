#!/usr/bin/env bash
set -euo pipefail

# Pi0.5 episodic LeRobot rollout for the GELLO/ZMQ Panda stack.
#
# The Pi0.5 policy is loaded only once. Multiple autonomous episodes are
# executed in the same process. Between episodes, LeRobot returns the robot
# to the joint position captured when this process connected to the robot.
#
# Default model source:
#   Hugging Face repo: TimR643/pick_rectangle_go_up_pi05
#   Checkpoint folder: checkpoints/002000/pretrained_model
#
# The first run downloads ONLY the selected pretrained_model folder into the
# Hugging Face cache. Later runs reuse the cache. The whole repository and the
# training_state folder are not downloaded.

source "${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
conda activate "${LEROBOT_ENV:-$HOME/miniconda3/envs/lerobot}"

REPO_DIR="${GELLO_REPO_DIR:-$HOME/gello_software}"
cd "$REPO_DIR"

# Hugging Face model source. The current repo contains 002000 and 004000.
: "${HF_MODEL_REPO:=TimR643/pick_rectangle_go_up_pi05}"
: "${HF_REVISION:=main}"
: "${HF_CHECKPOINT:=010000}"

# Optional local override. Leave empty to load the selected checkpoint from HF.
# Examples:
#   CKPT=/path/to/pretrained_model ./this_script.sh
#   HF_CHECKPOINT=004000 ./this_script.sh
: "${CKPT:=}"

: "${TASK:=pick up the red rectangle and go above the necessary height}"
: "${DURATION:=0}"
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
: "${RTC_EXECUTION_HORIZON:=12}"
: "${RTC_MAX_GUIDANCE_WEIGHT:=5.0}"
: "${RTC_PREFIX_ATTENTION_SCHEDULE:=}"
: "${LOG_ACTION_DIAGNOSTICS_EVERY_N:=25}"
: "${JOINT_INFERENCE_LOG_ENABLED:=false}"
: "${JOINT_INFERENCE_LOG_DIR:=logs/pi05_inference_joint_logs}"
: "${NUM_EPISODES:=10}"
: "${EPISODE_TIME_S:=100}"
: "${RESET_TIME_S:=15}"
: "${RESET_TO_INITIAL_POSITION:=true}"
: "${EPISODIC_BASE_DIR:=$HOME/lerobot_inferences/pi05_episodic}"
: "${EPISODIC_PUSH_TO_HUB:=false}"
: "${EPISODIC_STREAMING_ENCODING:=true}"
: "${EPISODIC_ENCODER_THREADS:=1}"
: "${INCLUDE_EMPTY_CAMERAS_IN_ROBOT_OBS:=false}"
: "${INCLUDE_UNMAPPED_POLICY_CAMERAS_AS_BLACK:=false}"

# Stable remote-camera profile, aligned with the SmolVLA launcher that does
# not lose the wrist-camera connection:
# - 10 Hz keeps the camera ZMQ server polled regularly.
# - 3000 ms tolerates short SSH/network/preprocessing stalls.
# - RTC horizon/guidance match the proven SmolVLA timing profile and provide
#   a longer overlap for Pi0.5 chunk transitions.
# - Eight CPU threads accelerate the expensive Pi0.5/PaliGemma construction.
# - Passive OpenMP waiting keeps those threads from busy-spinning during rollout,
#   preserving CPU time for the camera/ZMQ control loop.

# Keep Pi0.5 from exhausting CPU RAM/threads on the robot/server host.
# All values are still overridable by exporting them before launching.
: "${PI05_LIMIT_CPU_THREADS:=true}"
: "${PI05_CPU_THREADS:=8}"
: "${TOKENIZERS_PARALLELISM:=false}"
: "${PYTORCH_CUDA_ALLOC_CONF:=expandable_segments:True}"
: "${CUDA_MODULE_LOADING:=LAZY}"
: "${OMP_WAIT_POLICY:=PASSIVE}"
: "${KMP_BLOCKTIME:=0}"
: "${PI05_OFFLINE_LOAD:=true}"

if [[ "$PI05_LIMIT_CPU_THREADS" == "true" ]]; then
  export OMP_NUM_THREADS="${OMP_NUM_THREADS:-$PI05_CPU_THREADS}"
  export MKL_NUM_THREADS="${MKL_NUM_THREADS:-$PI05_CPU_THREADS}"
  export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-$PI05_CPU_THREADS}"
  export NUMEXPR_NUM_THREADS="${NUMEXPR_NUM_THREADS:-$PI05_CPU_THREADS}"
  export VECLIB_MAXIMUM_THREADS="${VECLIB_MAXIMUM_THREADS:-$PI05_CPU_THREADS}"
fi

export TOKENIZERS_PARALLELISM
export PYTORCH_CUDA_ALLOC_CONF
export CUDA_MODULE_LOADING
export OMP_WAIT_POLICY
export KMP_BLOCKTIME
export PYTHONUNBUFFERED=1

usage() {
  cat <<EOF_USAGE
Usage:
  $0 [--sync] [--rtc] [--help]

The policy is loaded once and reused for every episode.

Episode overrides:
  NUM_EPISODES              Default: 10
  EPISODE_TIME_S             Default: 100 seconds
  RESET_TIME_S               Default: 15 seconds
  RESET_TO_INITIAL_POSITION  Default: true
  EPISODIC_BASE_DIR          Default: \$HOME/lerobot_inferences/pi05_episodic

During the session:
  Right arrow  End the current episode or reset phase early
  Left arrow   Discard and repeat the current episode
  Escape       End the complete session

Important:
  Move the robot to the desired initial pose before starting this script.
  LeRobot captures that pose when it connects and returns to it between episodes.

Model source:
  HF_MODEL_REPO       Default: TimR643/pick_rectangle_go_up_pi05
  HF_REVISION         Default: main
  HF_CHECKPOINT       Default: 010000
  CKPT                Optional local pretrained_model path
  LOCAL_MODEL_DIR     Optional local training/output root

Rollout overrides:
  TASK                 Task prompt
  FPS                  Default: 10
  DEVICE               Default: cuda

Camera/ZMQ:
  ZMQ_TIMEOUT_MS       Default: 3000
  LIVE_CAMERA_NAMES    Usually wrist,base
  POLICY_CAMERA_NAMES  Policy-facing camera names

Inference:
  INFERENCE_TYPE             rtc or sync; default: rtc
  RTC_EXECUTION_HORIZON      Default: 12
  RTC_MAX_GUIDANCE_WEIGHT    Default: 5.0

Performance:
  PI05_CPU_THREADS      Default: 8
  PI05_OFFLINE_LOAD     Default: true
EOF_USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
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

resolve_local_policy_path() {
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

download_hf_checkpoint() {
  python - "$HF_MODEL_REPO" "$HF_REVISION" "$HF_CHECKPOINT" <<'PY'
import sys
from pathlib import Path

try:
    from huggingface_hub import snapshot_download
except ImportError as exc:
    raise SystemExit(
        "huggingface_hub fehlt. Installiere es im aktiven Environment mit: "
        "pip install -U huggingface_hub"
    ) from exc

repo_id, revision, checkpoint = sys.argv[1:4]
subdir = f"checkpoints/{checkpoint}/pretrained_model"

try:
    # Repeated HPC starts should not wait for Hub HEAD/metadata requests.
    # Resolve the already cached snapshot locally first and use the network only
    # when the requested checkpoint is genuinely absent.
    download_kwargs = dict(
        repo_id=repo_id,
        repo_type="model",
        revision=revision or None,
        allow_patterns=[f"{subdir}/*", f"{subdir}/**"],
    )
    try:
        snapshot_root = Path(snapshot_download(local_files_only=True, **download_kwargs))
        print("Verwende vollständig lokalen Hugging-Face-Cache.", file=sys.stderr)
    except Exception:
        snapshot_root = Path(snapshot_download(local_files_only=False, **download_kwargs))
except Exception as exc:
    raise SystemExit(
        f"Konnte {repo_id}@{revision}:{subdir} "
        f"nicht aus dem Hugging-Face-Hub laden: {exc}"
    ) from exc

policy_dir = snapshot_root / subdir
config_path = policy_dir / "config.json"

if not config_path.is_file():
    raise SystemExit(
        f"Checkpoint nicht gefunden: "
        f"{repo_id}@{revision}/{subdir}/config.json"
    )

print(policy_dir)
PY
}


prepare_compatible_policy_dir() {
  python - "$1" <<'PY_COMPAT'
import hashlib
import json
import os
import shutil
import sys
from pathlib import Path

policy_dir = Path(sys.argv[1]).expanduser().resolve()
config_path = policy_dir / "config.json"

if not config_path.is_file():
    raise SystemExit(f"FEHLT: {config_path}")

raw_config = config_path.read_bytes()
config = json.loads(raw_config)

# Ältere LeRobot-PI05Config-Versionen kennen dieses neue
# Metadatenfeld nicht. Beim lokalen Laden wird es nicht benötigt.
if "pretrained_revision" not in config:
    print(policy_dir)
    raise SystemExit(0)

removed_value = config.pop("pretrained_revision")

default_cache_root = policy_dir.parent / ".pi05_compat"
cache_root = Path(
    os.environ.get(
        "PI05_COMPAT_CACHE_DIR",
        str(default_cache_root),
    )
).expanduser()

digest = hashlib.sha256(
    str(policy_dir).encode("utf-8")
    + b"\0"
    + raw_config
).hexdigest()[:16]

compat_dir = cache_root / digest
compat_config = compat_dir / "config.json"

sanitized = json.dumps(
    config,
    indent=2,
    ensure_ascii=False,
) + "\n"

rebuild = True

if compat_config.is_file():
    try:
        rebuild = compat_config.read_text() != sanitized
    except OSError:
        rebuild = True

if rebuild:
    if compat_dir.exists() or compat_dir.is_symlink():
        if compat_dir.is_dir() and not compat_dir.is_symlink():
            shutil.rmtree(compat_dir)
        else:
            compat_dir.unlink()

    compat_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # Alle Modell-Dateien werden nur verlinkt.
    # Die großen Gewichte werden nicht kopiert.
    for source in policy_dir.iterdir():
        if source.name == "config.json":
            continue

        destination = compat_dir / source.name

        destination.symlink_to(
            source.resolve(),
            target_is_directory=source.is_dir(),
        )

    compat_config.write_text(sanitized)

print(
    "Kompatibilitätsfix aktiv: entferne "
    f"pretrained_revision={removed_value!r} "
    "nur aus einer separaten Config.",
    file=sys.stderr,
)

print(
    f"Original bleibt unverändert: {config_path}",
    file=sys.stderr,
)

print(
    f"Verwendeter Policy-Pfad: {compat_dir}",
    file=sys.stderr,
)

print(compat_dir)
PY_COMPAT
}

# Wenn CKPT nicht explizit gesetzt wurde, wird zuerst das lokal
# heruntergeladene Modell verwendet.
if [[ -z "$CKPT" ]]; then
  LOCAL_MODEL_DIR_RESOLVED="${LOCAL_MODEL_DIR:-$HOME/lerobot_outputs/train/${HF_MODEL_REPO##*/}}"
  LOCAL_CKPT_CANDIDATE="$LOCAL_MODEL_DIR_RESOLVED/checkpoints/$HF_CHECKPOINT/pretrained_model"

  if [[ -f "$LOCAL_CKPT_CANDIDATE/config.json" ]]; then
    CKPT="$LOCAL_CKPT_CANDIDATE"
    echo "Verwende lokalen Checkpoint: $CKPT" >&2
  fi
fi

if [[ -n "$CKPT" ]]; then
  ORIGINAL_CKPT="$CKPT"

  if ! CKPT="$(resolve_local_policy_path "$CKPT")"; then
    echo \
      "FEHLT: CKPT muss auf ein pretrained_model-Verzeichnis mit config.json zeigen: $ORIGINAL_CKPT" \
      >&2
    exit 1
  fi

  MODEL_SOURCE="local"
else
  if [[ ! "$HF_CHECKPOINT" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo \
      "UNGÜLTIG: HF_CHECKPOINT enthält unerlaubte Zeichen: $HF_CHECKPOINT" \
      >&2
    exit 2
  fi

  echo \
    "Lade/verifiziere HF-Checkpoint: ${HF_MODEL_REPO}@${HF_REVISION}/checkpoints/${HF_CHECKPOINT}/pretrained_model" \
    >&2
  echo \
    "Beim ersten Start werden nur die benötigten Modelldateien in den HF-Cache geladen." \
    >&2

  CKPT="$(download_hf_checkpoint)"
  MODEL_SOURCE="huggingface-cache"
fi

POLICY_CONFIG_PATH="$CKPT/config.json"

if [[ ! -f "$POLICY_CONFIG_PATH" ]]; then
  echo \
    "FEHLT: config.json wurde nicht gefunden: $POLICY_CONFIG_PATH" \
    >&2
  exit 1
fi

ORIGINAL_CKPT="$CKPT"

CKPT="$(prepare_compatible_policy_dir "$CKPT")"

POLICY_CONFIG_PATH="$CKPT/config.json"

if [[ "$PI05_OFFLINE_LOAD" == "true" ]]; then
  export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
  export TRANSFORMERS_OFFLINE="${TRANSFORMERS_OFFLINE:-1}"
  export HF_HUB_DISABLE_TELEMETRY="${HF_HUB_DISABLE_TELEMETRY:-1}"
fi

if [[ "$CKPT" != "$ORIGINAL_CKPT" ]]; then
  MODEL_SOURCE="${MODEL_SOURCE}-pi05-config-compat"
fi

export CKPT

infer_policy_image_names() {
  python - "$POLICY_CONFIG_PATH" <<'PY'
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
            if (
                feat.get("type") in ("VISUAL", "FeatureType.VISUAL")
                or typ.endswith("VISUAL")
            ):
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
  python \
    - "$1" \
    "$INCLUDE_EMPTY_CAMERAS_IN_ROBOT_OBS" \
    "$INCLUDE_UNMAPPED_POLICY_CAMERAS_AS_BLACK" <<'PY'
import sys

names = [
    name.strip()
    for name in sys.argv[1].split(",")
    if name.strip()
]

include_empty = sys.argv[2].lower() == "true"
include_unmapped = sys.argv[3].lower() == "true"

# GELLO currently has live wrist/base cameras. Legacy rename conventions are
# also accepted for checkpoints that call these camera1/camera2.
live_mappable = {
    "camera1",
    "camera2",
    "wrist",
    "base",
}

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
    print(
        "OMITTED_EMPTY=" + ",".join(omitted_empty),
        file=sys.stderr,
    )

if omitted_unmapped:
    print(
        "OMITTED_UNMAPPED=" + ",".join(omitted_unmapped),
        file=sys.stderr,
    )
PY
}

infer_live_camera_names_from_policy_names() {
  python - "$1" <<'PY'
import sys

names = [
    name.strip()
    for name in sys.argv[1].split(",")
    if name.strip()
]

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

ROBOT_POLICY_CAMERA_NAMES_DEFAULT="$(
  filter_policy_camera_names_for_robot \
    "$INFERRED_POLICY_IMAGE_NAMES"
)"

POLICY_CAMERA_NAMES_RESOLVED="${POLICY_CAMERA_NAMES:-$ROBOT_POLICY_CAMERA_NAMES_DEFAULT}"

LIVE_CAMERA_NAMES_RESOLVED="${LIVE_CAMERA_NAMES:-${CAMERA_NAMES:-$(infer_live_camera_names_from_policy_names "$POLICY_CAMERA_NAMES_RESOLVED")}}"

if [[ -z "$POLICY_CAMERA_NAMES_RESOLVED" ]]; then
  echo \
    "FEHLT: Konnte keine Policy-Kameras aus $POLICY_CONFIG_PATH ableiten." \
    >&2
  exit 1
fi

if (( ZMQ_TIMEOUT_MS < 3000 )); then
  echo "WARNUNG: ZMQ_TIMEOUT_MS=$ZMQ_TIMEOUT_MS ist fuer Pi0.5 ueber SSH knapp; empfohlen sind mindestens 3000 ms." >&2
fi

python - "$FPS" "$RTC_EXECUTION_HORIZON" <<'PY_TIMING_CHECK'
import sys
fps = float(sys.argv[1])
horizon = int(sys.argv[2])
if fps <= 0:
    raise SystemExit("FEHLT: FPS muss groesser als 0 sein.")
if horizon <= 0:
    raise SystemExit("FEHLT: RTC_EXECUTION_HORIZON muss groesser als 0 sein.")
if fps < 10:
    print(
        f"WARNUNG: FPS={fps:g}. Fuer den stabilen Remote-Kamerabetrieb wurde FPS=10 getestet.",
        file=sys.stderr,
    )
if horizon < 10:
    print(
        f"WARNUNG: RTC_EXECUTION_HORIZON={horizon}. Fuer Pi0.5 sind 10-12 robuster.",
        file=sys.stderr,
    )
PY_TIMING_CHECK

if [[ "$DEVICE" == cuda* ]]; then
  python - <<'PY_CUDA_CHECK'
try:
    import torch
except Exception as exc:
    raise SystemExit(f"FEHLT: DEVICE=cuda, aber torch kann nicht importiert werden: {exc}")

if not torch.cuda.is_available():
    raise SystemExit("FEHLT: DEVICE=cuda, aber torch.cuda.is_available() ist false. Setze DEVICE=cpu oder repariere CUDA.")
PY_CUDA_CHECK
fi

ROLLOUT_STRATEGY="${STRATEGY_TYPE:-episodic}"

if [[ "$ROLLOUT_STRATEGY" != "episodic" ]]; then
  echo "WARNUNG: Dieses Skript ist fuer strategy.type=episodic ausgelegt; aktuell: $ROLLOUT_STRATEGY" >&2
fi

if [[ "$TASK" == *"bock"* || "$TASK" == *"hight"* ]]; then
  echo \
    "WARNUNG: TASK enthält vermutlich einen Tippfehler. Aktueller TASK: $TASK" \
    >&2
fi

cat <<EOF_CONFIG
Using Pi0.5-focused native LeRobot rollout CLI with robot.type=gello_zmq
MODEL_SOURCE=$MODEL_SOURCE
HF_MODEL_REPO=$HF_MODEL_REPO
HF_REVISION=$HF_REVISION
HF_CHECKPOINT=$HF_CHECKPOINT
CKPT=$CKPT
TASK=$TASK
DURATION=$DURATION
FPS=$FPS
DEVICE=$DEVICE
ROLLOUT_STRATEGY=$ROLLOUT_STRATEGY
NUM_EPISODES=$NUM_EPISODES
EPISODE_TIME_S=$EPISODE_TIME_S
RESET_TIME_S=$RESET_TIME_S
RESET_TO_INITIAL_POSITION=$RESET_TO_INITIAL_POSITION
EPISODIC_BASE_DIR=$EPISODIC_BASE_DIR
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
LOG_ACTION_DIAGNOSTICS_EVERY_N=$LOG_ACTION_DIAGNOSTICS_EVERY_N
PI05_LIMIT_CPU_THREADS=$PI05_LIMIT_CPU_THREADS
PI05_CPU_THREADS=$PI05_CPU_THREADS
CUDA_MODULE_LOADING=$CUDA_MODULE_LOADING
OMP_WAIT_POLICY=$OMP_WAIT_POLICY
KMP_BLOCKTIME=$KMP_BLOCKTIME
OMP_NUM_THREADS=${OMP_NUM_THREADS:-<unset>}
MKL_NUM_THREADS=${MKL_NUM_THREADS:-<unset>}
OPENBLAS_NUM_THREADS=${OPENBLAS_NUM_THREADS:-<unset>}
PYTORCH_CUDA_ALLOC_CONF=$PYTORCH_CUDA_ALLOC_CONF
PI05_OFFLINE_LOAD=$PI05_OFFLINE_LOAD
HF_HUB_OFFLINE=${HF_HUB_OFFLINE:-<unset>}
TRANSFORMERS_OFFLINE=${TRANSFORMERS_OFFLINE:-<unset>}
ORIGINAL_CKPT=$ORIGINAL_CKPT
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
    cmd+=(
      --inference.rtc.prefix_attention_schedule="$RTC_PREFIX_ATTENTION_SCHEDULE"
    )
  fi
elif [[ "$INFERENCE_TYPE" == "sync" ]]; then
  cmd+=(--inference.type=sync)
else
  echo \
    "Unsupported INFERENCE_TYPE=$INFERENCE_TYPE; expected rtc or sync" \
    >&2
  exit 2
fi

# The current LeRobot episodic strategy requires a dataset configuration.
# Data is stored locally and is never uploaded unless explicitly overridden.
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
EPISODIC_ROOT="${EPISODIC_ROOT:-$EPISODIC_BASE_DIR/run_$RUN_STAMP}"
EPISODIC_REPO_ID="${EPISODIC_REPO_ID:-local/rollout_pi05_episodic_$RUN_STAMP}"

mkdir -p "$EPISODIC_ROOT"

cmd+=(
  --strategy.reset_to_initial_position="$RESET_TO_INITIAL_POSITION"
  --dataset.root="$EPISODIC_ROOT"
  --dataset.repo_id="$EPISODIC_REPO_ID"
  --dataset.single_task="$TASK"
  --dataset.fps="$FPS"
  --dataset.episode_time_s="$EPISODE_TIME_S"
  --dataset.reset_time_s="$RESET_TIME_S"
  --dataset.num_episodes="$NUM_EPISODES"
  --dataset.push_to_hub="$EPISODIC_PUSH_TO_HUB"
  --dataset.streaming_encoding="$EPISODIC_STREAMING_ENCODING"
  --dataset.encoder_threads="$EPISODIC_ENCODER_THREADS"
)

cat <<EOF_EPISODIC

Episodic session:
  Episodes:            $NUM_EPISODES
  Episode duration:    $EPISODE_TIME_S s
  Reset duration:      $RESET_TIME_S s
  Return to start:     $RESET_TO_INITIAL_POSITION
  Local dataset root:  $EPISODIC_ROOT

Keyboard:
  Right arrow = finish current episode/reset early
  Left arrow  = discard and repeat episode
  Escape      = stop session

EOF_EPISODIC

printf 'Executing:'
printf ' %q' "${cmd[@]}"
printf '\n'

"${cmd[@]}"