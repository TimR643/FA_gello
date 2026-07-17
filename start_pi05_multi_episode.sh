#!/usr/bin/env bash
set -euo pipefail

# Pi0.5 multi-episode rollout for the GELLO/ZMQ Panda stack.
#
# Compatible with LeRobot installations that only provide the built-in
# base/sentry/highlight/dagger strategies. This launcher installs a small
# local multi_episode strategy into the active LeRobot environment.
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
: "${EPISODE_TIME_S:=30}"
: "${RESET_TIME_S:=15}"
: "${RESET_MOVE_DURATION_S:=2}"
: "${RESET_TO_INITIAL_POSITION:=true}"
: "${AUTO_INSTALL_MULTI_EPISODE:=true}"
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

This script loads Pi0.5 once and executes several autonomous passes.

Episode settings:
  NUM_EPISODES              Default: 10
  EPISODE_TIME_S             Default: 100
  RESET_TIME_S               Default: 15
  RESET_MOVE_DURATION_S      Default: 2
  RESET_TO_INITIAL_POSITION  Default: true

Workflow:
  1. Move the robot to the desired starting pose before launching.
  2. Start this script. LeRobot captures that pose at connection time.
  3. After each episode, policy and RTC state are cleared.
  4. The robot returns to the captured starting pose.
  5. During RESET_TIME_S, place the object back.

The script adds a local strategy named multi_episode to the active LeRobot
installation when it is missing. Original files are backed up once with a
.pre_multi_episode suffix.

Model:
  HF_MODEL_REPO       Default: TimR643/pick_rectangle_go_up_pi05
  HF_CHECKPOINT       Default: 010000
  CKPT                Optional local pretrained_model path

Stable camera settings:
  FPS                   Default: 10
  ZMQ_TIMEOUT_MS        Default: 3000
  RTC_EXECUTION_HORIZON Default: 12
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

install_multi_episode_strategy() {
  python - <<'PY_MULTI_EPISODE_INSTALL'
from __future__ import annotations

import inspect
import py_compile
import shutil
from pathlib import Path

import lerobot.rollout.configs as rollout_configs

rollout_dir = Path(inspect.getfile(rollout_configs)).resolve().parent
configs_path = rollout_dir / "configs.py"
strategies_dir = rollout_dir / "strategies"
factory_path = strategies_dir / "factory.py"
init_path = strategies_dir / "__init__.py"
strategy_path = strategies_dir / "multi_episode.py"

required = [configs_path, factory_path, init_path, strategies_dir / "core.py"]
missing = [str(p) for p in required if not p.exists()]
if missing:
    raise SystemExit("LeRobot rollout files fehlen: " + ", ".join(missing))

strategy_source = r'''# Copyright 2025 The HuggingFace Inc. team.
"Autonomous multi-episode rollout without dataset recording."

from __future__ import annotations

import logging
import time

from lerobot.utils.robot_utils import precise_sleep

from ..context import RolloutContext
from .core import RolloutStrategy, send_next_action

logger = logging.getLogger(__name__)


class MultiEpisodeStrategy(RolloutStrategy):
    "Keep one policy loaded while executing and resetting multiple episodes."

    def setup(self, ctx: RolloutContext) -> None:
        self._init_engine(ctx)
        logger.info("Multi-episode strategy ready")

    def run(self, ctx: RolloutContext) -> None:
        cfg = ctx.runtime.cfg
        strategy_cfg = self.config
        robot = ctx.hardware.robot_wrapper
        engine = self._engine
        interpolator = self._interpolator

        control_interval = interpolator.get_control_interval(cfg.fps)
        num_episodes = int(strategy_cfg.num_episodes)
        episode_time_s = float(strategy_cfg.episode_time_s)
        reset_time_s = float(strategy_cfg.reset_time_s)

        for episode_index in range(num_episodes):
            if ctx.runtime.shutdown_event.is_set():
                break

            logger.info(
                "Starting episode %d/%d (%.1f s)",
                episode_index + 1,
                num_episodes,
                episode_time_s,
            )

            engine.reset()
            interpolator.reset()
            engine.resume()
            episode_start = time.perf_counter()

            while (
                time.perf_counter() - episode_start < episode_time_s
                and not ctx.runtime.shutdown_event.is_set()
            ):
                loop_start = time.perf_counter()
                obs = robot.get_observation()
                obs_processed = self._process_observation_and_notify(ctx.processors, obs)

                if self._handle_warmup(cfg.use_torch_compile, loop_start, control_interval):
                    continue

                action_dict = send_next_action(obs_processed, obs, ctx, interpolator)
                self._log_telemetry(obs_processed, action_dict, ctx.runtime)

                dt = time.perf_counter() - loop_start
                sleep_t = control_interval - dt
                if sleep_t > 0:
                    precise_sleep(sleep_t)
                else:
                    logger.warning(
                        "Control loop slower than target: %.1f Hz instead of %.1f Hz",
                        1.0 / max(dt, 1e-9),
                        cfg.fps,
                    )

            engine.pause()
            engine.reset()
            interpolator.reset()
            logger.info("Episode %d/%d finished", episode_index + 1, num_episodes)

            if episode_index >= num_episodes - 1 or ctx.runtime.shutdown_event.is_set():
                break

            if strategy_cfg.reset_to_initial_position:
                logger.info(
                    "Returning robot to captured initial position over %.1f s",
                    strategy_cfg.reset_move_duration_s,
                )
                self._return_to_initial_position(
                    hw=ctx.hardware,
                    duration_s=float(strategy_cfg.reset_move_duration_s),
                )

            logger.info("Reset phase %.1f s: place the object back now", reset_time_s)
            reset_start = time.perf_counter()

            # Poll robot and cameras throughout reset so remote ZMQ stays active.
            while (
                time.perf_counter() - reset_start < reset_time_s
                and not ctx.runtime.shutdown_event.is_set()
            ):
                loop_start = time.perf_counter()
                robot.get_observation()
                dt = time.perf_counter() - loop_start
                precise_sleep(max(control_interval - dt, 0.0))

        logger.info("All requested episodes finished")

    def teardown(self, ctx: RolloutContext) -> None:
        self._teardown_hardware(
            ctx.hardware,
            return_to_initial_position=ctx.runtime.cfg.return_to_initial_position,
        )
        logger.info("Multi-episode strategy teardown complete")
'''

config_block = '''
@RolloutStrategyConfig.register_subclass("multi_episode")
@dataclass
class MultiEpisodeStrategyConfig(RolloutStrategyConfig):
    "Run multiple autonomous episodes while keeping the policy loaded."

    num_episodes: int = 10
    episode_time_s: float = 100.0
    reset_time_s: float = 15.0
    reset_move_duration_s: float = 2.0
    reset_to_initial_position: bool = True


'''

def backup_once(path: Path) -> None:
    backup = path.with_name(path.name + ".pre_multi_episode")
    if not backup.exists():
        shutil.copy2(path, backup)

def replace_file(path: Path, content: str) -> None:
    backup_once(path)
    path.write_text(content)

strategies_dir.mkdir(parents=True, exist_ok=True)
strategy_path.write_text(strategy_source)

configs_text = configs_path.read_text()
if 'register_subclass("multi_episode")' not in configs_text:
    marker = '@RolloutStrategyConfig.register_subclass("dagger")'
    if marker not in configs_text:
        raise SystemExit(f"Konnte Einfuegepunkt in {configs_path} nicht finden.")
    configs_text = configs_text.replace(marker, config_block + marker, 1)
    replace_file(configs_path, configs_text)

factory_text = factory_path.read_text()
if "from .multi_episode import MultiEpisodeStrategy" not in factory_text:
    import_marker = "from .highlight import HighlightStrategy"
    if import_marker not in factory_text:
        import_marker = "from .sentry import SentryStrategy"
    if import_marker not in factory_text:
        raise SystemExit(f"Konnte Import-Einfuegepunkt in {factory_path} nicht finden.")
    factory_text = factory_text.replace(
        import_marker,
        "from .multi_episode import MultiEpisodeStrategy\n" + import_marker,
        1,
    )

if 'config.type == "multi_episode"' not in factory_text:
    dispatch_marker = "    raise ValueError("
    if dispatch_marker not in factory_text:
        raise SystemExit(f"Konnte Factory-Einfuegepunkt in {factory_path} nicht finden.")
    factory_text = factory_text.replace(
        dispatch_marker,
        '    if config.type == "multi_episode":\n'
        '        return MultiEpisodeStrategy(config)\n'
        + dispatch_marker,
        1,
    )
replace_file(factory_path, factory_text)

init_text = init_path.read_text()
if "from .multi_episode import MultiEpisodeStrategy" not in init_text:
    import_marker = "from .highlight import HighlightStrategy"
    if import_marker not in init_text:
        import_marker = "from .sentry import SentryStrategy"
    if import_marker in init_text:
        init_text = init_text.replace(
            import_marker,
            "from .multi_episode import MultiEpisodeStrategy\n" + import_marker,
            1,
        )

if '"MultiEpisodeStrategy",' not in init_text and "__all__" in init_text:
    list_marker = '    "HighlightStrategy",'
    if list_marker in init_text:
        init_text = init_text.replace(
            list_marker,
            list_marker + '\n    "MultiEpisodeStrategy",',
            1,
        )
replace_file(init_path, init_text)

for path in (configs_path, factory_path, init_path, strategy_path):
    py_compile.compile(str(path), doraise=True)

print(f"Multi-episode strategy installed in: {rollout_dir}")
PY_MULTI_EPISODE_INSTALL
}

if [[ "$AUTO_INSTALL_MULTI_EPISODE" == "true" ]]; then
  install_multi_episode_strategy
fi

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

ROLLOUT_STRATEGY="${STRATEGY_TYPE:-multi_episode}"

if [[ "$ROLLOUT_STRATEGY" != "multi_episode" ]]; then
  echo "FEHLT: Dieses Skript erwartet STRATEGY_TYPE=multi_episode." >&2
  exit 2
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
RESET_MOVE_DURATION_S=$RESET_MOVE_DURATION_S
RESET_TO_INITIAL_POSITION=$RESET_TO_INITIAL_POSITION
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
  --strategy.num_episodes="$NUM_EPISODES"
  --strategy.episode_time_s="$EPISODE_TIME_S"
  --strategy.reset_time_s="$RESET_TIME_S"
  --strategy.reset_move_duration_s="$RESET_MOVE_DURATION_S"
  --strategy.reset_to_initial_position="$RESET_TO_INITIAL_POSITION"
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

cat <<EOF_MULTI

Multi-episode session:
  Episodes:          $NUM_EPISODES
  Episode duration:  $EPISODE_TIME_S s
  Reset duration:    $RESET_TIME_S s
  Reset movement:    $RESET_MOVE_DURATION_S s
  Return to start:   $RESET_TO_INITIAL_POSITION

The model remains loaded for the complete session.
Press Ctrl+C to stop safely.

EOF_MULTI

printf 'Executing:' 
printf ' %q' "${cmd[@]}"
printf '\n'

"${cmd[@]}"