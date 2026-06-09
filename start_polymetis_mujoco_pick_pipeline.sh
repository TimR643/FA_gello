#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
MUJOCO_DIR="${MUJOCO_DIR:-/home/tim/mujoco-3.9.0-linux-x86_64}"
SESSION="${SESSION:-polymetis_mujoco_pick_pipeline}"
CONDA_SETUP="${CONDA_SETUP:-$HOME/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-polymetis}"
HOST="${HOST:-127.0.0.1}"
ROBOT_PORT="${ROBOT_PORT:-6001}"
WRIST_PORT="${WRIST_PORT:-5000}"
RECORD_STREAM_PORT="${RECORD_STREAM_PORT:-7000}"
SAVE_MODE="${SAVE_MODE:-lerobot}"
START_POLYMETIS_SIM="${START_POLYMETIS_SIM:-1}"
START_ROBOT_ZMQ="${START_ROBOT_ZMQ:-1}"
START_WRIST_CAMERA="${START_WRIST_CAMERA:-0}"
RESET_STALE_POLYMETIS="${RESET_STALE_POLYMETIS:-1}"
POLYMETIS_GRPC_PORT="${POLYMETIS_GRPC_PORT:-50051}"
POLYMETIS_READY_TIMEOUT="${POLYMETIS_READY_TIMEOUT:-60}"
MUJOCO_GL="${MUJOCO_GL:-glfw}"
MUJOCO_GUI="${MUJOCO_GUI:-true}"
LEROBOT_ROOT="${LEROBOT_ROOT:-~/lerobot_data/polymetis_mujoco_pick}"
LEROBOT_REPO_ID="${LEROBOT_REPO_ID:-local/polymetis_mujoco_pick_wrist}"
POLYMETIS_SIM_CMD="${POLYMETIS_SIM_CMD:-launch_robot.py robot_client=mujoco_sim use_real_time=false gui=$MUJOCO_GUI}"
WRIST_CAMERA_CMD="${WRIST_CAMERA_CMD:-}"

export MUJOCO_PATH="$MUJOCO_DIR"
export LD_LIBRARY_PATH="$MUJOCO_DIR/lib:${LD_LIBRARY_PATH:-}"
export MUJOCO_GL="$MUJOCO_GL"
SESSION_STARTED=0
cleanup_on_error() {
  local status=$?
  if [[ $status -ne 0 && "$SESSION_STARTED" == "1" ]]; then
    echo "Launcher failed; stopping tmux session $SESSION" >&2
    tmux kill-session -t "$SESSION" 2>/dev/null || true
  fi
}
trap cleanup_on_error EXIT

activate_conda_env() {
  # Some conda activation hooks (for example MKL) read unset variables.
  # The launcher itself uses `set -u`, so temporarily relax nounset while
  # sourcing conda and activating the runtime environment.
  local status=0
  set +u
  source "$CONDA_SETUP" || status=$?
  if [[ $status -eq 0 ]]; then
    conda activate "$CONDA_ENV" || status=$?
  fi
  set -u
  return "$status"
}

TMUX_ACTIVATE_CMD="set +u; source '$CONDA_SETUP'; conda activate '$CONDA_ENV'; set -u"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required for this launcher." >&2
  exit 1
fi

if [[ ! -f "$CONDA_SETUP" ]]; then
  echo "Conda setup script not found: $CONDA_SETUP" >&2
  echo "Set CONDA_SETUP=/path/to/conda.sh if your conda install lives elsewhere." >&2
  exit 1
fi

if [[ ! -d "$MUJOCO_DIR" ]]; then
  echo "MuJoCo directory not found: $MUJOCO_DIR" >&2
  echo "Set MUJOCO_DIR=/home/tim/mujoco-3.9.0-linux-x86_64 or your actual MuJoCo path." >&2
  exit 1
fi

if [[ "$MUJOCO_GUI" == "true" && -z "${DISPLAY:-}" ]]; then
  echo "Warning: MUJOCO_GUI=true but DISPLAY is not set; the Franka viewer may not appear." >&2
fi

echo "MuJoCo visualization: gui=$MUJOCO_GUI, MUJOCO_GL=$MUJOCO_GL"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' already exists. Attach with: tmux attach -t $SESSION" >&2
  echo "Or stop it first with: tmux kill-session -t $SESSION" >&2
  exit 1
fi

if [[ "$START_POLYMETIS_SIM" == "1" && "$RESET_STALE_POLYMETIS" == "1" ]]; then
  echo "Stopping stale local Polymetis simulator processes before starting MuJoCo..."
  pkill -9 -f "run_server" 2>/dev/null || true
  pkill -9 -f "launch_robot.py.*robot_client=.*sim" 2>/dev/null || true
  sleep 1
fi

if [[ "$SAVE_MODE" == "lerobot" || "$SAVE_MODE" == "recording_stream" ]]; then
  if ! activate_conda_env >/dev/null 2>&1 || ! python - <<'PY' >/dev/null 2>&1
from lerobot.datasets import LeRobotDataset  # noqa: F401
PY
  then
    cat >&2 <<EOF2
LeRobot is not importable in conda env '$CONDA_ENV'.
Install LeRobot in that env before recording, or run a movement-only smoke test with:
  SAVE_MODE=none ./start_polymetis_mujoco_pick_pipeline.sh
EOF2
    exit 1
  fi
fi

WAIT_FOR_POLYMETIS_CMD=$(cat <<'PY'
import sys
import time
from polymetis import RobotInterface

host = sys.argv[1]
port = int(sys.argv[2])
timeout = float(sys.argv[3])
deadline = time.time() + timeout
last_exc = None
while time.time() < deadline:
    try:
        robot = RobotInterface(ip_address=host, port=port)
        robot.get_joint_positions()
        print(f"Polymetis robot server is ready at {host}:{port}")
        break
    except Exception as exc:
        last_exc = exc
        time.sleep(1.0)
else:
    print(
        f"Timed out waiting for a valid Polymetis robot server at {host}:{port}.\n"
        f"Last error: {last_exc}\n"
        "If you see 'Robot context not valid' or 'Port unavailable', kill stale servers with:\n"
        "  pkill -9 run_server\n"
        "Then restart this launcher.",
        file=sys.stderr,
    )
    sys.exit(1)
PY
)

tmux new-session -d -s "$SESSION" -n "pipeline"
SESSION_STARTED=1

if [[ "$START_POLYMETIS_SIM" == "1" ]]; then
  tmux new-window -t "$SESSION:1" -n "polymetis_sim"
  tmux send-keys -t "$SESSION:1" "$TMUX_ACTIVATE_CMD" C-m
  tmux send-keys -t "$SESSION:1" "export MUJOCO_PATH='$MUJOCO_DIR'" C-m
  tmux send-keys -t "$SESSION:1" "export LD_LIBRARY_PATH='$MUJOCO_DIR/lib:\${LD_LIBRARY_PATH:-}'" C-m
  tmux send-keys -t "$SESSION:1" "export MUJOCO_GL='$MUJOCO_GL'" C-m
  tmux send-keys -t "$SESSION:1" "$POLYMETIS_SIM_CMD" C-m
fi

# Wait in the foreground as well so the ZMQ node is not started against a stale or half-started robot context.
if [[ "$START_POLYMETIS_SIM" == "1" || "$START_ROBOT_ZMQ" == "1" ]]; then
  activate_conda_env
  python -c "$WAIT_FOR_POLYMETIS_CMD" "$HOST" "$POLYMETIS_GRPC_PORT" "$POLYMETIS_READY_TIMEOUT"
fi

if [[ "$START_ROBOT_ZMQ" == "1" ]]; then
  tmux new-window -t "$SESSION:2" -n "robot_zmq"
  tmux send-keys -t "$SESSION:2" "$TMUX_ACTIVATE_CMD" C-m
  tmux send-keys -t "$SESSION:2" "cd '$PROJECT_DIR'" C-m
  tmux send-keys -t "$SESSION:2" "python -u experiments/launch_nodes.py --robot panda --hostname '$HOST' --robot_port $ROBOT_PORT --robot-ip 127.0.0.1" C-m
  sleep 3
fi

if [[ "$START_WRIST_CAMERA" == "1" ]]; then
  if [[ -z "$WRIST_CAMERA_CMD" ]]; then
    echo "START_WRIST_CAMERA=1 requires WRIST_CAMERA_CMD to start a MuJoCo wrist-camera ZMQ server." >&2
    exit 1
  fi
  tmux new-window -t "$SESSION:3" -n "wrist_camera"
  tmux send-keys -t "$SESSION:3" "$TMUX_ACTIVATE_CMD" C-m
  tmux send-keys -t "$SESSION:3" "cd '$PROJECT_DIR'" C-m
  tmux send-keys -t "$SESSION:3" "$WRIST_CAMERA_CMD" C-m
  sleep 2
fi

if [[ "$SAVE_MODE" == "recording_stream" ]]; then
  tmux new-window -t "$SESSION:4" -n "recorder"
  tmux send-keys -t "$SESSION:4" "$TMUX_ACTIVATE_CMD" C-m
  tmux send-keys -t "$SESSION:4" "cd '$PROJECT_DIR'" C-m
  tmux send-keys -t "$SESSION:4" "python -u experiments/record_lerobot_stream.py --port $RECORD_STREAM_PORT --lerobot-root '$LEROBOT_ROOT' --lerobot-repo-id '$LEROBOT_REPO_ID' --lerobot-task 'Pick the cube in MuJoCo through the Polymetis Panda interface.'" C-m
  sleep 2
fi

CAMERA_ARGS="--no-use-wrist-camera"
if [[ "$START_WRIST_CAMERA" == "1" ]]; then
  CAMERA_ARGS="--use-wrist-camera"
fi

tmux send-keys -t "$SESSION:0" "$TMUX_ACTIVATE_CMD" C-m
tmux send-keys -t "$SESSION:0" "cd '$PROJECT_DIR'" C-m
tmux send-keys -t "$SESSION:0" "python -u experiments/run_polymetis_mujoco_pick_pipeline.py --save-mode $SAVE_MODE --robot-host '$HOST' --robot-port $ROBOT_PORT --wrist-camera-host '$HOST' --wrist-camera-port $WRIST_PORT --record-stream-port $RECORD_STREAM_PORT --lerobot-root '$LEROBOT_ROOT' --lerobot-repo-id '$LEROBOT_REPO_ID' $CAMERA_ARGS" C-m

tmux select-window -t "$SESSION:0"
tmux attach-session -t "$SESSION"
