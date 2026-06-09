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
SIM_BACKEND="${SIM_BACKEND:-external_fer_polymetis}"
FER_POLYMETIS_SIM_CMD="${FER_POLYMETIS_SIM_CMD:-}"
START_ROBOT_ZMQ="${START_ROBOT_ZMQ:-1}"
START_WRIST_CAMERA="${START_WRIST_CAMERA:-0}"
PANDA_USE_GRIPPER="${PANDA_USE_GRIPPER:-0}"
PANDA_INITIALIZE_ROBOT="${PANDA_INITIALIZE_ROBOT:-0}"
PANDA_MANUAL_GRIPPER_OVERRIDE="${PANDA_MANUAL_GRIPPER_OVERRIDE:-0}"
ZMQ_READY_TIMEOUT="${ZMQ_READY_TIMEOUT:-20}"
RESET_STALE_POLYMETIS="${RESET_STALE_POLYMETIS:-1}"
POLYMETIS_GRPC_PORT="${POLYMETIS_GRPC_PORT:-50051}"
POLYMETIS_READY_TIMEOUT="${POLYMETIS_READY_TIMEOUT:-60}"
AUTO_SELECT_POLYMETIS_PORT="${AUTO_SELECT_POLYMETIS_PORT:-1}"
LOG_DIR="${LOG_DIR:-/tmp/${SESSION}_logs}"
RESTART_EXISTING_SESSION="${RESTART_EXISTING_SESSION:-1}"
KEEP_TMUX_ON_FAILURE="${KEEP_TMUX_ON_FAILURE:-1}"
PYTHON_BIN="${PYTHON_BIN:-python}"
POLYMETIS_SIM_METADATA_OVERRIDES="${POLYMETIS_SIM_METADATA_OVERRIDES:-}"
MUJOCO_GL="${MUJOCO_GL:-glfw}"
MUJOCO_GUI="${MUJOCO_GUI:-true}"
LEROBOT_ROOT="${LEROBOT_ROOT:-~/lerobot_data/polymetis_mujoco_pick}"
LEROBOT_REPO_ID="${LEROBOT_REPO_ID:-local/polymetis_mujoco_pick_wrist}"
POLYMETIS_SIM_CMD_USER_SET=0
if [[ -n "${POLYMETIS_SIM_CMD+x}" ]]; then
  POLYMETIS_SIM_CMD_USER_SET=1
fi
POLYMETIS_SIM_CMD="${POLYMETIS_SIM_CMD:-}"
WRIST_CAMERA_CMD="${WRIST_CAMERA_CMD:-}"

export MUJOCO_PATH="$MUJOCO_DIR"
export LD_LIBRARY_PATH="$MUJOCO_DIR/lib:${LD_LIBRARY_PATH:-}"
export MUJOCO_GL="$MUJOCO_GL"
export HYDRA_FULL_ERROR="${HYDRA_FULL_ERROR:-1}"
SESSION_STARTED=0
cleanup_on_error() {
  local status=$?
  if [[ $status -ne 0 && "$SESSION_STARTED" == "1" ]]; then
    mkdir -p "$LOG_DIR"
    for pane in $(tmux list-panes -a -F '#{session_name}:#{window_index}.#{pane_index}' 2>/dev/null | grep "^$SESSION:" || true); do
      local safe_pane="${pane//[:.]/_}"
      tmux capture-pane -p -S -2000 -t "$pane" > "$LOG_DIR/$safe_pane.log" 2>/dev/null || true
    done
    echo "Launcher failed; captured tmux pane logs in $LOG_DIR" >&2
    if [[ "$KEEP_TMUX_ON_FAILURE" == "1" ]]; then
      echo "Keeping tmux session $SESSION alive for inspection. Attach with: tmux attach -t $SESSION" >&2
      echo "Run ./stop_polymetis_mujoco_pick_pipeline.sh before the next clean start, or keep RESTART_EXISTING_SESSION=1." >&2
    else
      echo "Stopping tmux session $SESSION" >&2
      tmux kill-session -t "$SESSION" 2>/dev/null || true
    fi
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

port_is_open() {
  "$PYTHON_BIN" - "$HOST" "$1" <<'PYPORTOPEN'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(0.25)
try:
    sock.connect((host, port))
except OSError:
    sys.exit(1)
else:
    sys.exit(0)
finally:
    sock.close()
PYPORTOPEN
}

find_free_polymetis_port() {
  "$PYTHON_BIN" - "$HOST" "$POLYMETIS_GRPC_PORT" <<'PYFREEPORT'
import socket
import sys

host = sys.argv[1]
# Avoid 50052 because Polymetis commonly uses it for the gripper server.
start = max(int(sys.argv[2]) + 1, 50100)
for port in range(start, start + 100):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind((host, port))
    except OSError:
        continue
    finally:
        sock.close()
    print(port)
    sys.exit(0)
raise SystemExit("No free Polymetis port found")
PYFREEPORT
}

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

activate_conda_env
PYTHON_BIN="$(command -v python || command -v python3 || true)"
if [[ -z "$PYTHON_BIN" ]]; then
  echo "No python executable found after activating conda env '$CONDA_ENV'." >&2
  exit 1
fi

echo "Using Python: $PYTHON_BIN"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  if [[ "$RESTART_EXISTING_SESSION" == "1" ]]; then
    echo "tmux session '$SESSION' already exists; restarting it for a clean launch."
    tmux kill-session -t "$SESSION"
  else
    echo "tmux session '$SESSION' already exists. Attach with: tmux attach -t $SESSION" >&2
    echo "Or stop it first with: tmux kill-session -t $SESSION" >&2
    exit 1
  fi
fi

if [[ "$START_POLYMETIS_SIM" == "1" && "$RESET_STALE_POLYMETIS" == "1" ]]; then
  echo "Stopping stale local Polymetis simulator processes before starting MuJoCo..."
  pkill -9 -f "run_server" 2>/dev/null || true
  pkill -9 -f "launch_robot.py.*robot_client=.*sim" 2>/dev/null || true
  sleep 1
fi

if [[ "$START_POLYMETIS_SIM" == "1" ]] && port_is_open "$POLYMETIS_GRPC_PORT"; then
  if [[ "$AUTO_SELECT_POLYMETIS_PORT" == "1" ]]; then
    OLD_POLYMETIS_GRPC_PORT="$POLYMETIS_GRPC_PORT"
    POLYMETIS_GRPC_PORT="$(find_free_polymetis_port)"
    echo "Port $OLD_POLYMETIS_GRPC_PORT is still occupied after cleanup; using Polymetis port $POLYMETIS_GRPC_PORT for this run."
  else
    echo "Polymetis port $POLYMETIS_GRPC_PORT is still occupied after cleanup." >&2
    echo "Run ./stop_polymetis_mujoco_pick_pipeline.sh or choose another port with POLYMETIS_GRPC_PORT=50100." >&2
    exit 1
  fi
fi

if [[ "$START_POLYMETIS_SIM" == "1" ]]; then
case "$SIM_BACKEND" in
  external_fer_polymetis)
    if [[ -z "$FER_POLYMETIS_SIM_CMD" && "$POLYMETIS_SIM_CMD_USER_SET" == "0" ]]; then
      cat >&2 <<EOF2
No FER-compatible Polymetis simulator command configured.
The built-in Polymetis robot_client=mujoco_sim does not reproduce the FER MuJoCo scene
(table/cube/wrist camera) and should not be used for the real pipeline simulation.

Set FER_POLYMETIS_SIM_CMD to the command that starts your FER MuJoCo scene as a
Polymetis robot server. Use {POLYMETIS_GRPC_PORT} as a placeholder for the selected port, for example:
  FER_POLYMETIS_SIM_CMD='python -u /path/to/fer_polymetis_server.py --port {POLYMETIS_GRPC_PORT} --gui'

If you only want the old toy arm smoke test, run explicitly:
  SIM_BACKEND=polymetis_builtin_smoke SAVE_MODE=none ./start_polymetis_mujoco_pick_pipeline.sh
EOF2
      exit 1
    fi
    if [[ "$POLYMETIS_SIM_CMD_USER_SET" == "0" ]]; then
      POLYMETIS_SIM_CMD="$FER_POLYMETIS_SIM_CMD"
      POLYMETIS_SIM_CMD="${POLYMETIS_SIM_CMD//\{POLYMETIS_GRPC_PORT\}/$POLYMETIS_GRPC_PORT}"
      POLYMETIS_SIM_CMD="${POLYMETIS_SIM_CMD//\{MUJOCO_GUI\}/$MUJOCO_GUI}"
      POLYMETIS_SIM_CMD="${POLYMETIS_SIM_CMD//\{MUJOCO_GL\}/$MUJOCO_GL}"
    fi
    if [[ "$POLYMETIS_SIM_CMD" == *"robot_client=mujoco_sim"* ]]; then
      cat >&2 <<EOF2
Refusing to use robot_client=mujoco_sim for SIM_BACKEND=external_fer_polymetis.
That built-in simulator is the wrong scene (the gray toy arm you saw), not the FER table/cube/wrist-camera environment.
Use SIM_BACKEND=polymetis_builtin_smoke only for a control-path smoke test.
EOF2
      exit 1
    fi
    ;;
  polymetis_builtin_smoke)
    if [[ -z "$POLYMETIS_SIM_METADATA_OVERRIDES" ]]; then
      POLYMETIS_SIM_METADATA_OVERRIDES="'+default_Kq=[150,150,150,150,150,150,150]' '+default_Kqd=[10,10,10,10,10,10,10]' '+default_Kx=[50,50,50,50,50,50]' '+default_Kxd=[10,10,10,10,10,10]'"
    fi
    if [[ "$POLYMETIS_SIM_CMD_USER_SET" == "0" ]]; then
      POLYMETIS_SIM_CMD="launch_robot.py robot_client=mujoco_sim use_real_time=false gui=$MUJOCO_GUI port=$POLYMETIS_GRPC_PORT $POLYMETIS_SIM_METADATA_OVERRIDES"
    else
      echo "Using custom POLYMETIS_SIM_CMD. Make sure it binds to port $POLYMETIS_GRPC_PORT or set POLYMETIS_GRPC_PORT to match it."
    fi
    echo "WARNING: SIM_BACKEND=polymetis_builtin_smoke is only a toy Polymetis control-path smoke test, not the FER scene."
    ;;
  *)
    echo "Unknown SIM_BACKEND=$SIM_BACKEND. Use external_fer_polymetis or polymetis_builtin_smoke." >&2
    exit 1
    ;;
esac

echo "Simulation backend: $SIM_BACKEND"
echo "Polymetis sim command: $POLYMETIS_SIM_CMD"
else
  echo "START_POLYMETIS_SIM=0; assuming an external Polymetis-compatible simulator is already running on $HOST:$POLYMETIS_GRPC_PORT."
fi

if [[ "$SAVE_MODE" == "lerobot" || "$SAVE_MODE" == "recording_stream" ]]; then
  if ! "$PYTHON_BIN" - <<'PY' >/dev/null 2>&1
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
        "The launcher keeps/captures tmux logs on failure. Inspect the polymetis_sim log "
        "to see the underlying MuJoCo/Polymetis error.",
        file=sys.stderr,
    )
    sys.exit(1)
PY
)

WAIT_FOR_TCP_PORT_CMD=$(cat <<'PYTCPWAIT'
import socket
import sys
import time

host = sys.argv[1]
port = int(sys.argv[2])
timeout = float(sys.argv[3])
deadline = time.time() + timeout
while time.time() < deadline:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.5)
    try:
        sock.connect((host, port))
    except OSError:
        time.sleep(0.25)
    else:
        sock.close()
        print(f"TCP server is ready at {host}:{port}")
        break
    finally:
        sock.close()
else:
    print(f"Timed out waiting for TCP server at {host}:{port}", file=sys.stderr)
    sys.exit(1)
PYTCPWAIT
)

tmux new-session -d -s "$SESSION" -n "pipeline"
SESSION_STARTED=1

if [[ "$START_POLYMETIS_SIM" == "1" ]]; then
  tmux new-window -t "$SESSION:1" -n "polymetis_sim"
  tmux send-keys -t "$SESSION:1" "$TMUX_ACTIVATE_CMD" C-m
  tmux send-keys -t "$SESSION:1" "export MUJOCO_PATH='$MUJOCO_DIR'" C-m
  tmux send-keys -t "$SESSION:1" "export LD_LIBRARY_PATH='$MUJOCO_DIR/lib:\${LD_LIBRARY_PATH:-}'" C-m
  tmux send-keys -t "$SESSION:1" "export MUJOCO_GL='$MUJOCO_GL'" C-m
  tmux send-keys -t "$SESSION:1" "export HYDRA_FULL_ERROR='${HYDRA_FULL_ERROR:-1}'" C-m
  tmux send-keys -t "$SESSION:1" "$POLYMETIS_SIM_CMD" C-m
fi

# Wait in the foreground as well so the ZMQ node is not started against a stale or half-started robot context.
if [[ "$START_POLYMETIS_SIM" == "1" || "$START_ROBOT_ZMQ" == "1" ]]; then
  "$PYTHON_BIN" -c "$WAIT_FOR_POLYMETIS_CMD" "$HOST" "$POLYMETIS_GRPC_PORT" "$POLYMETIS_READY_TIMEOUT"
fi

PANDA_GRIPPER_ARG="--no-panda-use-gripper"
if [[ "$PANDA_USE_GRIPPER" == "1" ]]; then
  PANDA_GRIPPER_ARG="--panda-use-gripper"
fi
PANDA_INIT_ARG="--no-panda-initialize-robot"
if [[ "$PANDA_INITIALIZE_ROBOT" == "1" ]]; then
  PANDA_INIT_ARG="--panda-initialize-robot"
fi
PANDA_MANUAL_ARG="--no-panda-manual-gripper-override"
if [[ "$PANDA_MANUAL_GRIPPER_OVERRIDE" == "1" ]]; then
  PANDA_MANUAL_ARG="--panda-manual-gripper-override"
fi

if [[ "$START_ROBOT_ZMQ" == "1" ]]; then
  tmux new-window -t "$SESSION:2" -n "robot_zmq"
  tmux send-keys -t "$SESSION:2" "$TMUX_ACTIVATE_CMD" C-m
  tmux send-keys -t "$SESSION:2" "cd '$PROJECT_DIR'" C-m
  tmux send-keys -t "$SESSION:2" "python -u experiments/launch_nodes.py --robot panda --hostname '$HOST' --robot_port $ROBOT_PORT --robot-ip 127.0.0.1 --polymetis-port $POLYMETIS_GRPC_PORT $PANDA_GRIPPER_ARG $PANDA_INIT_ARG $PANDA_MANUAL_ARG" C-m
  "$PYTHON_BIN" -c "$WAIT_FOR_TCP_PORT_CMD" "$HOST" "$ROBOT_PORT" "$ZMQ_READY_TIMEOUT"
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
