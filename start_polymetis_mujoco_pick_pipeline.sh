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
LEROBOT_ROOT="${LEROBOT_ROOT:-~/lerobot_data/polymetis_mujoco_pick}"
LEROBOT_REPO_ID="${LEROBOT_REPO_ID:-local/polymetis_mujoco_pick_wrist}"
POLYMETIS_SIM_CMD="${POLYMETIS_SIM_CMD:-launch_robot.py robot_client=mujoco_sim use_real_time=false gui=true}"
WRIST_CAMERA_CMD="${WRIST_CAMERA_CMD:-}"

export MUJOCO_PATH="$MUJOCO_DIR"
export LD_LIBRARY_PATH="$MUJOCO_DIR/lib:${LD_LIBRARY_PATH:-}"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required for this launcher." >&2
  exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' already exists. Attach with: tmux attach -t $SESSION" >&2
  exit 1
fi

tmux new-session -d -s "$SESSION" -n "pipeline"

if [[ "$START_POLYMETIS_SIM" == "1" ]]; then
  tmux new-window -t "$SESSION:1" -n "polymetis_sim"
  tmux send-keys -t "$SESSION:1" "source '$CONDA_SETUP'" C-m
  tmux send-keys -t "$SESSION:1" "conda activate '$CONDA_ENV'" C-m
  tmux send-keys -t "$SESSION:1" "export MUJOCO_PATH='$MUJOCO_DIR'" C-m
  tmux send-keys -t "$SESSION:1" "export LD_LIBRARY_PATH='$MUJOCO_DIR/lib:\${LD_LIBRARY_PATH:-}'" C-m
  tmux send-keys -t "$SESSION:1" "$POLYMETIS_SIM_CMD" C-m
  sleep 5
fi

if [[ "$START_ROBOT_ZMQ" == "1" ]]; then
  tmux new-window -t "$SESSION:2" -n "robot_zmq"
  tmux send-keys -t "$SESSION:2" "source '$CONDA_SETUP'" C-m
  tmux send-keys -t "$SESSION:2" "conda activate '$CONDA_ENV'" C-m
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
  tmux send-keys -t "$SESSION:3" "source '$CONDA_SETUP'" C-m
  tmux send-keys -t "$SESSION:3" "conda activate '$CONDA_ENV'" C-m
  tmux send-keys -t "$SESSION:3" "cd '$PROJECT_DIR'" C-m
  tmux send-keys -t "$SESSION:3" "$WRIST_CAMERA_CMD" C-m
  sleep 2
fi

if [[ "$SAVE_MODE" == "recording_stream" ]]; then
  tmux new-window -t "$SESSION:4" -n "recorder"
  tmux send-keys -t "$SESSION:4" "source '$CONDA_SETUP'" C-m
  tmux send-keys -t "$SESSION:4" "conda activate '$CONDA_ENV'" C-m
  tmux send-keys -t "$SESSION:4" "cd '$PROJECT_DIR'" C-m
  tmux send-keys -t "$SESSION:4" "python -u experiments/record_lerobot_stream.py --port $RECORD_STREAM_PORT --lerobot-root '$LEROBOT_ROOT' --lerobot-repo-id '$LEROBOT_REPO_ID' --lerobot-task 'Pick the cube in MuJoCo through the Polymetis Panda interface.'" C-m
  sleep 2
fi

CAMERA_ARGS="--no-use-wrist-camera"
if [[ "$START_WRIST_CAMERA" == "1" ]]; then
  CAMERA_ARGS="--use-wrist-camera"
fi

tmux send-keys -t "$SESSION:0" "source '$CONDA_SETUP'" C-m
tmux send-keys -t "$SESSION:0" "conda activate '$CONDA_ENV'" C-m
tmux send-keys -t "$SESSION:0" "cd '$PROJECT_DIR'" C-m
tmux send-keys -t "$SESSION:0" "python -u experiments/run_polymetis_mujoco_pick_pipeline.py --save-mode $SAVE_MODE --robot-host '$HOST' --robot-port $ROBOT_PORT --wrist-camera-host '$HOST' --wrist-camera-port $WRIST_PORT --record-stream-port $RECORD_STREAM_PORT --lerobot-root '$LEROBOT_ROOT' --lerobot-repo-id '$LEROBOT_REPO_ID' $CAMERA_ARGS" C-m

tmux select-window -t "$SESSION:0"
tmux attach-session -t "$SESSION"
