#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
FER_WS="${FER_WS:-$HOME/fer_ros2_simulation/ros2_ws}"
MUJOCO_DIR="${MUJOCO_DIR:-/home/tim/mujoco-3.9.0-linux-x86_64}"
SESSION="${SESSION:-fer_mujoco_pick_pipeline}"
SAVE_MODE="${SAVE_MODE:-lerobot}"
START_SIM="${START_SIM:-1}"
USE_RVIZ="${USE_RVIZ:-false}"
RECORD_STREAM_PORT="${RECORD_STREAM_PORT:-7000}"
LEROBOT_ROOT="${LEROBOT_ROOT:-~/lerobot_data/fer_mujoco_pick}"
LEROBOT_REPO_ID="${LEROBOT_REPO_ID:-local/fer_mujoco_pick_wrist}"
WRIST_RGB_TOPIC="${WRIST_RGB_TOPIC:-/wrist_camera/image_raw}"
WRIST_DEPTH_TOPIC="${WRIST_DEPTH_TOPIC:-/wrist_camera/depth/image_raw}"

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

if [[ "$START_SIM" == "1" ]]; then
  tmux new-window -t "$SESSION:1" -n "fer_sim"
  tmux send-keys -t "$SESSION:1" "source '$FER_WS/install/setup.bash'" C-m
  tmux send-keys -t "$SESSION:1" "export MUJOCO_PATH='$MUJOCO_DIR'" C-m
  tmux send-keys -t "$SESSION:1" "export LD_LIBRARY_PATH='$MUJOCO_DIR/lib:\${LD_LIBRARY_PATH:-}'" C-m
  tmux send-keys -t "$SESSION:1" "ros2 launch franka_mujoco_sim_bringup fer_mujoco_ros2_control.launch.py use_rviz:=$USE_RVIZ arm_control_type:=effort hand_control_type:=effort" C-m
  sleep 8
fi

if [[ "$SAVE_MODE" == "recording_stream" ]]; then
  tmux new-window -t "$SESSION:2" -n "recorder"
  tmux send-keys -t "$SESSION:2" "cd '$PROJECT_DIR'" C-m
  tmux send-keys -t "$SESSION:2" "python -u experiments/record_lerobot_stream.py --port $RECORD_STREAM_PORT --lerobot-root '$LEROBOT_ROOT' --lerobot-repo-id '$LEROBOT_REPO_ID' --lerobot-task 'Pick the cube from the table in the FER MuJoCo simulator.'" C-m
  sleep 2
fi

tmux send-keys -t "$SESSION:0" "cd '$PROJECT_DIR'" C-m
tmux send-keys -t "$SESSION:0" "source '$FER_WS/install/setup.bash'" C-m
tmux send-keys -t "$SESSION:0" "export MUJOCO_PATH='$MUJOCO_DIR'" C-m
tmux send-keys -t "$SESSION:0" "export LD_LIBRARY_PATH='$MUJOCO_DIR/lib:\${LD_LIBRARY_PATH:-}'" C-m
tmux send-keys -t "$SESSION:0" "python -u experiments/run_fer_mujoco_pick_pipeline.py --save-mode $SAVE_MODE --record-stream-port $RECORD_STREAM_PORT --lerobot-root '$LEROBOT_ROOT' --lerobot-repo-id '$LEROBOT_REPO_ID' --wrist-rgb-topic '$WRIST_RGB_TOPIC' --wrist-depth-topic '$WRIST_DEPTH_TOPIC'" C-m

tmux select-window -t "$SESSION:0"
tmux attach-session -t "$SESSION"
