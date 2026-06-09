#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
MUJOCO_DIR="${MUJOCO_DIR:-/home/tim/mujoco-3.9.0-linux-x86_64}"
SESSION="${SESSION:-mujoco_pick_pipeline}"
SAVE_MODE="${SAVE_MODE:-lerobot}"
RECORD_STREAM_PORT="${RECORD_STREAM_PORT:-7000}"
LEROBOT_ROOT="${LEROBOT_ROOT:-~/lerobot_data/mujoco_panda_pick}"
LEROBOT_REPO_ID="${LEROBOT_REPO_ID:-local/mujoco_panda_pick_wrist}"

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

tmux new-session -d -s "$SESSION" -n "sim_pick"

if [[ "$SAVE_MODE" == "recording_stream" ]]; then
  tmux new-window -t "$SESSION:1" -n "recorder"
  tmux send-keys -t "$SESSION:1" "cd '$PROJECT_DIR'" C-m
  tmux send-keys -t "$SESSION:1" "python -u experiments/record_lerobot_stream.py --port $RECORD_STREAM_PORT --lerobot-root '$LEROBOT_ROOT' --lerobot-repo-id '$LEROBOT_REPO_ID' --lerobot-task 'Pick the red cube from the table in MuJoCo.' --no-lerobot-streaming-encoding" C-m
  sleep 2
fi

tmux send-keys -t "$SESSION:0" "cd '$PROJECT_DIR'" C-m
tmux send-keys -t "$SESSION:0" "export MUJOCO_PATH='$MUJOCO_DIR'" C-m
tmux send-keys -t "$SESSION:0" "export LD_LIBRARY_PATH='$MUJOCO_DIR/lib:\${LD_LIBRARY_PATH:-}'" C-m
tmux send-keys -t "$SESSION:0" "python -u experiments/run_mujoco_pick_pipeline.py --save-mode $SAVE_MODE --record-stream-port $RECORD_STREAM_PORT --lerobot-root '$LEROBOT_ROOT' --lerobot-repo-id '$LEROBOT_REPO_ID'" C-m

tmux select-window -t "$SESSION:0"
tmux attach-session -t "$SESSION"
