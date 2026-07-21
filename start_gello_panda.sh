#!/bin/bash
set -e

# Laptop-side startup for stable two-camera recording:
# - robot control loop does NOT read/serialize camera frames
# - recording stream sends only small robot state/action messages to the recorder
# - cameras should be connected to/captured on the recorder machine by default

SESSION="${SESSION:-gello_panda_remote_recording}"
PROJECT_DIR="${PROJECT_DIR:-$HOME/gello_software}"
CONDA_SETUP="${CONDA_SETUP:-$HOME/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-polymetis}"

HOST="${HOST:-127.0.0.1}"
CAMERA_BIND_HOST="${CAMERA_BIND_HOST:-0.0.0.0}"
START_LAPTOP_CAMERA_SERVERS="${START_LAPTOP_CAMERA_SERVERS:-0}"
ROBOT_PORT="${ROBOT_PORT:-6001}"
WRIST_PORT="${WRIST_PORT:-5000}"
BASE_PORT="${BASE_PORT:-5001}"
WRIST_CAMERA_ID="${WRIST_CAMERA_ID:-6CD1460304A5}"
BASE_CAMERA_ID="${BASE_CAMERA_ID:-318122303303}"

# Set this to the IP/hostname that the laptop can use to reach the HPC recorder.
HPC_RECORD_HOST="${HPC_RECORD_HOST:-172.16.0.11}"
RECORD_STREAM_PORT="${RECORD_STREAM_PORT:-7000}"
RECORD_STREAM_HWM="${RECORD_STREAM_HWM:-1}"
CONTROL_HZ="${CONTROL_HZ:-20}"

# Optional: pass GELLO_PORT=/dev/serial/by-id/... when multiple serial devices exist.
GELLO_PORT_ARG=""
if [ -n "${GELLO_PORT:-}" ]; then
  GELLO_PORT_ARG="--gello-port ${GELLO_PORT}"
fi

cat <<EOF
Starting laptop-side GELLO session: $SESSION
  Project dir:          $PROJECT_DIR
  Robot/control host:   $HOST:$ROBOT_PORT
  Laptop camera servers:$START_LAPTOP_CAMERA_SERVERS (leave 0 for Polymetis realtime safety)
  Camera bind host:     $CAMERA_BIND_HOST
  Wrist camera:         id=$WRIST_CAMERA_ID port=$WRIST_PORT
  Base camera:          id=$BASE_CAMERA_ID port=$BASE_PORT
  HPC recorder target:  $HPC_RECORD_HOST:$RECORD_STREAM_PORT
  Control Hz:           $CONTROL_HZ

Make sure the recorder is running with cameras connected to that recorder, e.g.:
  BASE_CAMERA_ID=<BASE_SERIAL> ./start_hpc_remote_camera_recorder.sh

Only set START_LAPTOP_CAMERA_SERVERS=1 for debugging without Franka realtime control.
EOF

# Kill an old session with the same name, if present.
tmux kill-session -t "$SESSION" 2>/dev/null || true
source "$CONDA_SETUP"

tmux new-session -d -s "$SESSION"

# Window 0: Robot
tmux rename-window -t "$SESSION:0" "robot"
tmux send-keys -t "$SESSION:0" "source '$CONDA_SETUP'" C-m
tmux send-keys -t "$SESSION:0" "conda activate '$CONDA_ENV'" C-m
tmux send-keys -t "$SESSION:0" "export HYDRA_FULL_ERROR=1" C-m
tmux send-keys -t "$SESSION:0" "launch_robot.py robot_client=franka_hardware" C-m
sleep 8

# Window 1: Gripper
tmux new-window -t "$SESSION:1" -n "gripper"
tmux send-keys -t "$SESSION:1" "source '$CONDA_SETUP'" C-m
tmux send-keys -t "$SESSION:1" "conda activate '$CONDA_ENV'" C-m
tmux send-keys -t "$SESSION:1" "launch_gripper.py gripper=franka_hand" C-m
sleep 5

# Window 2: Robot ZMQ node
tmux new-window -t "$SESSION:2" -n "nodes"
tmux send-keys -t "$SESSION:2" "source '$CONDA_SETUP'" C-m
tmux send-keys -t "$SESSION:2" "conda activate '$CONDA_ENV'" C-m
tmux send-keys -t "$SESSION:2" "cd '$PROJECT_DIR'" C-m
tmux send-keys -t "$SESSION:2" "python experiments/launch_nodes.py --robot panda --hostname '$HOST' --robot-port '$ROBOT_PORT' --robot-ip 127.0.0.1" C-m
sleep 3

if [ "$START_LAPTOP_CAMERA_SERVERS" = "1" ]; then
  # Debug only: camera servers on the Franka laptop can disturb Polymetis realtime.
  tmux new-window -t "$SESSION:3" -n "camera_wrist"
  tmux send-keys -t "$SESSION:3" "source '$CONDA_SETUP'" C-m
  tmux send-keys -t "$SESSION:3" "conda activate '$CONDA_ENV'" C-m
  tmux send-keys -t "$SESSION:3" "cd '$PROJECT_DIR'" C-m
  tmux send-keys -t "$SESSION:3" "python -u experiments/launch_camera_single.py --hostname '$CAMERA_BIND_HOST' --port '$WRIST_PORT' --camera-id '$WRIST_CAMERA_ID'" C-m
  sleep 5

  tmux new-window -t "$SESSION:4" -n "camera_base"
  tmux send-keys -t "$SESSION:4" "source '$CONDA_SETUP'" C-m
  tmux send-keys -t "$SESSION:4" "conda activate '$CONDA_ENV'" C-m
  tmux send-keys -t "$SESSION:4" "cd '$PROJECT_DIR'" C-m
  tmux send-keys -t "$SESSION:4" "python -u experiments/launch_camera_single.py --hostname '$CAMERA_BIND_HOST' --port '$BASE_PORT' --camera-id '$BASE_CAMERA_ID'" C-m
  sleep 3
fi

# Window 5: Robot control loop, no camera clients, state/action-only recording stream
tmux new-window -t "$SESSION:5" -n "env"
tmux send-keys -t "$SESSION:5" "source '$CONDA_SETUP'" C-m
tmux send-keys -t "$SESSION:5" "conda activate '$CONDA_ENV'" C-m
tmux send-keys -t "$SESSION:5" "cd '$PROJECT_DIR'" C-m
tmux send-keys -t "$SESSION:5" "python experiments/run_env.py --agent gello --hostname '$HOST' --robot-port '$ROBOT_PORT' --no-use-wrist-camera --no-use-base-camera --hz '$CONTROL_HZ' --use-save-interface --save-mode recording_stream --record-stream-host '$HPC_RECORD_HOST' --record-stream-port '$RECORD_STREAM_PORT' --record-stream-hwm '$RECORD_STREAM_HWM' --no-record-stream-include-camera-data $GELLO_PORT_ARG" C-m

tmux select-window -t "$SESSION:5"
tmux attach-session -t "$SESSION"
