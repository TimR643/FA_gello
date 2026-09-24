#!/bin/bash
set -e

# Laptop-side tmux setup for stable two-camera recording.  Commands are filled
# into the six windows but deliberately not submitted: the operator starts them
# with Enter in the hardware-safe order documented in README.md.
# - robot control loop does NOT read/serialize camera frames
# - wrist/base cameras are exposed as ZMQ camera servers for the HPC recorder
# - recording stream sends only small robot state/action messages to the HPC

SESSION="${SESSION:-gello_panda_remote_recording}"
PROJECT_DIR="${PROJECT_DIR:-$HOME/gello_software}"
CONDA_SETUP="${CONDA_SETUP:-$HOME/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-polymetis}"

HOST="${HOST:-127.0.0.1}"
CAMERA_BIND_HOST="${CAMERA_BIND_HOST:-0.0.0.0}"
ROBOT_PORT="${ROBOT_PORT:-6001}"
WRIST_PORT="${WRIST_PORT:-5000}"
BASE_PORT="${BASE_PORT:-5001}"
WRIST_CAMERA_ID="${WRIST_CAMERA_ID:-6CD1460304A5}"
BASE_CAMERA_ID="${BASE_CAMERA_ID:-234222303420}"

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
Preparing laptop-side GELLO session: $SESSION
  Project dir:          $PROJECT_DIR
  Robot/control host:   $HOST:$ROBOT_PORT
  Camera bind host:     $CAMERA_BIND_HOST
  Wrist camera:         id=$WRIST_CAMERA_ID port=$WRIST_PORT
  Base camera:          id=$BASE_CAMERA_ID port=$BASE_PORT
  HPC recorder target:  $HPC_RECORD_HOST:$RECORD_STREAM_PORT
  Control Hz:           $CONTROL_HZ

After tmux opens, press Enter in this order:
  0 robot -> 1 gripper -> 2 nodes
  unplug the USB base camera -> 4 camera_wrist
  reconnect the USB base camera -> 3 camera_base
  align GELLO with the Panda -> 5 env

For HPC recording, start the recorder on the HPC and then run
./start_franka_to_hpc_reverse_tunnel.sh in another Franka-laptop terminal.
EOF

# Kill an old session with the same name, if present.
tmux kill-session -t "$SESSION" 2>/dev/null || true
source "$CONDA_SETUP"

tmux new-session -d -s "$SESSION"

# Window 0: Robot
tmux rename-window -t "$SESSION:0" "robot"
tmux send-keys -l -t "$SESSION:0" "source '$CONDA_SETUP' && conda activate '$CONDA_ENV' && export HYDRA_FULL_ERROR=1 && launch_robot.py robot_client=franka_hardware"

# Window 1: Gripper
tmux new-window -t "$SESSION:1" -n "gripper"
tmux send-keys -l -t "$SESSION:1" "source '$CONDA_SETUP' && conda activate '$CONDA_ENV' && launch_gripper.py gripper=franka_hand"

# Window 2: Robot ZMQ node
tmux new-window -t "$SESSION:2" -n "nodes"
tmux send-keys -l -t "$SESSION:2" "source '$CONDA_SETUP' && conda activate '$CONDA_ENV' && cd '$PROJECT_DIR' && python experiments/launch_nodes.py --robot panda --hostname '$HOST' --robot-port '$ROBOT_PORT' --robot-ip 127.0.0.1"

# Window 3: USB base camera. Start only after reconnecting it.
tmux new-window -t "$SESSION:3" -n "camera_base"
tmux send-keys -l -t "$SESSION:3" "source '$CONDA_SETUP' && conda activate '$CONDA_ENV' && cd '$PROJECT_DIR' && python -u experiments/launch_camera_single.py --hostname '$CAMERA_BIND_HOST' --port '$BASE_PORT' --camera-id '$BASE_CAMERA_ID'"

# Window 4: Wrist camera. Start while the USB base camera is unplugged.
tmux new-window -t "$SESSION:4" -n "camera_wrist"
tmux send-keys -l -t "$SESSION:4" "source '$CONDA_SETUP' && conda activate '$CONDA_ENV' && cd '$PROJECT_DIR' && python -u experiments/launch_camera_single.py --hostname '$CAMERA_BIND_HOST' --port '$WRIST_PORT' --camera-id '$WRIST_CAMERA_ID'"

# Window 5: Robot control loop, no camera clients, state/action-only recording stream
tmux new-window -t "$SESSION:5" -n "env"
tmux send-keys -l -t "$SESSION:5" "source '$CONDA_SETUP' && conda activate '$CONDA_ENV' && cd '$PROJECT_DIR' && python experiments/run_env.py --agent gello --hostname '$HOST' --robot-port '$ROBOT_PORT' --no-use-wrist-camera --no-use-base-camera --hz '$CONTROL_HZ' --use-save-interface --save-mode recording_stream --record-stream-host '$HPC_RECORD_HOST' --record-stream-port '$RECORD_STREAM_PORT' --record-stream-hwm '$RECORD_STREAM_HWM' --no-record-stream-include-camera-data $GELLO_PORT_ARG"

tmux select-window -t "$SESSION:0"
tmux attach-session -t "$SESSION"
