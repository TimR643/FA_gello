#!/bin/bash

SESSION="gello_panda_onecam_stream"
PROJECT_DIR="~/gello_software"
HOST="127.0.0.1"
ROBOT_PORT=6001
WRIST_PORT=5000
WRIST_CAMERA_ID="6CD1460304A5"

HPC_RECORD_HOST="172.16.0.11"
RECORD_STREAM_PORT=7000
RECORD_STREAM_HWM=2

tmux kill-session -t $SESSION 2>/dev/null
source ~/miniconda3/etc/profile.d/conda.sh

tmux new-session -d -s $SESSION

# Fenster 0: Robot
tmux rename-window -t $SESSION:0 "robot"
tmux send-keys -t $SESSION:0 "source ~/miniconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t $SESSION:0 "conda activate polymetis" C-m
tmux send-keys -t $SESSION:0 "export HYDRA_FULL_ERROR=1" C-m
tmux send-keys -t $SESSION:0 "launch_robot.py robot_client=franka_hardware" C-m
sleep 8

# Fenster 1: Gripper
tmux new-window -t $SESSION:1 -n "gripper"
tmux send-keys -t $SESSION:1 "source ~/miniconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t $SESSION:1 "conda activate polymetis" C-m
tmux send-keys -t $SESSION:1 "launch_gripper.py gripper=franka_hand" C-m
sleep 5

# Fenster 2: Robot ZMQ Node
tmux new-window -t $SESSION:2 -n "nodes"
tmux send-keys -t $SESSION:2 "source ~/miniconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t $SESSION:2 "conda activate polymetis" C-m
tmux send-keys -t $SESSION:2 "cd $PROJECT_DIR" C-m
tmux send-keys -t $SESSION:2 "python experiments/launch_nodes.py --robot panda --hostname $HOST --robot_port $ROBOT_PORT --robot-ip 127.0.0.1" C-m
sleep 3

# Fenster 3: Wrist Camera ZMQ Node
tmux new-window -t $SESSION:3 -n "camera_wrist"
tmux send-keys -t $SESSION:3 "source ~/miniconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t $SESSION:3 "conda activate polymetis" C-m
tmux send-keys -t $SESSION:3 "cd $PROJECT_DIR" C-m
tmux send-keys -t $SESSION:3 "python -u experiments/launch_camera_single.py --hostname $HOST --port $WRIST_PORT --camera-id $WRIST_CAMERA_ID" C-m
sleep 3

# Fenster 4: ENV control loop with non-blocking HPC recording stream
tmux new-window -t $SESSION:4 -n "env"
tmux send-keys -t $SESSION:4 "source ~/miniconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t $SESSION:4 "conda activate polymetis" C-m
tmux send-keys -t $SESSION:4 "cd $PROJECT_DIR" C-m
tmux send-keys -t $SESSION:4 "python experiments/run_env.py --agent gello --hostname $HOST --robot_port $ROBOT_PORT --wrist_camera_port $WRIST_PORT --no-use-base-camera --use-save-interface --save-mode recording_stream --record-stream-host $HPC_RECORD_HOST --record-stream-port $RECORD_STREAM_PORT --record-stream-hwm $RECORD_STREAM_HWM" C-m

tmux select-window -t $SESSION:4
tmux attach-session -t $SESSION
