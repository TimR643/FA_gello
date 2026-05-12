#!/bin/bash

SESSION="gello_panda"

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

# Fenster 2: Nodes
tmux new-window -t $SESSION:2 -n "nodes"
tmux send-keys -t $SESSION:2 "source ~/miniconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t $SESSION:2 "conda activate polymetis" C-m
tmux send-keys -t $SESSION:2 "cd ~/gello_software" C-m
tmux send-keys -t $SESSION:2 "python experiments/launch_nodes.py --robot panda --robot-ip 127.0.0.1" C-m

sleep 3

# Fenster 3: Camera nodes
tmux new-window -t $SESSION:3 -n "cameras"
tmux send-keys -t $SESSION:3 "source ~/miniconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t $SESSION:3 "conda activate polymetis" C-m
tmux send-keys -t $SESSION:3 "cd ~/gello_software" C-m
tmux send-keys -t $SESSION:3 "pkill -f rsviewer || true" C-m
tmux send-keys -t $SESSION:3 "echo Starting Intel RealSense with rsviewer" C-m
tmux send-keys -t $SESSION:3 "rsviewer >/tmp/rsviewer_gello.log 2>&1 &" C-m
tmux send-keys -t $SESSION:3 "sleep 4" C-m
tmux send-keys -t $SESSION:3 "pkill -f rsviewer || true" C-m
tmux send-keys -t $SESSION:3 "sleep 1" C-m
tmux send-keys -t $SESSION:3 "python experiments/launch_camera_nodes.py --hostname 127.0.0.1" C-m

sleep 2

# Fenster 4: Env nur vorbereiten
tmux new-window -t $SESSION:4 -n "env"
tmux send-keys -t $SESSION:4 "source ~/miniconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t $SESSION:4 "conda activate polymetis" C-m
tmux send-keys -t $SESSION:4 "cd ~/gello_software" C-m
tmux send-keys -t $SESSION:4 "echo 'Run with camera recording:'" C-m
tmux send-keys -t $SESSION:4 "echo 'python experiments/run_env.py --agent=gello --use-save-interface --use-wrist-camera --wrist-camera-port 5000'" C-m

tmux select-window -t $SESSION:4
tmux attach-session -t $SESSION

