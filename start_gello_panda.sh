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

# Fenster 3: Env nur vorbereiten
tmux new-window -t $SESSION:3 -n "env"
tmux send-keys -t $SESSION:3 "source ~/miniconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t $SESSION:3 "conda activate polymetis" C-m
tmux send-keys -t $SESSION:3 "cd ~/gello_software" C-m

tmux select-window -t $SESSION:3
tmux attach-session -t $SESSION