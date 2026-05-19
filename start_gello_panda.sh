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

# Fenster 3: Wrist Camera / FRAMOS D435e
tmux new-window -t $SESSION:3 -n "camera_wrist"
tmux send-keys -t $SESSION:3 "source ~/miniconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t $SESSION:3 "conda activate polymetis" C-m
tmux send-keys -t $SESSION:3 "cd ~/gello_software" C-m
tmux send-keys -t $SESSION:3 "python -u experiments/launch_camera_single.py --hostname 127.0.0.1 --port 5000 --camera-id 6CD1460304A5" C-m

sleep 5

# Fenster 4: Base Camera / Intel RealSense D455
tmux new-window -t $SESSION:4 -n "camera_base"
tmux send-keys -t $SESSION:4 "source ~/miniconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t $SESSION:4 "conda activate polymetis" C-m
tmux send-keys -t $SESSION:4 "cd ~/gello_software" C-m
tmux send-keys -t $SESSION:4 "python -u experiments/launch_camera_single.py --hostname 127.0.0.1 --port 5001 --camera-id 234222303420" C-m

sleep 3

# Fenster 5: Env nur vorbereiten
tmux new-window -t $SESSION:5 -n "env"
tmux send-keys -t $SESSION:5 "source ~/miniconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t $SESSION:5 "conda activate polymetis" C-m
tmux send-keys -t $SESSION:5 "cd ~/gello_software" C-m
tmux send-keys -t $SESSION:5 "python experiments/run_env.py --agent gello --hostname 127.0.0.1 --wrist-camera-port 5000 --base-camera-port 5001 --use-save-interface"

tmux select-window -t $SESSION:5
tmux attach-session -t $SESSION