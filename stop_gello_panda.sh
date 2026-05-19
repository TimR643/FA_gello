#!/bin/bash

tmux kill-session -t gello_panda 2>/dev/null

source ~/miniconda3/etc/profile.d/conda.sh
conda activate polymetis

sudo pkill -9 franka_panda_client
sudo pkill -9 run_server
sudo pkill -9 launch_gripper.py
sudo pkill -9 launch_robot.py
sudo pkill -9 -f "python experiments/launch_nodes.py"
sudo pkill -9 -f "python experiments/run_env.py"

# Camera server
sudo pkill -9 -f "python -u experiments/launch_camera_single.py"
sudo pkill -9 -f "python experiments/launch_camera_single.py"
sudo pkill -9 -f "python -u experiments/launch_two_camera_nodes.py"
sudo pkill -9 -f "python experiments/launch_two_camera_nodes.py"
sudo pkill -9 -f "python experiments/launch_camera_nodes.py"

# Optional policy scripts
sudo pkill -9 -f "test_policy_live_no_motion.py"
sudo pkill -9 -f "run_policy_safe_rollout.py"