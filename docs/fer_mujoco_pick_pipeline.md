# FER MuJoCo pick-task pipeline test

This branch now uses the **correct FER/Panda robot simulation** from [`GKnerd/fer_ros2_simulation`](https://github.com/GKnerd/fer_ros2_simulation) instead of a local toy MuJoCo robot.  That upstream project describes itself as a high-fidelity MuJoCo + ROS 2 Jazzy FER simulation and imports the official `franka_description`, `franka_mujoco_sim_bringup`, `mujoco_ros2_control`, `mujoco_vendor`, Cartesian controllers, and MoveIt config through its `.repos` manifest.

The local GELLO pipeline stays the same at the laptop boundary:

- `RobotEnv` still receives a `Robot` implementation.
- Observations still expose `joint_positions`, `joint_velocities`, `ee_pos_quat`, and optionally `wrist_rgb` / `wrist_depth`.
- LeRobot recording still goes through `LeRobotDatasetWriter` or the existing ZMQ recording stream.
- GELLO teleoperation is replaced only for this simulation test by a deterministic FER pick trajectory.

## Required simulator workspace

Prepare the simulator exactly as in the FER repository, or use the helper script added in this branch:

```bash
./setup_fer_mujoco_sim_workspace.sh
cd ~/fer_ros2_simulation
./.docker/build_image.sh
```

Manual equivalent:

```bash
git clone https://github.com/GKnerd/fer_ros2_simulation.git ~/fer_ros2_simulation
cd ~/fer_ros2_simulation
mkdir -p ros2_ws/src
vcs import ros2_ws/src < fer_ros2_mujoco.repos
./.docker/build_image.sh
```

The launcher defaults to:

```bash
FER_WS=$HOME/fer_ros2_simulation/ros2_ws
MUJOCO_DIR=/home/tim/mujoco-3.9.0-linux-x86_64
```

Override either variable if your paths differ.

## Run one direct LeRobot episode

```bash
./start_fer_mujoco_pick_pipeline.sh
```

The launcher starts `franka_mujoco_sim_bringup fer_mujoco_ros2_control.launch.py`, waits for the ROS 2 simulator, and then runs `experiments/run_fer_mujoco_pick_pipeline.py` against `/joint_states`, `/joint_effort_traj_controller/joint_trajectory`, and `/gripper_effort_controller/gripper_cmd`.

## Run through the recording stream

```bash
SAVE_MODE=recording_stream ./start_fer_mujoco_pick_pipeline.sh
```

This starts the same ZMQ LeRobot stream recorder used by the real robot pipeline before the deterministic pick task begins.

## Wrist camera topic

The Python pipeline expects the simulated wrist camera to be mounted in the FER ROS/MuJoCo model and published as ROS image topics:

```bash
WRIST_RGB_TOPIC=/wrist_camera/image_raw
WRIST_DEPTH_TOPIC=/wrist_camera/depth/image_raw
```

If the simulator launch uses different camera topic names, override them when starting the pipeline:

```bash
WRIST_RGB_TOPIC=/my_camera/color/image_raw \
WRIST_DEPTH_TOPIC=/my_camera/depth/image_raw \
./start_fer_mujoco_pick_pipeline.sh
```

For a joint-only dry run while wiring the simulated wrist camera topic, use:

```bash
python -u experiments/run_fer_mujoco_pick_pipeline.py --no-use-wrist-camera --save-mode none
```

## Why this is different from the previous version

The previous branch generated a simplified MJCF robot locally.  This version removes that model and bridges directly into the FER ROS 2 MuJoCo stack, using the joint names and controller interfaces from `franka_mujoco_sim_bringup` (`fer_joint1` ... `fer_joint7`, `fer_finger_joint1`, `joint_effort_traj_controller`, and `gripper_effort_controller`).
