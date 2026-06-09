# Polymetis-controlled MuJoCo pick pipeline

This simulation path deliberately controls the robot exactly like the real Panda path in this repository: the pipeline talks to a GELLO ZMQ robot node, that node instantiates `PandaRobot`, and `PandaRobot` uses `polymetis.RobotInterface` / `polymetis.GripperInterface` for every motion command.  No ROS 2 controller is used for robot movement.

## Architecture

1. Start a MuJoCo-backed Polymetis robot server.
2. Start this repo's regular ZMQ robot node with `--robot panda --robot-ip 127.0.0.1`.
3. Run the deterministic pick task through `RobotEnv`.
4. Record with either `LeRobotDatasetWriter` or the existing ZMQ recording stream.

The deterministic agent only replaces GELLO teleoperation; it still emits the same 8-dimensional Panda command schema used by the real setup: 7 arm joints plus normalized gripper.

## Start everything in tmux

```bash
cd /workspace/FA_gello
./start_polymetis_mujoco_pick_pipeline.sh
```

The launcher defaults to:

```bash
MUJOCO_DIR=/home/tim/mujoco-3.9.0-linux-x86_64
CONDA_ENV=polymetis
POLYMETIS_SIM_CMD="launch_robot.py robot_client=mujoco_sim use_real_time=false gui=true"
SAVE_MODE=lerobot
```

If your Polymetis installation uses a different Hydra config name for the MuJoCo simulation, override only the simulator command:

```bash
POLYMETIS_SIM_CMD="launch_robot.py robot_client=franka_sim use_real_time=false gui=true" \
./start_polymetis_mujoco_pick_pipeline.sh
```

## Recording-stream mode

```bash
SAVE_MODE=recording_stream ./start_polymetis_mujoco_pick_pipeline.sh
```

## Running against an already started Polymetis simulator

If the MuJoCo-backed Polymetis server is already running on localhost, skip starting it:

```bash
START_POLYMETIS_SIM=0 ./start_polymetis_mujoco_pick_pipeline.sh
```

## Wrist camera

Robot movement is Polymetis-only.  For camera recording the pipeline still expects the same ZMQ camera interface as the real setup.  If your MuJoCo wrist camera has a ZMQ camera server, start it through the launcher:

```bash
START_WRIST_CAMERA=1 \
WRIST_CAMERA_CMD="python -u path/to/your_mujoco_wrist_camera_zmq_server.py --host 127.0.0.1 --port 5000" \
./start_polymetis_mujoco_pick_pipeline.sh
```

While wiring the camera, the default launcher runs joint-only by passing no camera keys to the LeRobot writer.
