# Polymetis-controlled MuJoCo pick pipeline

This simulation path deliberately controls the robot exactly like the real Panda path in this repository: the pipeline talks to a GELLO ZMQ robot node, that node instantiates `PandaRobot`, and `PandaRobot` uses `polymetis.RobotInterface` / `polymetis.GripperInterface` for every motion command. No ROS 2 controller is used for robot movement.

## Important: use the launcher to start MuJoCo

`experiments/run_polymetis_mujoco_pick_pipeline.py` is only the final pick-policy client. Running that file directly does **not** start MuJoCo, the Polymetis robot server, or the GELLO ZMQ robot node.

Start the complete stack with:

```bash
cd /home/tim/gello_software
./start_polymetis_mujoco_pick_pipeline.sh
```

The launcher opens a tmux session and starts, in order:

1. a MuJoCo-backed Polymetis robot server,
2. this repo's regular ZMQ robot node with `--robot panda --robot-ip 127.0.0.1`,
3. optional wrist-camera ZMQ process,
4. optional LeRobot stream recorder,
5. the deterministic pick-policy client.

## Reset after a failed run

If Polymetis reports that port `50051` is unavailable or the robot context is invalid, stop stale simulator processes first:

```bash
cd /home/tim/gello_software
./stop_polymetis_mujoco_pick_pipeline.sh
```

The start launcher also performs this cleanup automatically by default when `START_POLYMETIS_SIM=1`.

## First movement-only smoke test

If LeRobot is not installed in the `polymetis` conda environment yet, first test only the simulated motion path:

```bash
cd /home/tim/gello_software
SAVE_MODE=none ./start_polymetis_mujoco_pick_pipeline.sh
```

This still starts MuJoCo and controls the simulated Panda through Polymetis, but it does not try to write a LeRobot dataset.

## Recording to LeRobot

After `lerobot` is importable in the same conda environment used for Polymetis, run:

```bash
cd /home/tim/gello_software
SAVE_MODE=lerobot ./start_polymetis_mujoco_pick_pipeline.sh
```

The launcher checks this before starting. If `lerobot` is missing, it exits with an explicit message instead of opening a broken pipeline window.

## Architecture

1. Start a MuJoCo-backed Polymetis robot server.
2. Wait until `polymetis.RobotInterface` can read valid joint positions from the server.
3. Start this repo's regular ZMQ robot node with `--robot panda --robot-ip 127.0.0.1`.
4. Run the deterministic pick task through `RobotEnv`.
5. Record with either `LeRobotDatasetWriter` or the existing ZMQ recording stream.

The deterministic agent only replaces GELLO teleoperation; it still emits the same 8-dimensional Panda command schema used by the real setup: 7 arm joints plus normalized gripper.

## Launcher defaults

```bash
MUJOCO_DIR=/home/tim/mujoco-3.9.0-linux-x86_64
CONDA_ENV=polymetis
POLYMETIS_SIM_CMD="launch_robot.py robot_client=mujoco_sim use_real_time=false gui=true"
POLYMETIS_GRPC_PORT=50051
RESET_STALE_POLYMETIS=1
SAVE_MODE=lerobot
```

`RESET_STALE_POLYMETIS=1` intentionally kills stale local `run_server` / MuJoCo-sim launcher processes before starting a new simulation. This fixes the common Polymetis error:

```text
AssertionError: Port unavailable; possibly another server found on designated address
```

If you intentionally started a valid simulator yourself and do not want the launcher to kill it, use:

```bash
START_POLYMETIS_SIM=0 RESET_STALE_POLYMETIS=0 ./start_polymetis_mujoco_pick_pipeline.sh
```

## If the MuJoCo Hydra config name differs

If your Polymetis installation uses a different Hydra config name for the MuJoCo simulation, override only the simulator command:

```bash
POLYMETIS_SIM_CMD="launch_robot.py robot_client=franka_sim use_real_time=false gui=true" \
./start_polymetis_mujoco_pick_pipeline.sh
```

## Recording-stream mode

```bash
SAVE_MODE=recording_stream ./start_polymetis_mujoco_pick_pipeline.sh
```

This starts `experiments/record_lerobot_stream.py` in a separate tmux window and streams frames through the same recording-publisher path used elsewhere in this repo.

## Manual commands, if you do not want tmux

Use these only for debugging. Terminal 1 must remain running before Terminal 2 starts.

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate polymetis
export MUJOCO_PATH=/home/tim/mujoco-3.9.0-linux-x86_64
export LD_LIBRARY_PATH=$MUJOCO_PATH/lib:${LD_LIBRARY_PATH:-}
pkill -9 run_server 2>/dev/null || true
launch_robot.py robot_client=mujoco_sim use_real_time=false gui=true
```

Terminal 2:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate polymetis
cd /home/tim/gello_software
python -u experiments/launch_nodes.py \
  --robot panda \
  --hostname 127.0.0.1 \
  --robot_port 6001 \
  --robot-ip 127.0.0.1
```

Terminal 3, movement-only:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate polymetis
cd /home/tim/gello_software
python -u experiments/run_polymetis_mujoco_pick_pipeline.py \
  --save-mode none \
  --robot-host 127.0.0.1 \
  --robot-port 6001 \
  --no-use-wrist-camera
```

## Wrist camera

Robot movement is Polymetis-only. For camera recording the pipeline still expects the same ZMQ camera interface as the real setup. If your MuJoCo wrist camera has a ZMQ camera server, start it through the launcher:

```bash
START_WRIST_CAMERA=1 \
WRIST_CAMERA_CMD="python -u path/to/your_mujoco_wrist_camera_zmq_server.py --host 127.0.0.1 --port 5000" \
./start_polymetis_mujoco_pick_pipeline.sh
```

While wiring the camera, the default launcher runs joint-only by passing no camera keys to the LeRobot writer.
