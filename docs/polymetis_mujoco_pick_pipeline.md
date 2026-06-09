# Polymetis-controlled MuJoCo pick pipeline

This simulation path deliberately controls the robot exactly like the real Panda path in this repository: the pipeline talks to a GELLO ZMQ robot node, that node instantiates `PandaRobot`, and `PandaRobot` uses `polymetis.RobotInterface` / `polymetis.GripperInterface` for every motion command. No ROS 2 controller is used for robot movement.

## Important: use the launcher to start MuJoCo

`experiments/run_polymetis_mujoco_pick_pipeline.py` is only the final pick-policy client. Running that file directly does **not** start MuJoCo, the Polymetis robot server, or the GELLO ZMQ robot node.

Start the complete stack with:

```bash
cd /home/tim/gello_software
./start_polymetis_mujoco_pick_pipeline.sh
```

The launcher opens a tmux session and starts, in order. If an old `polymetis_mujoco_pick_pipeline` tmux session already exists, the launcher now restarts it by default (`RESTART_EXISTING_SESSION=1`) so repeated smoke-test attempts start cleanly:

1. a visible MuJoCo-backed Polymetis robot server,
2. this repo's regular ZMQ robot node with `--robot panda --robot-ip 127.0.0.1`,
3. a GELLO Panda ZMQ node that defaults to arm-only simulation mode (`PANDA_USE_GRIPPER=0`) because Polymetis `mujoco_sim` usually does not start a separate gripper server,
4. optional wrist-camera ZMQ process,
5. optional LeRobot stream recorder,
6. the deterministic pick-policy client.

## Visible MuJoCo Franka viewer

The launcher defaults to a visible MuJoCo viewer for the simulated Franka:

```bash
MUJOCO_GUI=true
MUJOCO_GL=glfw
POLYMETIS_SIM_CMD="launch_robot.py robot_client=mujoco_sim use_real_time=false gui=true port=$POLYMETIS_GRPC_PORT $POLYMETIS_SIM_METADATA_OVERRIDES"
```

So the normal command should open the MuJoCo window in the `polymetis_sim` tmux pane while the pick client runs in the `pipeline` pane:

```bash
SAVE_MODE=none ./start_polymetis_mujoco_pick_pipeline.sh
```

If you are running over SSH, make sure X forwarding or your local display is available. The launcher warns when `MUJOCO_GUI=true` but `DISPLAY` is missing. If you intentionally need headless mode, run:

```bash
MUJOCO_GUI=false SAVE_MODE=none ./start_polymetis_mujoco_pick_pipeline.sh
```


## Built-in Polymetis simulator versus the FER scene

The command `launch_robot.py robot_client=mujoco_sim ...` starts the simulator shipped by your local Polymetis installation. On your machine its log prints `pybullet build time`, so this built-in simulator is useful for validating the **Polymetis control path**, but it is not necessarily the same table/cube/wrist-camera scene from `GKnerd/fer_ros2_simulation`. That FER repository is ROS2/MuJoCo-oriented; to use its exact assets while still commanding through Polymetis, there must be a Polymetis-compatible simulator server for that scene.

This launcher is now prepared for that: replace only `POLYMETIS_SIM_CMD` with the command that starts your FER-compatible Polymetis server, and keep the same `POLYMETIS_GRPC_PORT` so the GELLO ZMQ node connects to it. Until that server exists, the current `mujoco_sim` path is an arm-control smoke test, not a faithful FER scene reproduction.

## Reset after a failed run

If Polymetis reports that port `50051` is unavailable or the robot context is invalid, stop stale simulator processes first:

```bash
cd /home/tim/gello_software
./stop_polymetis_mujoco_pick_pipeline.sh
```

The start launcher also performs this cleanup automatically by default when `START_POLYMETIS_SIM=1`. If port `50051` is still occupied after cleanup, `AUTO_SELECT_POLYMETIS_PORT=1` makes the launcher choose a free nearby port and passes the same port to both `launch_robot.py` and the GELLO Panda ZMQ node.



## If readiness still times out

When startup fails, the launcher captures the tmux panes before tearing down the session. Look in the printed `LOG_DIR`, which defaults to:

```bash
/tmp/polymetis_mujoco_pick_pipeline_logs
```

The most important file is usually the `polymetis_sim` pane log because it contains the actual `launch_robot.py` / MuJoCo error. The readiness probe only tells us that `RobotInterface` could not read valid metadata yet; the tmux log usually tells us whether the MuJoCo viewer failed, the Polymetis config crashed, or a stale server was still bound. By default `KEEP_TMUX_ON_FAILURE=1`, so the tmux session is also left alive for inspection; attach with `tmux attach -t polymetis_mujoco_pick_pipeline`.

If you want to force a specific clean Polymetis port manually, run for example:

```bash
POLYMETIS_GRPC_PORT=50100 SAVE_MODE=none ./start_polymetis_mujoco_pick_pipeline.sh
```


## Polymetis `default_Kq` / `default_Kx` Hydra error

Some Polymetis installs ship `robot_client=mujoco_sim` with metadata fields that interpolate root-level gain values such as `default_Kq`, `default_Kqd`, `default_Kx`, and `default_Kxd`. If those root keys are missing, `launch_robot.py` fails with:

```text
str interpolation key 'default_Kq' not found
```

The launcher now supplies those gains by default through `POLYMETIS_SIM_METADATA_OVERRIDES` and includes them in the default `launch_robot.py` command. If your local Hydra config already defines these keys and complains about duplicate `+default_*` overrides, disable the injected overrides with:

```bash
POLYMETIS_SIM_METADATA_OVERRIDES= SAVE_MODE=none ./start_polymetis_mujoco_pick_pipeline.sh
```

When you provide a fully custom `POLYMETIS_SIM_CMD`, include the same metadata overrides yourself if your config needs them. The launcher also exports `HYDRA_FULL_ERROR=1` for the simulator window so the captured `polymetis_sim` log contains the full Hydra stack trace.

## Conda MKL `MKL_INTERFACE_LAYER: unbound variable`

The launcher uses `set -u` for safer shell scripting, but some conda activation hooks read unset variables. The script now temporarily disables `nounset` only around `conda activate`, and it activates conda before any Python-based port checks, so this error and `python: command not found` should no longer stop startup:

```text
libblas_mkl_activate.sh: line 1: MKL_INTERFACE_LAYER: unbound variable
```

You do not need to pre-activate the environment manually; run the launcher directly from the repo.

## First movement-only smoke test

If LeRobot is not installed in the `polymetis` conda environment yet, first test only the simulated motion path:

```bash
cd /home/tim/gello_software
SAVE_MODE=none ./start_polymetis_mujoco_pick_pipeline.sh
```

This still starts MuJoCo and controls the simulated Panda through Polymetis, but it does not try to write a LeRobot dataset.


## Polymetis gripper server errors in simulation

The built-in Polymetis simulator often exposes only the arm server on `50051`; no gripper server is available on the separate default gripper endpoint. Therefore the launcher starts the Panda ZMQ node with:

```bash
PANDA_USE_GRIPPER=0
PANDA_INITIALIZE_ROBOT=0
PANDA_MANUAL_GRIPPER_OVERRIDE=0
```

The pipeline still keeps the same 8-value action/observation schema by storing a scalar simulated gripper value locally. If you later run against hardware or a simulator that really provides a Polymetis gripper server, set `PANDA_USE_GRIPPER=1`.

The launcher also waits until the ZMQ robot port is reachable before starting the pick client, so the client should no longer immediately fail with `ZMQ timeout - robot may be disconnected` just because the Panda node is still initializing.

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
POLYMETIS_SIM_CMD="launch_robot.py robot_client=mujoco_sim use_real_time=false gui=true port=$POLYMETIS_GRPC_PORT $POLYMETIS_SIM_METADATA_OVERRIDES"
MUJOCO_GUI=true
MUJOCO_GL=glfw
POLYMETIS_SIM_METADATA_OVERRIDES="'+default_Kq=[150,150,150,150,150,150,150]' '+default_Kqd=[10,10,10,10,10,10,10]' '+default_Kx=[50,50,50,50,50,50]' '+default_Kxd=[10,10,10,10,10,10]'"
POLYMETIS_GRPC_PORT=50051
AUTO_SELECT_POLYMETIS_PORT=1
PANDA_USE_GRIPPER=0
PANDA_INITIALIZE_ROBOT=0
PANDA_MANUAL_GRIPPER_OVERRIDE=0
ZMQ_READY_TIMEOUT=20
RESTART_EXISTING_SESSION=1
KEEP_TMUX_ON_FAILURE=1
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
POLYMETIS_SIM_CMD="launch_robot.py robot_client=franka_sim use_real_time=false gui=true port=$POLYMETIS_GRPC_PORT $POLYMETIS_SIM_METADATA_OVERRIDES" \
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
export MUJOCO_GL=glfw
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
  --robot-ip 127.0.0.1 \
  --polymetis-port 50051
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
