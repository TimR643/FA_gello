# MuJoCo pick-task pipeline test

This branch adds a local MuJoCo simulation path for testing the same laptop-side recording pipeline used with the real Franka/GELLO setup.  GELLO teleoperation is replaced by a deterministic pick trajectory that moves a red cube from a table, while observations keep the real deployment schema:

- `joint_positions`: 8 values (`panda_joint1` ... `panda_joint7`, `gripper`)
- `joint_velocities`: 8 values
- `ee_pos_quat`: 7 values
- `wrist_rgb`: RGB image, default `480x640x3`
- `wrist_depth`: depth image, default `480x640x1`

The simulated wrist camera is mounted on the Franka-style wrist body in the MuJoCo XML and is exposed as the same `wrist` camera key used by the LeRobot writer.

## MuJoCo location

The launcher assumes the MuJoCo archive is available at:

```bash
/home/tim/mujoco-3.9.0-linux-x86_64/
```

It exports both `MUJOCO_PATH` and `LD_LIBRARY_PATH` before starting the pipeline.  Override the path if needed:

```bash
MUJOCO_DIR=/other/mujoco/path ./start_mujoco_pick_pipeline.sh
```

## Direct LeRobot recording

Run one deterministic simulated episode and write it directly as a LeRobot video dataset:

```bash
./start_mujoco_pick_pipeline.sh
```

Equivalent command without tmux:

```bash
export MUJOCO_PATH=/home/tim/mujoco-3.9.0-linux-x86_64
export LD_LIBRARY_PATH=$MUJOCO_PATH/lib:${LD_LIBRARY_PATH:-}
python -u experiments/run_mujoco_pick_pipeline.py \
  --save-mode lerobot \
  --lerobot-root ~/lerobot_data/mujoco_panda_pick \
  --lerobot-repo-id local/mujoco_panda_pick_wrist
```

## Recording-stream mode

To exercise the same ZMQ recording-stream path as the real robot setup, start the recorder first in one terminal:

```bash
python -u experiments/record_lerobot_stream.py \
  --port 7000 \
  --lerobot-root ~/lerobot_data/mujoco_panda_pick_stream \
  --lerobot-repo-id local/mujoco_panda_pick_wrist_stream \
  --lerobot-task "Pick the red cube from the table in MuJoCo."
```

Then run the simulation in another terminal:

```bash
python -u experiments/run_mujoco_pick_pipeline.py \
  --save-mode recording_stream \
  --record-stream-port 7000
```

Or let the launcher create the tmux windows:

```bash
SAVE_MODE=recording_stream ./start_mujoco_pick_pipeline.sh
```

## Notes and limits

- The MuJoCo model is deliberately self-contained so no `mujoco_menagerie` checkout is required.
- The arm geometry is Franka-style and uses Panda joint limits, but it is a deterministic pipeline simulator rather than a dynamics-accurate Panda digital twin.
- A copy of the generated XML is written to `mujoco_panda_pick_scene.xml` by default for inspection and reproducibility.
