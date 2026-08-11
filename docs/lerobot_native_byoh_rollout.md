# Native LeRobot BYOH rollout for GELLO/ZMQ Panda

This branch adds a LeRobot Bring-Your-Own-Hardware robot plugin named
`gello_zmq`.  The goal is to stop bypassing LeRobot's runtime path with a custom
policy loop and instead let the official `lerobot-record` command load the
policy, run the robot loop, and record evaluation episodes.

## Why this path

LeRobot's BYOH workflow expects an installable package whose name starts with
`lerobot_robot_`, a `RobotConfig` subclass registered with a type name, and a
matching `Robot` implementation.  The new package `lerobot_robot_gello` follows
that convention and wraps the existing GELLO ZMQ robot and camera servers.

## Run it

1. Start the usual GELLO/Polymetis robot server and camera ZMQ servers on the
   robot laptop.
2. On the machine that runs LeRobot inference, run:

```bash
./start_lerobot_native_real_policy.sh
```

The policy launcher uses `lerobot-rollout`, not `lerobot-record`, because
LeRobot 0.5.x exposes dataset-recording arguments under `--dataset.*` and does
not accept `--policy.path` on `lerobot-record`.

Useful overrides:

```bash
CKPT=/path/to/pretrained_model \
ROBOT_HOST=127.0.0.1 \
CAMERA_NAMES="wrist,base" \
DURATION=30 \
MAX_JOINT_DELTA=0.01 \
ZMQ_TIMEOUT_MS=3000 \
./start_lerobot_native_real_policy.sh
```

The script installs both this repository and `lerobot_robot_gello` in editable
mode, then calls `lerobot-rollout --strategy.type=base --robot.type=gello_zmq --policy.path=...`.
Use `CAMERA_NAMES`, not `--robot.cameras`: LeRobot's base `RobotConfig` already
uses `cameras` for its own camera-config dictionary, so the GELLO plugin keeps
the simple wrist/base selection in `camera_names` as a comma-separated string such as `wrist,base`.

## H5 logging during teleoperation recording

The H5 logger is part of the `gello_zmq` robot plugin. It is therefore active
for policy rollouts **and** for the official LeRobot recording loop with a
teleoperation arm; no temporary CSV conversion or policy checkpoint is needed.
Add these robot arguments to the `lerobot-record` command you already use:

```bash
lerobot-record \
  --robot.type=gello_zmq \
  --robot.robot_host=127.0.0.1 \
  --robot.h5_log_enabled=true \
  --robot.h5_log_path="$HOME/lerobot_recordings/h5/teleop_$(date +%Y%m%d_%H%M%S).h5" \
  --robot.h5_flush_every=1 \
  --teleop.type=<DEIN_TELEOPERATOR_TYP> \
  --dataset.repo_id=<DEINE_REPO_ID> \
  --dataset.single_task="<DEINE_AUFGABE>"
```

Keep the remaining robot, camera, teleoperator, and dataset arguments from the
working recording command. `h5_flush_every=1` makes every sample durable
immediately; a larger value reduces disk overhead. The file contains
`/time`, measured `/state/q`, `/state/dq`, `/state/tau`, as well as the raw
teleoperator command in `/action/raw` and the safety-limited command actually
sent to the Panda in `/action/sent`. Observation and action rows share their
index; an interrupted cycle retains `NaN` in its missing action row.

For native policy rollouts, `./start_lerobot_native_real_policy.sh --h5-log`
now enables this same direct logger and creates one H5 file per episode.

### Remote-camera recorder on the HPC

For the existing remote-camera workflow, no `lerobot-record` command is
required. H5 logging is enabled by default in the HPC launcher, so start it as
before:

```bash
HPC_CAMERA_HOST=<LAPTOP_IP> ./start_hpc_remote_camera_recorder.sh
```

Every press of `S` in the laptop control window opens a new H5 episode. After
`Q`, RIGHT keeps both the LeRobot episode and its H5 file, while LEFT discards
both. By default, H5 files are written to `${LEROBOT_ROOT}_h5`. The location and
filename prefix can be changed when starting the HPC recorder:

```bash
HPC_CAMERA_HOST=<LAPTOP_IP> \
H5_LOG_DIR="$HOME/lerobot_h5/precision_peg" \
H5_LOG_BASENAME="precision_peg" \
./start_hpc_remote_camera_recorder.sh
```

Set `H5_LOG_ENABLED=false` only when H5 output is not wanted. The laptop stream
also forwards measured joint torques when the robot provides them; missing
torques are stored as `NaN` rather than substituted with another signal.

## Safety defaults

`gello_zmq` still applies the same last-mile safety limiter before commands hit
ZMQ:

- absolute Panda joint limits,
- `MAX_JOINT_DELTA` per control step for arm joints,
- `MAX_GRIPPER_DELTA` per control step for the gripper,
- configurable `ACTION_MODE` (`absolute_joint_position` by default).

If the old custom rollout loop was causing the chaotic behavior, this native path
removes that loop from the critical path while preserving conservative hardware
clipping.


## ZMQ/SSH tunnel check

`Robot connected: gello_zmq` only means the local ZMQ client object was created.
The plugin now performs a preflight `num_dofs()` request with `ZMQ_TIMEOUT_MS` so
missing SSH tunnels or stopped ZMQ servers fail quickly instead of hanging on the
first observation. If this fails, verify that the robot laptop forwards port 6001
and the camera ports 5000/5001 to the machine running `lerobot-rollout`.

## Reverse tunnel workflow from the Franka laptop

If you start the tunnel from the Franka laptop, keep using reverse SSH forwards
(`-R`).  This exposes the Franka laptop's local ZMQ servers on the HPC loopback
interface, so the rollout on the HPC must keep `ROBOT_HOST=127.0.0.1`.

Run this on the Franka laptop:

```bash
./start_franka_to_hpc_reverse_tunnel.sh
```

Equivalent manual command:

```bash
ssh -N \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=30 \
  -o ServerAliveCountMax=3 \
  -R 127.0.0.1:6001:127.0.0.1:6001 \
  -R 127.0.0.1:5000:127.0.0.1:5000 \
  -R 127.0.0.1:5001:127.0.0.1:5001 \
  tim_st179133@172.16.0.11
```

Then, in a second shell on the HPC, run the LeRobot rollout with
`ROBOT_HOST=127.0.0.1`. The reverse tunnel must stay open for the entire rollout.

If the Franka laptop prints `connect_to 127.0.0.1 port 6001: failed` for the
reverse tunnel, the HPC did reach the reverse-forwarded port, but the SSH client
on the Franka laptop could not connect to the local target. In practice this
means the local GELLO ZMQ servers are not listening yet, are listening on a
different host/interface, or the ports differ. Run the tunnel script only after
`start_gello_panda.sh` has brought up the robot/camera ZMQ tmux windows. You can
check locally on the Franka laptop with:

```bash
ss -ltnp | grep -E ':(6001|5000|5001)'
```

If the servers are bound to the laptop LAN IP instead of loopback, keep the HPC
rollout at `ROBOT_HOST=127.0.0.1` but start the reverse tunnel with e.g.:

```bash
LOCAL_ZMQ_HOST=<FRANKA_LAPTOP_IP> ./start_franka_to_hpc_reverse_tunnel.sh
```


## Camera image size convention

The GELLO RealSense camera driver passes `img_size` to OpenCV as `(width, height)`.
The LeRobot feature schema is still declared as image shape `(height, width, 3)`.
Therefore the plugin requests `(image_width, image_height)` from ZMQ and validates
that the returned RGB image is `(image_height, image_width, 3)`.


## Policy-facing feature names

LeRobot robot hardware features should be raw hardware names.  The rollout code
turns camera feature keys such as `camera1` into dataset keys such as
`observation.images.camera1`, and motor keys ending in `.pos` into
`observation.state` / `action`.  For the current training setup, the first policy
camera was the wrist camera, the second policy camera was the base camera, and
only the third policy camera was empty.  The native GELLO rollout defaults mirror
that convention: `CAMERA_NAMES=wrist,base` and
`POLICY_CAMERA_NAMES=camera1,camera2,camera3`, so `camera1=wrist`,
`camera2=base`, and `camera3` is padded with a black image.


## Smoothness knobs

The rollout path no longer applies an additional low-pass smoothing alpha. The
policy target is only passed through the safety executor, which clips each step
with `MAX_JOINT_DELTA` and `MAX_GRIPPER_DELTA` before the command reaches ZMQ. If
the physical motion is too jerky, reduce those per-step deltas or lower `FPS`; if
the robot does not move far enough toward the table, increase the deltas
carefully rather than reintroducing smoothing.


## Rollout FPS and shutdown reset

LeRobot's rollout runtime defaults to 30 FPS. The observed live loop often runs
around 7.8-8.0 Hz with occasional lower spikes, so targeting exactly `FPS=8` can
trigger continuous "loop is running slower" warnings. The launcher now defaults
to `FPS=7` to leave timing headroom; override `FPS` only after measuring that
cameras, inference, and ZMQ can sustain the requested rate.

LeRobot also defaults to returning the robot to its initial position on shutdown.
For the Panda state vector this includes the gripper joint, so the final reset can
look like the gripper closes once at the end. The launcher now defaults to
`RETURN_TO_INITIAL_POSITION=false` to leave the robot in the final rollout pose.
Set it to `true` only if you explicitly want LeRobot's automatic shutdown reset.


## ACT checkpoint launcher

The SmolVLA checkpoint and the older ACT checkpoint were trained with different
policy-facing camera names.  SmolVLA currently uses the generic launcher default
`POLICY_CAMERA_NAMES=camera1,camera2,camera3`, while the ACT checkpoint in
`~/lerobot_outputs/train/act_left_green_right_red_two_cams/checkpoints/last/pretrained_model`
expects `observation.images.wrist`.  Use the dedicated ACT wrapper so the robot
advertises the same visual feature name that the ACT policy expects:

```bash
./start_lerobot_native_act_policy.sh
```

The wrapper delegates to `start_lerobot_native_real_policy.sh` after setting
`CKPT`, `CAMERA_NAMES=wrist`, `POLICY_CAMERA_NAMES=wrist`, `FPS=7`,
conservative ACT per-step delta defaults, and `RETURN_TO_INITIAL_POSITION=false`.
Override those environment variables before the command if a different ACT checkpoint expects a
different camera schema.

## Checkpoint path layout

`CKPT` must resolve to a LeRobot `pretrained_model` directory that contains
`config.json`.  Valid examples are:

```bash
CKPT=/path/to/checkpoints/last/pretrained_model ./start_lerobot_native_act_policy.sh
CKPT=/path/to/checkpoints/100000/pretrained_model ./start_lerobot_native_act_policy.sh
```

For convenience, the launcher also accepts the parent step directory when it
contains `pretrained_model/config.json`, for example
`CKPT=/path/to/checkpoints/100000`.  A path such as
`.../checkpoints/last/100000` is not a LeRobot pretrained model directory unless
it contains its own `config.json`, so the script now fails before reinstalling the
packages and prints the expected layout.

## Checking the recording start pose

Use `check_gello_start_position.sh` before recording to verify that the live
Panda/GELLO ZMQ state is close to the desired start pose.  By default it now
checks against Tim's preferred start pose:
`5.185,0,0,-126.8,121.58,-54.39,50.81` degrees plus normalized gripper value
`1.0`:

```bash
./check_gello_start_position.sh
```

The command exits with status `0` when every joint is within tolerance and `1`
otherwise, so it can be used as a pre-recording guard.  To watch while manually
moving GELLO into place, run the slow watch mode:

```bash
./check_gello_start_position.sh --watch --period-s 5
```

Do not leave `--watch` running during recording or policy rollout: it queries the
same ZMQ robot server and can add enough load to disturb the real-time arm loop.
Use it only before recording, then stop it with Ctrl-C.

If the checker prints a ZMQ timeout, it means the robot ZMQ server did not answer
within `ZMQ_TIMEOUT_MS` (default 3000 ms).  First make sure you are not running
the checker during recording/rollout, then verify the robot server/tunnel.  If the
server is alive but slow, retry once with `ZMQ_TIMEOUT_MS=10000`.

Useful overrides:

```bash
START_JOINTS_DEG="5.185,0,0,-126.8,121.58,-54.39,50.81" \
START_GRIPPER=1.0 \
START_ARM_TOLERANCE_RAD=0.035 \
START_GRIPPER_TOLERANCE=0.08 \
./check_gello_start_position.sh
```

To convert between radians and degrees, use `degrees = radians * 180 / pi` and
`radians = degrees * pi / 180`.  The default arm pose above is approximately
`0.0905,0,0,-2.2131,2.1220,-0.9493,0.8868` radians.

## Low-rate wrist camera preview

You can preview the wrist camera through the existing ZMQ camera server with:

```bash
./view_wrist_camera.sh
```

The preview is intentionally low-rate (`PREVIEW_FPS=2` by default) because it is
an additional ZMQ camera client.  The camera server serves requests serially, so
leaving the preview open during recording or policy rollout can reduce camera FPS
or disturb timing.  Use it for setup/alignment, then close it before starting the
real recording/rollout.  If you need a different rate or port:

```bash
PREVIEW_FPS=1 WRIST_CAMERA_PORT=5000 ./view_wrist_camera.sh
```

## Wrist camera alignment after a bump

If the physical wrist camera was bumped, align it against a known-good LeRobot
recording frame instead of eyeballing the robot pose alone:

```bash
DATASET_ROOT=/path/to/lerobot_dataset \
REFERENCE_EPISODE_INDEX=0 \
REFERENCE_FRAME_INDEX=0 \
./align_wrist_camera_to_lerobot_frame.sh
```

The window shows the recorded reference frame, the live wrist image, an overlay,
and an absolute-difference view. Move the camera until the live image matches the
reference; stop this preview before recording or policy rollout.
