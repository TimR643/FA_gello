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
`observation.state` / `action`.  For the SmolVLA two-camera checkpoint, the
plugin maps live `wrist,base` cameras to policy-facing `camera1,camera2` and
adds a black dummy `camera3` by default because the policy expects three visual
features.


## Smoothness knobs

If the policy chooses the right behavior but the physical motion is still too
jerky, reduce the per-step deltas first (`MAX_JOINT_DELTA`, `MAX_GRIPPER_DELTA`).
For additional low-pass smoothing, set `COMMAND_SMOOTHING_ALPHA` below `1.0`.
`1.0` preserves the raw clipped target, while values such as `0.3` or `0.5` blend
the new safe target with the previous command before sending it to ZMQ. The
smoothed command is clipped again against the same per-step safety limits before
it reaches the robot.


## Rollout FPS and shutdown reset

LeRobot's rollout runtime defaults to 30 FPS. If the live loop is only reaching
about 8 Hz, pass `FPS=8` (or the measured stable rate) so LeRobot does not try to
command faster than cameras/inference/ZMQ can run. The launcher now defaults to
`FPS=8` for this setup.

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
`CKPT`, `CAMERA_NAMES=wrist`, `POLICY_CAMERA_NAMES=wrist`, `FPS=8`,
conservative ACT smoothing defaults, and `RETURN_TO_INITIAL_POSITION=false`.
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
