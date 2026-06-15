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
