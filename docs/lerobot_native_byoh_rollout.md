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
DURATION=30 \
MAX_JOINT_DELTA=0.01 \
./start_lerobot_native_real_policy.sh
```

The script installs both this repository and `lerobot_robot_gello` in editable
mode, then calls `lerobot-rollout --strategy.type=base --robot.type=gello_zmq --policy.path=...`.

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
