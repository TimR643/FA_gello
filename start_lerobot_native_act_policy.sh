#!/usr/bin/env bash
set -euo pipefail

# Convenience launcher for ACT checkpoints trained with the legacy GELLO/LeRobot
# feature names.  The ACT checkpoints reported by Tim expect the legacy visual
# features `observation.images.wrist`/`observation.images.base`, so
# this wrapper overrides the generic SmolVLA camera defaults before delegating to
# the shared native rollout script. If only one live wrist camera is available,
# the GELLO LeRobot plugin supplies a zero-filled base image fallback.

export CKPT="${CKPT:-$HOME/lerobot_outputs/train/act_left_green_right_red_two_cams/checkpoints/last/pretrained_model}"
export CAMERA_NAMES="${CAMERA_NAMES:-wrist}"
export POLICY_CAMERA_NAMES="${POLICY_CAMERA_NAMES:-wrist,base}"
export FPS="${FPS:-8}"
export MAX_JOINT_DELTA="${MAX_JOINT_DELTA:-0.01}"
export MAX_GRIPPER_DELTA="${MAX_GRIPPER_DELTA:-0.02}"
export COMMAND_SMOOTHING_ALPHA="${COMMAND_SMOOTHING_ALPHA:-0.5}"
export RETURN_TO_INITIAL_POSITION="${RETURN_TO_INITIAL_POSITION:-false}"

exec "$(dirname "$0")/start_lerobot_native_real_policy.sh"
