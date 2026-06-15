#!/usr/bin/env bash
set -euo pipefail

# Run this on the Franka laptop. It opens reverse SSH forwards on the HPC so
# lerobot-rollout running on the HPC can reach the Franka laptop's local GELLO
# ZMQ robot/camera servers through 127.0.0.1.

HPC_USER="${HPC_USER:-tim_st179133}"
HPC_HOST="${HPC_HOST:-172.16.0.11}"
ROBOT_PORT="${ROBOT_PORT:-6001}"
WRIST_CAMERA_PORT="${WRIST_CAMERA_PORT:-5000}"
BASE_CAMERA_PORT="${BASE_CAMERA_PORT:-5001}"

ssh -N \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=30 \
  -o ServerAliveCountMax=3 \
  -R "127.0.0.1:${ROBOT_PORT}:127.0.0.1:${ROBOT_PORT}" \
  -R "127.0.0.1:${WRIST_CAMERA_PORT}:127.0.0.1:${WRIST_CAMERA_PORT}" \
  -R "127.0.0.1:${BASE_CAMERA_PORT}:127.0.0.1:${BASE_CAMERA_PORT}" \
  "${HPC_USER}@${HPC_HOST}"
