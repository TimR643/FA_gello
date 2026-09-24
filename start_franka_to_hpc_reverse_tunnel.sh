#!/usr/bin/env bash
set -euo pipefail

# Run this on the Franka laptop after the GELLO robot/camera ZMQ servers are
# running. It opens reverse SSH forwards on the HPC so lerobot-rollout running on
# the HPC can reach the Franka laptop's local GELLO ZMQ servers through
# 127.0.0.1 on the HPC.

HPC_USER="${HPC_USER:-tim_st179133}"
HPC_HOST="${HPC_HOST:-172.16.0.11}"
LOCAL_ZMQ_HOST="${LOCAL_ZMQ_HOST:-127.0.0.1}"
REMOTE_BIND_HOST="${REMOTE_BIND_HOST:-127.0.0.1}"
ROBOT_PORT="${ROBOT_PORT:-6001}"
WRIST_CAMERA_PORT="${WRIST_CAMERA_PORT:-5000}"
BASE_CAMERA_PORT="${BASE_CAMERA_PORT:-5001}"
SKIP_LOCAL_PORT_CHECK="${SKIP_LOCAL_PORT_CHECK:-0}"

check_local_port() {
  local name="$1"
  local port="$2"
  if ! timeout 2 bash -c "</dev/tcp/${LOCAL_ZMQ_HOST}/${port}" 2>/dev/null; then
    cat >&2 <<MSG
FEHLT: ${name} is not reachable on ${LOCAL_ZMQ_HOST}:${port} from this Franka laptop.
Start the GELLO ZMQ server first, or set LOCAL_ZMQ_HOST to the address where it
is bound. Quick check: ss -ltnp | grep -E ':(${ROBOT_PORT}|${WRIST_CAMERA_PORT}|${BASE_CAMERA_PORT})'
MSG
    exit 1
  fi
}

if [ "$SKIP_LOCAL_PORT_CHECK" != "1" ]; then
  check_local_port "robot ZMQ server" "$ROBOT_PORT"
  check_local_port "wrist camera ZMQ server" "$WRIST_CAMERA_PORT"
  check_local_port "base camera ZMQ server" "$BASE_CAMERA_PORT"
fi

ssh -N \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=30 \
  -o ServerAliveCountMax=3 \
  -R "${REMOTE_BIND_HOST}:${ROBOT_PORT}:${LOCAL_ZMQ_HOST}:${ROBOT_PORT}" \
  -R "${REMOTE_BIND_HOST}:${WRIST_CAMERA_PORT}:${LOCAL_ZMQ_HOST}:${WRIST_CAMERA_PORT}" \
  -R "${REMOTE_BIND_HOST}:${BASE_CAMERA_PORT}:${LOCAL_ZMQ_HOST}:${BASE_CAMERA_PORT}" \
  "${HPC_USER}@${HPC_HOST}"
