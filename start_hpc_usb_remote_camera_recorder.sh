#!/bin/bash
set -euo pipefail

# HPC-side wrapper for recording over a direct USB/USB-C network link to the
# Franka laptop. The HPC pulls camera frames from the laptop's USB-link IP
# instead of sharing the camera switch.

LAPTOP_USB_IP="${LAPTOP_USB_IP:-10.66.0.1}"
HPC_USB_IP="${HPC_USB_IP:-10.66.0.2}"

export HPC_CAMERA_HOST="${HPC_CAMERA_HOST:-$LAPTOP_USB_IP}"
export BIND_HOSTNAME="${BIND_HOSTNAME:-0.0.0.0}"

cat <<EOF2
Starting HPC recorder for direct laptop<->HPC USB link
  Laptop USB IP / camera host:  $HPC_CAMERA_HOST
  HPC USB IP:                  $HPC_USB_IP
  Recorder bind host:           $BIND_HOSTNAME
EOF2

exec "$(dirname "$0")/start_hpc_remote_camera_recorder.sh"
