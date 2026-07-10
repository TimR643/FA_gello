#!/bin/bash
set -euo pipefail

# Laptop-side wrapper for the direct USB/USB-C link workflow.
# Use this when the HPC is connected directly to the Franka laptop via a
# point-to-point USB network interface, while cameras stay on the camera/laptop
# network. This keeps HPC traffic off the camera switch.

LAPTOP_USB_IP="${LAPTOP_USB_IP:-10.66.0.1}"
HPC_USB_IP="${HPC_USB_IP:-10.66.0.2}"

export HPC_RECORD_HOST="${HPC_RECORD_HOST:-$HPC_USB_IP}"
export CAMERA_BIND_HOST="${CAMERA_BIND_HOST:-0.0.0.0}"

cat <<EOF2
Starting GELLO for direct laptop<->HPC USB link
  Laptop USB IP:     $LAPTOP_USB_IP
  HPC USB IP:        $HPC_USB_IP
  HPC_RECORD_HOST:   $HPC_RECORD_HOST
  CAMERA_BIND_HOST:  $CAMERA_BIND_HOST

Expected network layout:
  cameras <-> Franka laptop on the camera network/switch
  HPC     <-> Franka laptop on the direct USB network only
EOF2

exec "$(dirname "$0")/start_gello_panda.sh"
