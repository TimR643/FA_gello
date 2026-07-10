#!/usr/bin/env bash
set -euo pipefail

# Laptop-side reverse tunnel over the direct USB/USB-C network link.
# Use this for HPC policy rollouts: the HPC connects to local 127.0.0.1 ports,
# and SSH forwards them over the USB link to the Franka laptop's ZMQ servers.

LAPTOP_USB_IP="${LAPTOP_USB_IP:-10.66.0.1}"
HPC_USB_IP="${HPC_USB_IP:-10.66.0.2}"

export HPC_HOST="${HPC_HOST:-$HPC_USB_IP}"
export LOCAL_ZMQ_HOST="${LOCAL_ZMQ_HOST:-127.0.0.1}"
export REMOTE_BIND_HOST="${REMOTE_BIND_HOST:-127.0.0.1}"

cat <<EOF2
Starting reverse tunnel over direct laptop<->HPC USB link
  Laptop USB IP:     $LAPTOP_USB_IP
  HPC SSH host:      $HPC_HOST
  Local ZMQ host:    $LOCAL_ZMQ_HOST
  Remote bind host:  $REMOTE_BIND_HOST
EOF2

exec "$(dirname "$0")/start_franka_to_hpc_reverse_tunnel.sh"
