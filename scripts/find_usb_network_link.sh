#!/usr/bin/env bash
set -euo pipefail

# Print likely USB/USB-C network interfaces and their IPv4 addresses.
# This is read-only: it does not configure network interfaces.

if ! command -v ip >/dev/null 2>&1; then
  echo "ERROR: the 'ip' command was not found. Install iproute2 or run this on the robot/HPC host." >&2
  exit 127
fi

printf 'Active network interfaces with IPv4 addresses:\n'
ip -br -4 addr show up || true

printf '\nLikely USB/USB-C network interfaces:\n'
found=0
while read -r iface _state addrs; do
  case "$iface" in
    usb*|enx*|enp*s*u*|enp*u*|eth*usb*)
      printf '  %-20s %s\n' "$iface" "${addrs:-no IPv4 address}"
      found=1
      ;;
  esac
done < <(ip -br -4 addr show up || true)

if [ "$found" = "0" ]; then
  cat <<'MSG'
  No obvious USB network interface with an IPv4 address was found.
  Plug in the USB/USB-C cable, enable USB networking if needed, then compare:
    ip -br link
  before and after plugging in the cable.
MSG
fi

cat <<'MSG'

How to identify the two USB-link IPs:
  1. Run this script on the Franka laptop and note the USB interface IPv4 address.
  2. Run it on the HPC and note its USB interface IPv4 address.
  3. Test the link with ping in both directions.

Example if using the repository defaults:
  Franka laptop: 10.66.0.1
  HPC:           10.66.0.2
  From HPC:      ping 10.66.0.1
  From laptop:   ping 10.66.0.2
MSG
