# Direct USB/USB-C link between Franka laptop and HPC

Yes: moving the HPC off the camera switch can help if the camera timeouts started
after the HPC was connected to the same Ethernet switch as the FRAMOS/RealSense
camera and laptop. The direct USB network link gives HPC traffic its own
point-to-point path, so camera traffic stays on the camera/laptop network.

Recommended layout:

```text
FRAMOS/RealSense camera(s)  <->  camera switch / camera NIC  <->  Franka laptop
HPC                         <->  direct USB/USB-C network    <->  Franka laptop
```

The wrapper scripts assume this static USB-link addressing by default:

- Franka laptop USB IP: `10.66.0.1`
- HPC USB IP: `10.66.0.2`

You can override both with `LAPTOP_USB_IP` and `HPC_USB_IP`.


## Find the USB-link IP addresses

Run this helper on both machines after plugging in the USB/USB-C cable:

```bash
./scripts/find_usb_network_link.sh
```

Look for interfaces named like `usb*`, `enx*`, or `enp...u...`. If no IPv4
address is shown, compare `ip -br link` before and after plugging in the cable or
configure static addresses such as `10.66.0.1/24` on the Franka laptop and
`10.66.0.2/24` on the HPC.

## Remote-camera recording workflow

On the HPC:

```bash
HPC_USB_IP=10.66.0.2 LAPTOP_USB_IP=10.66.0.1 \
  ./start_hpc_usb_remote_camera_recorder.sh
```

On the Franka laptop:

```bash
HPC_USB_IP=10.66.0.2 LAPTOP_USB_IP=10.66.0.1 \
  ./start_gello_panda_usb_link.sh
```

This makes the laptop stream robot state/action messages to the HPC over the USB
link and makes the HPC pull camera frames from the laptop USB IP instead of from
a shared camera switch path.

## HPC policy rollout workflow

For policy rollouts where the HPC needs robot and camera ZMQ ports, keep the
reverse tunnel but run it over the USB-link IP:

```bash
HPC_USB_IP=10.66.0.2 LAPTOP_USB_IP=10.66.0.1 \
  ./start_franka_to_hpc_usb_reverse_tunnel.sh
```

Then run the HPC rollout with `ROBOT_HOST=127.0.0.1` and, unless you intentionally
separate camera routing, leave `CAMERA_HOST` unset so it also uses the tunnel.


## SSH traffic during recording

An idle SSH shell usually produces negligible traffic and should not by itself
overload the camera network. It can still contribute to camera timeouts if it
runs over the same switch/interface as the cameras and carries heavy traffic, for
example `scp`/`rsync`, X11 forwarding, large logs, port forwards, or a policy
rollout tunnel. Prefer running laptop<->HPC SSH over the direct USB link, or keep
heavy transfers stopped during recording.

## Checks

On both machines, verify the USB interface IPs before starting:

```bash
ip addr
ping 10.66.0.1   # from the HPC
ping 10.66.0.2   # from the Franka laptop
```

On the laptop, keep the camera servers reachable on the laptop side:

```bash
ss -ltnp | grep -E ':(5000|5001|6001)'
```
