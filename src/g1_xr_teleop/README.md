# g1_xr_teleop

`g1_xr_teleop` is a ROS 2 teleoperation bridge for the Unitree G1 control
workspace. It follows the data flow used by Unitree's `xr_teleoperate`
repository, but keeps robot execution behind this project's stable ROS 2 API.

The bridge is intended to consume XR wrist, hand, and controller data, convert
that input into arm and locomotion commands, and publish only:

- `/g1/command/arm_joints`
- `/g1/command/loco`

It subscribes to:

- `/g1/state/arm`
- `/g1/state/controller_status`

The package does not publish directly to `/arm_sdk`, does not publish Unitree
low-level DDS commands, and does not call Unitree locomotion clients directly.
Those responsibilities remain in `g1_control_interface`.

## Current Status

This first scaffold provides:

- ROS 2 package metadata, launch, and configuration.
- A safe `dry_run` default.
- State subscriptions for the existing G1 control interface.
- Command mappers for arm joint targets and locomotion velocity.
- Safety checks for controller status, XR readiness, deadman input, NaN values,
  and joint target jumps.
- Vendored Unitree `TeleVuerWrapper`, G1 arm IK helpers, and G1 URDF/mesh assets.

The default launch does not command the robot. Enable XR and IK only after their
Python dependencies are installed.

## Vendored Unitree Files

The package currently vendors the Unitree files needed by the first arm/loco
teleoperation path:

- `g1_xr_teleop/assets/g1`: G1 23-DoF and 29-DoF URDF/MJCF files plus meshes.
- `g1_xr_teleop/third_party/unitree_xr/robot_arm_ik.py`: Unitree G1 IK code,
  patched to use this package's local assets.
- `g1_xr_teleop/third_party/unitree_xr/weighted_moving_filter.py`: smoothing
  helper used by the IK implementation.
- `g1_xr_teleop/third_party/televuer`: Unitree's XR browser/WebXR adapter.

Dexterous-hand retargeting assets and the `dex-retargeting` package are not
vendored yet because this first stage does not command hands. Copy those assets
when hand retargeting becomes part of this package.

## Python Dependencies

Dry-run ROS 2 startup only requires this workspace's ROS dependencies. Enabling
XR or IK also requires the Python dependencies used by the vendored Unitree
components, including:

- `numpy`
- `vuer`
- `opencv-python`
- `pinocchio`
- `casadi`
- `meshcat`

Use the same environment strategy as Unitree's `xr_teleoperate` README for these
dependencies. Keep ROS 2 command execution in a shell that has sourced
`~/unitree_ros2/setup.sh` and `g1_control_ws/install/setup.bash`.

## Build

From the parent project:

```bash
cd ~/unitree_ros2
source ./setup.sh
cd g1_control_ws
colcon build --packages-select g1_xr_teleop
```

If `g1_control_msgs` has not been built in this workspace yet, build up to this
package instead:

```bash
colcon build --packages-up-to g1_xr_teleop
```

## Run

Start the robot control interface first:

```bash
cd ~/unitree_ros2
source ./setup.sh
ros2 launch g1_control_interface g1_control_interface.launch.py arm_model:=G1ARM5
```

Check status:

```bash
ros2 topic echo /g1/state/controller_status
ros2 topic echo /g1/state/arm
```

Start the teleop bridge in dry-run mode:

```bash
ros2 launch g1_xr_teleop g1_xr_teleop.launch.py
```

The default arm model is `G1ARM5`, matching a 23-DoF G1 without dexterous hands
or waist joints exposed through the teleop interface. Override it when needed:

```bash
ros2 launch g1_xr_teleop g1_xr_teleop.launch.py arm_model:=G1ARM7
```

Enable XR and IK only when their dependencies are installed:

```bash
ros2 launch g1_xr_teleop g1_xr_teleop.launch.py enable_xr:=true enable_ik:=true
```

Publish motion commands only after dry-run output has been checked:

```bash
ros2 launch g1_xr_teleop g1_xr_teleop.launch.py dry_run:=false enable_xr:=true enable_ik:=true
```

## Meta Quest Connection

The first XR path uses the vendored Unitree TeleVuer browser/WebXR approach.
The PC runs the TeleVuer HTTPS/WSS server, and the Quest browser connects to it.
The controllers are paired with the Quest headset; the PC does not connect to
the controllers directly.

### Network Layout

Use two PC network interfaces when the G1 control network and the normal
router/Wi-Fi network are separate:

```text
G1 robot  -- Ethernet --> router / G1 control network
PC wired  -- Ethernet --> same G1 control network
PC Wi-Fi  -- Wi-Fi ----> router Wi-Fi used by Quest
Quest 3   -- Wi-Fi ----> same Wi-Fi/router network as the PC Wi-Fi interface
```

Recommended PC addressing:

```text
PC wired interface, for G1:
  IPv4 address: 192.168.123.160/24
  gateway: empty

PC Wi-Fi interface, for Quest and internet:
  IPv4 address: DHCP, for example 192.168.0.128/24
  gateway: router address, for example 192.168.0.1
```

Do not set a default gateway on the wired G1 interface. The PC should use the
wired interface for G1 communication and the Wi-Fi interface for Quest,
TeleVuer, and normal internet access.

Check the active addresses before launching:

```bash
ip -br addr
ip route
```

Expected shape:

```text
enp...    UP    192.168.123.160/24
wlp...    UP    192.168.0.xxx/24
default via 192.168.0.1 dev wlp...
192.168.123.0/24 dev enp... src 192.168.123.160
```

The Quest browser must use the PC Wi-Fi IP, not the G1 wired IP. For example,
if the PC Wi-Fi address is `192.168.0.128`, use that address in the Quest URL.

### Quest Browser Flow

1. Connect the Quest 3 headset to the same router/Wi-Fi network as the PC Wi-Fi
   interface.
2. Configure the HTTPS certificate used by TeleVuer. It checks, in order:
   `XR_TELEOP_CERT`/`XR_TELEOP_KEY`, then `~/.config/xr_teleoperate/cert.pem`
   and `key.pem`.
3. Start this node with XR enabled and keep `dry_run:=true` during input tests:

   ```bash
   ros2 launch g1_xr_teleop g1_xr_teleop.launch.py arm_model:=G1ARM5 enable_xr:=true enable_ik:=false dry_run:=true
   ```

4. Confirm that port `8012` is owned by the current teleop process:

   ```bash
   ss -ltnp | grep 8012
   curl -k https://<pc-wifi-ip>:8012
   ```

   If this returns a Vuer page before the node is launched, an old Vuer process
   is still running. Stop it before testing.

5. In the Quest browser, open:

   ```text
   https://<pc-wifi-ip>:8012/?ws=wss://<pc-wifi-ip>:8012
   ```

   Example:

   ```text
   https://192.168.0.128:8012/?ws=wss://192.168.0.128:8012
   ```

   If the local page opens but no XR events arrive, try Unitree's alternate
   front-end URL:

   ```text
   https://vuer.ai?ws=wss://192.168.0.128:8012
   ```

6. Accept the certificate warning or install the certificate on the headset.
7. In the Vuer page, click `Virtual Reality` and allow the browser prompts.
8. Move the headset and controllers while checking the ROS logs. In dry-run
   mode the node prints low-rate XR summaries, including head pose, left/right
   wrist pose, trigger values, squeeze values, thumbsticks, buttons, event
   counters, `xr_health`, safety state, and target availability. `xr_health`
   is diagnostic only: when it reports `stuck=True`, adjust or re-wear the
   headset and confirm the event counters recover.

## Development Plan

1. Verify ROS 2 build and dry-run startup.
2. Verify `/g1/state/arm` and `/g1/state/controller_status` subscriptions.
3. Initialize XR input and print `motion_data_ready`, wrist poses, controller
   triggers, buttons, and thumbsticks.
4. Enable IK and inspect joint targets without publishing.
5. Publish arm commands at low scale and low rate with a deadman input.
6. Test locomotion commands alone with conservative velocity limits.
7. Combine arm and locomotion after both paths are stable.
8. Add episode recording and hand retargeting in later iterations.
