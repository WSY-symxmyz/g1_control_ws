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
- A native bounded optimization IK backend for G1ARM5 and G1ARM7. It reproduces
  Unitree's dual-wrist objective, constraints, warm start, and output filter
  without requiring `pinocchio.casadi`.
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

## Optimization IK

The default `native_optimization` backend uses numeric Pinocchio kinematics and
SciPy's bounded trust-region least-squares solver. Its residual preserves the
Unitree objective terms for dual-wrist translation, rotation, joint
regularization, and previous-command smoothing. The G1ARM5 backend also keeps
Unitree's `0.20 m` wrist endpoint offset and `50.0`, `0.5`, `0.02`, `0.1`
weights.

Runtime protection includes finite pose checks, SO(3) normalization, URDF joint
limits, a configurable per-cycle output step, weighted moving-average output,
quality thresholds, and current-state fallback. The output step follows
Unitree's arm controller convention: when any joint exceeds the configured
step, the complete joint delta vector is scaled uniformly so its direction is
preserved. Low-rate debug output separates the raw solver error from the
limited output error and reports solve time and evaluation counts.

## Arm Tracking Modes

`arm_tracking_mode` selects how the Unitree-normalized XR wrist poses become IK
targets. Both modes require the configured deadman input in this ROS bridge.

### `relative_clutch`

This is the default and is recommended for first robot tests. On each deadman
press, the bridge captures both current XR wrist poses and both robot wrist
poses calculated from the measured arm joints. Subsequent XR translation and
rotation deltas are applied to those robot anchors. Releasing the deadman,
losing XR motion readiness, or losing the active XR event stream clears the
anchors; the next valid press captures fresh anchors.

This makes the first target equal to the robot's current pose and avoids a
large motion when the operator and robot begin in different postures.

### `unitree_absolute`

This mode follows the official Unitree arm target path. `TeleVuerWrapper` has
already converted OpenXR poses into the robot waist convention, including the
head-yaw and head-to-waist transforms, so the bridge sends those wrist SE(3)
poses directly to IK without capturing relative anchors. This preserves a
consistent operator-to-robot pose mapping for teleoperation demonstrations and
data collection.

The official example uses a one-time keyboard start and then tracks these poses
continuously. This bridge deliberately retains its ROS deadman gate: releasing
the deadman suppresses arm targets, but it does not alter the absolute mapping.
Re-engaging can therefore request a large posture change; always validate this
mode in dry-run and start the robot from a compatible neutral pose.

Optional `arm_position_scale` and `arm_rotation_scale` apply only to
`relative_clutch`. Keep both at `1.0` unless a reduced-motion test is intended.

The original vendored backend remains available for environments that provide
Pinocchio's CasADi binding:

```bash
ros2 launch g1_xr_teleop g1_xr_teleop.launch.py \
  enable_ik:=true ik_backend:=unitree_casadi
```

## Python Dependencies

Dry-run ROS 2 startup only requires this workspace's ROS dependencies. Enabling
XR or IK also requires the Python dependencies used by the vendored Unitree
components, including:

- `numpy`
- `vuer[all]==0.0.60`
- `params-proto==2.13.2`
- `opencv-python`
- `pinocchio` (the PyPI distribution is named `pin`)
- `scipy>=1.13,<1.16`

The default native IK does not require `casadi`, `pinocchio.casadi`, or
`meshcat`. Those packages are needed only by the vendored Unitree CasADi IK and
its visualization path.

For the project venv, activate it before sourcing and running ROS commands:

```bash
source ~/.venvs/g1_xr_teleop/bin/activate
export PYTHONNOUSERSITE=1
source ~/unitree_ros2/setup.sh
```

Verify that ROS and the native IK dependencies resolve in the same interpreter:

```bash
python3 -c "import rclpy, pinocchio, scipy; print(pinocchio.__version__, scipy.__version__)"
```

Do not combine NumPy 2.x with the Ubuntu 22.04 system SciPy 1.8. Install a
compatible SciPy wheel inside the venv:

```bash
python3 -m pip install "scipy>=1.13,<1.16"
```

Keep `params-proto` pinned for Vuer 0.0.60. Its unconstrained installation can
select params-proto 3.x, where the `Flag` API required by Vuer has been removed:

```bash
python3 -m pip install "vuer[all]==0.0.60" "params-proto==2.13.2"
python3 -c "from vuer import Vuer; print('Vuer import OK')"
```

Use the same environment strategy as Unitree's `xr_teleoperate` README for these
dependencies. Keep ROS 2 command execution in a shell that has sourced
`~/unitree_ros2/setup.sh` and `g1_control_ws/install/setup.bash`.

## Build

From the parent project:

```bash
cd ~/unitree_ros2
source ~/.venvs/g1_xr_teleop/bin/activate
export PYTHONNOUSERSITE=1
source ./setup.sh
cd g1_control_ws
python3 /usr/bin/colcon build --packages-select g1_xr_teleop
```

Calling `/usr/bin/colcon` through the active venv interpreter is intentional.
Running plain `colcon build` on this ROS Humble installation generates console
scripts with `#!/usr/bin/python3`, which cannot import Pinocchio installed only
inside the venv. Confirm the installed entry point after building:

```bash
head -n 1 install/g1_xr_teleop/lib/g1_xr_teleop/xr_teleop_node
```

It should name `~/.venvs/g1_xr_teleop/bin/python3`.

If `g1_control_msgs` has not been built in this workspace yet, build up to this
package instead:

```bash
python3 /usr/bin/colcon build --packages-up-to g1_xr_teleop
```

Run the robot-free IK consistency and timing test after building:

```bash
source install/setup.bash
ros2 run g1_xr_teleop ik_offline_check \
  --arm-model G1ARM5 --samples 200 --rate-hz 30
```

All samples should be accepted and `solve_p95_ms` should remain below the
reported `control_period_ms`.

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

This selects `ik_backend:=native_optimization` by default. During dry-run,
inspect `ik.accepted`, `ik.elapsed_ms`, left/right pose errors, and
`arm_target` in the one-Hz debug line. No arm command is published while
`dry_run:=true`.

Keep locomotion disabled while validating arm IK in isolation:

```bash
ros2 launch g1_xr_teleop g1_xr_teleop.launch.py \
  arm_model:=G1ARM5 enable_xr:=true enable_ik:=true \
  enable_arm:=true enable_loco:=false dry_run:=true
```

Start with relative clutch tracking:

```bash
ros2 launch g1_xr_teleop g1_xr_teleop.launch.py \
  arm_model:=G1ARM5 arm_tracking_mode:=relative_clutch \
  enable_xr:=true enable_ik:=true enable_loco:=false dry_run:=true
```

The first one-Hz debug line after pressing the deadman should show
`tracking.state=relative_tracking`, increment `capture_count`, and hold the
measured arm state for the capture cycle. Move one wrist slowly,
release the deadman, change posture, and press again; `capture_count` should
increment without a target jump.

After relative tracking is verified, inspect the important absolute mapping:

```bash
ros2 launch g1_xr_teleop g1_xr_teleop.launch.py \
  arm_model:=G1ARM5 arm_tracking_mode:=unitree_absolute \
  enable_xr:=true enable_ik:=true enable_loco:=false dry_run:=true
```

Before setting `dry_run:=false`, verify `ik.accepted`, solver pose errors,
`raw_max_joint_step_rad`, `output_max_joint_step_rad`, and the difference
between `/g1/state/arm` and `arm_target`. Test arm motion with the robot secured,
locomotion disabled, small wrist motion, and a spotter near the emergency stop.

Important IK parameters are in `config/xr_teleop.yaml`:

- `ik_max_nfev`: maximum optimizer function evaluations per frame.
- `ik_max_output_step_rad`: maximum joint change per cycle; the entire joint
  delta vector is uniformly scaled when this limit is exceeded.
- `ik_max_position_error_m` and `ik_max_rotation_error_rad`: reject poor
  solutions and hold the measured joint state.
- `ik_filter_weights`: newest-to-oldest moving-average weights, matching the
  Unitree filter convention.

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
4. Run `ik_offline_check`, then enable native IK and inspect `ik` diagnostics
   and `arm_target` without publishing.
5. Publish arm commands at low scale and low rate with a deadman input.
6. Test locomotion commands alone with conservative velocity limits.
7. Combine arm and locomotion after both paths are stable.
8. Add episode recording and hand retargeting in later iterations.
