# G1 Control Workspace

`g1_control_ws` is a ROS 2 workspace that provides a unified control interface
for the Unitree G1 robot.

The goal of this workspace is to offer a safer and cleaner entry point for:

- upper-body joint control through the dedicated `arm_sdk` channel
- lower-body locomotion through the Unitree high-level locomotion API
- teleoperation, scripted testing, and future simulation or policy training

This workspace is designed to sit next to the original `unitree_ros2`
repository content and reuse the existing Unitree ROS 2 dependencies that are
already built in `cyclonedds_ws` and `example`.

## Installation in `unitree_ros2`

Clone this repository directly under the root of your existing
`unitree_ros2` workspace. The expected final path is:

```bash
~/unitree_ros2/g1_control_ws
```

For example:

```bash
cd ~/unitree_ros2
git clone https://github.com/<YOUR_GITHUB_USER>/<YOUR_REPOSITORY>.git g1_control_ws
```

Make sure the repository-level `setup.sh` sources this workspace after the
Unitree dependencies. Add the following block to `~/unitree_ros2/setup.sh` if it
is not already present:

```bash
# Load the unified G1 control workspace when it has been built.
if [ -f "$HOME/unitree_ros2/g1_control_ws/install/setup.bash" ]; then
  source "$HOME/unitree_ros2/g1_control_ws/install/setup.bash"
fi
```

The block is safe before the first build because it only sources
`g1_control_ws/install/setup.bash` when that file exists.

## Workspace Layout

The workspace currently contains two ROS 2 packages:

- `g1_control_msgs`
  - custom ROS 2 messages and services used by the unified control interface
- `g1_control_interface`
  - the runtime node, arm controller, locomotion controller, and launch/config
    files

Important directories:

- `src/g1_control_msgs`
- `src/g1_control_interface`
- `install/setup.bash`
- `src/g1_control_interface/config`
- `src/g1_control_interface/launch`

## Control Design

This workspace intentionally separates arm and locomotion control:

- **Arm control**
  - uses the dedicated `/arm_sdk` topic
  - consumes `/lowstate`
  - exposes named upper-body joints
  - provides a high-rate streaming interface for teleoperation or RL
  - provides a service-based queued interface for single position moves
  - performs command validation, joint-limit clamping, velocity limiting, and
    official-style arm_sdk release behavior
  - publishes `/arm_sdk` from a dedicated 50 Hz control thread so locomotion
    API calls or status reporting cannot block arm position control

- **Locomotion control**
  - uses the Unitree `/api/sport/request` and `/api/sport/response` interface
  - supports the locomotion mode based on `fsm_id = 500`
  - exposes velocity commands rather than low-level leg joint control
  - includes keepalive and timeout stop behavior
  - creates one persistent response subscription and matches replies by Unitree
    request identity, avoiding per-command DDS endpoint creation
  - checks FSM once after startup, requests `fsm_id=500` only when needed, and
    verifies the result before accepting velocity commands

This split is important for long-term reliability. In normal usage, the arm
should be controlled through `/arm_sdk`, while walking and turning should be
controlled through the high-level locomotion interface.

Arm control has two public command paths:

- `/g1/command/arm_joints` is the servo/streaming path. Use this for keyboard,
  joystick, RL teleoperation, or policies that send frequent small target
  updates. New commands replace the previous target immediately; they are not
  queued.
- `/g1/arm/set_pose` is the queued single-position path. Use this for larger
  absolute-position targets during testing or task-level scripted motion.
  Requests are executed in FIFO order.

Both paths are converted into absolute joint positions before publishing
`unitree_hg/msg/LowCmd` on `/arm_sdk`. The controller always limits the actual
published command step, so `/arm_sdk` does not receive sudden large position
jumps.

## Dependencies

Before using this workspace, make sure the following are available:

- ROS 2 Humble
- `cyclonedds_ws/install/setup.bash`
- the Unitree packages already used by this repository
  - `unitree_api`
  - `unitree_hg`

The repository-level `setup.sh` already sources the expected dependencies and,
when present, also sources:

- `example/install/setup.bash`
- `g1_control_ws/install/setup.bash`

## Build

Build this workspace from the `g1_control_ws` directory:

```bash
cd ~/unitree_ros2
source ./setup.sh
cd g1_control_ws
colcon build
```

After a successful build:

```bash
source ~/unitree_ros2/g1_control_ws/install/setup.bash
```

For new terminals after the first successful build, it is enough to source the
repository-level setup file:

```bash
cd ~/unitree_ros2
source ./setup.sh
```

## Launch

Start the unified control node:

```bash
cd ~/unitree_ros2
source ./setup.sh
ros2 launch g1_control_interface g1_control_interface.launch.py arm_model:=G1ARM5
```

If your robot uses the 7-DoF arm layout:

```bash
ros2 launch g1_control_interface g1_control_interface.launch.py arm_model:=G1ARM7
```

## Topics and Services

### Input Topics

- `/g1/command/arm_joints`
  - type: `g1_control_msgs/msg/ArmJointCommand`
- `/g1/command/loco`
  - type: `g1_control_msgs/msg/LocoCommand`

### Output Topics

- `/g1/state/arm`
  - type: `g1_control_msgs/msg/ArmJointState`
- `/g1/state/controller_status`
  - type: `g1_control_msgs/msg/ControllerStatus`
- `/g1/debug/arm_command`
  - type: `g1_control_msgs/msg/ArmCommandDebug`
  - publishes measured, target, and commanded arm positions for debugging

### Services

- `/g1/arm/set_pose`
  - type: `g1_control_msgs/srv/SetArmPose`
- `/g1/arm/stop`
  - type: `g1_control_msgs/srv/StopArm`
- `/g1/loco/set_mode`
  - type: `g1_control_msgs/srv/SetLocoMode`
- `/g1/loco/stop`
  - type: `g1_control_msgs/srv/StopLoco`

## Message and Service Summary

### `ArmJointCommand`

Used for streaming arm commands over a topic. This is the preferred interface
for teleoperation and RL.

Fields:

- `joint_names`
- `is_delta`
- `positions`
- `deltas`
- `velocities`
- `kp`
- `kd`
- `duration`
- `hold`

When `is_delta` is false, `positions` contains absolute joint targets. When
`is_delta` is true, `deltas` contains increments that are added to the current
servo target. Delta commands are clamped per command. Absolute streaming
commands are also clamped per command relative to the latest measured pose, or
to the last commanded pose if `/lowstate` is briefly stale. This keeps
teleoperation responsive without allowing a single large target jump.

Streaming commands use a latest-command-wins policy. If a new command arrives
before the arm reaches the previous target, the internal target is updated
immediately. The already published command position remains continuous and
continues moving toward the latest target under the configured velocity limit.
The controller then clamps the resulting absolute targets to joint limits and
publishes smooth absolute commands to `/arm_sdk`.

### `ArmCommandDebug`

Used to inspect the controller's internal arm command pipeline.

Fields:

- `joint_names`
- `measured_positions`
- `target_positions`
- `commanded_positions`
- `control_active`
- `trajectory_mode`
- `release_after_motion`

`measured_positions` comes from `/lowstate`. `target_positions` is the latest
internal target after applying absolute or delta commands. `commanded_positions`
is the actual position vector sent to `/arm_sdk` after velocity limiting.

### `LocoCommand`

Used for velocity-based locomotion commands.

Fields:

- `vx`
- `vy`
- `omega`
- `duration`
- `continuous`
- `enable`

### `SetArmPose`

Used for simple service-based arm motion requests.
The request is interpreted as a sparse joint command: only joints listed in
`joint_names` are moved to new targets, while all unlisted joints hold their
latest measured positions. `velocities`, `kp`, and `kd` are optional per-joint
arrays and must not be longer than `joint_names`. Requests are queued; if a new
request arrives while another service target is still moving, the new target is
executed after the controller's commanded position reaches the current target.

### `SetLocoMode`

Used to enable or disable locomotion mode. The current unified interface only
supports `fsm_id = 500` for locomotion. The controller automatically queries the
FSM once after startup. If the robot is already in FSM 500, it becomes ready
without sending a mode command. Otherwise it requests FSM 500 and verifies the
result. Enabling through this service re-runs that initialization only when FSM
500 is not currently confirmed.

Disabling sends zero velocity and stops accepting motion commands. `StopLoco`
only stops the current motion, so later commands can resume. Neither operation
switches the robot out of FSM 500, and velocity commands never set the FSM
themselves.

## Parameters

The main parameters are in:

- `src/g1_control_interface/config/controller.yaml`
- `src/g1_control_interface/config/g1_arm5.yaml`
- `src/g1_control_interface/config/g1_arm7.yaml`
- `src/g1_control_interface/config/joint_limits.yaml`

Notable parameters:

- `arm_model`
- `arm_control_period_sec`
- `arm_servo_max_delta_per_command_rad`
- `arm_servo_max_abs_step_per_command_rad`
- `arm_max_position_step_rad`
- `arm_lowstate_fresh_timeout_sec`
- `arm_release_duration_sec`
- `arm_default_kp`
- `arm_default_kd`
- `arm_waist_kp_scale`
- `arm_waist_kd_scale`
- `loco_command_timeout_sec`
- `loco_keepalive_period_sec`
- `loco_fsm_startup_delay_sec`
- `loco_api_response_timeout_sec`
- `loco_velocity_duration_sec`
- `loco_api_failure_limit`
- `loco_max_vx`
- `loco_max_vy`
- `loco_max_omega`

The arm joint list and the joint limits are also parameterized, so they can be
adjusted without changing source code.

`arm_max_position_step_rad` limits how much the commanded joint position may
change on each control tick. The default `0.01` rad at a 20 ms control period
matches the original `g1_arm_sdk_dds_example` default of about `0.5 rad/s`.
Per-joint velocity limits in `joint_limits.yaml` are also applied, so the
effective step is the smaller of `arm_max_position_step_rad` and
`joint_limit_max_velocity * arm_control_period_sec`.

`arm_servo_max_delta_per_command_rad` limits a single incoming delta command
before it is accumulated into the streaming target. This protects keyboard,
joystick, and RL clients from accidentally sending one very large relative
step.

`arm_servo_max_abs_step_per_command_rad` limits a single incoming absolute
streaming command relative to the latest measured or commanded pose. Large
absolute targets are therefore converted into safe small servo steps instead of
being rejected.

`arm_release_duration_sec` controls how long `/g1/arm/stop` takes to lower the
arm_sdk control weight from `1.0` to `0.0`. Stop releases arm_sdk control; it
does not move the arm to a configured home pose.

## Minimal Verification

### 1. Check node status

In a second terminal:

```bash
cd ~/unitree_ros2
source ./setup.sh
ros2 topic echo /g1/state/controller_status
```

Look for:

- `lowstate_online: true`

The locomotion FSM check starts asynchronously after
`loco_fsm_startup_delay_sec`, so it does not block node construction or the arm
control thread. Before sending locomotion commands, wait for:

- `fsm_id: 500`
- `message: locomotion ready; FSM already 500`, or
  `message: locomotion ready; FSM set and verified as 500`

If the query, mode change, or verification fails, `fsm_id` remains `-1` and
velocity commands are rejected.

If `lowstate_online` is false, do not proceed with arm commands.

### 2. Test arm control

For teleoperation/RL-style streaming, start with a very small delta command:

```bash
ros2 topic pub --once /g1/command/arm_joints g1_control_msgs/msg/ArmJointCommand "{joint_names: ['left_shoulder_pitch'], is_delta: true, positions: [], deltas: [0.02], velocities: [], kp: [40.0], kd: [1.0], duration: {sec: 0, nanosec: 0}, hold: true}"
```

You can also run the minimal keyboard delta teleoperation demo:

```bash
ros2 run g1_control_interface g1_arm_keyboard_delta_teleop
```

Keyboard mapping:

- `w` / `s`: increase/decrease `left_shoulder_pitch`
- `a` / `d`: increase/decrease `left_shoulder_roll`
- up/down arrows: increase/decrease `right_shoulder_pitch`
- left/right arrows: increase/decrease `right_shoulder_roll`
- `q`: quit

The demo publishes `ArmJointCommand` messages with `is_delta=true`. Holding a
key relies on the terminal's key-repeat behavior, so repeated key events keep
sending small delta commands through the same streaming/RL path. The default
step is `0.01 rad` per key event and can be changed with a ROS parameter:

```bash
ros2 run g1_control_interface g1_arm_keyboard_delta_teleop --ros-args -p delta_step_rad:=0.005
```

You can also stream small absolute targets:

```bash
ros2 topic pub --once /g1/command/arm_joints g1_control_msgs/msg/ArmJointCommand "{joint_names: ['left_shoulder_pitch'], is_delta: false, positions: [0.35], deltas: [], velocities: [], kp: [40.0], kd: [1.0], duration: {sec: 0, nanosec: 0}, hold: true}"
```

For a larger debug or task-level move, use the service path:

```bash
ros2 service call /g1/arm/set_pose g1_control_msgs/srv/SetArmPose "{joint_names: ['left_shoulder_pitch'], positions: [0.2], velocities: [0.0], kp: [40.0], kd: [1.0], duration_sec: 2.0, hold: true}"
```

To stop arm control:

```bash
ros2 service call /g1/arm/stop g1_control_msgs/srv/StopArm "{}"
```

This keeps the current commanded pose while the arm_sdk control weight is
lowered to zero, then stops publishing arm commands.

### 3. Test locomotion control

First verify that startup FSM initialization completed:

```bash
ros2 topic echo /g1/state/controller_status
```

Wait for `fsm_id: 500`. The service below is only needed to re-enable the
command interface or retry FSM initialization after an error:

```bash
ros2 service call /g1/loco/set_mode g1_control_msgs/srv/SetLocoMode "{enable: true, continuous: true, requested_fsm_id: 500}"
```

Publish a very small forward command:

```bash
ros2 topic pub --once /g1/command/loco g1_control_msgs/msg/LocoCommand "{vx: 0.05, vy: 0.0, omega: 0.0, duration: {sec: 2, nanosec: 0}, continuous: false, enable: true}"
```

Publish a very small turn command:

```bash
ros2 topic pub --once /g1/command/loco g1_control_msgs/msg/LocoCommand "{vx: 0.0, vy: 0.0, omega: 0.15, duration: {sec: 2, nanosec: 0}, continuous: false, enable: true}"
```

Stop locomotion:

```bash
ros2 service call /g1/loco/stop g1_control_msgs/srv/StopLoco "{}"
```

The interface follows the official non-continuous `Move()` behavior by sending
`7105 SetVelocity` with a default one-second validity horizon while refreshing
it every `loco_keepalive_period_sec`. If the ROS command stream becomes stale,
the independent `loco_command_timeout_sec` watchdog immediately sends zero
velocity. A stop does not resend `7101` and does not invalidate the confirmed
FSM 500 state.

For API-level verification, observe the shared Unitree request topic:

```bash
timeout 10s ros2 topic echo /api/sport/request
```

Requests from this interface use nanosecond-scale identity values. With the
robot already in FSM 500, startup should produce one `7001` query and no `7101`
mode change. A sustained velocity command should then produce uninterrupted
`7105` requests at approximately `1 / loco_keepalive_period_sec`, with
`duration` equal to `loco_velocity_duration_sec`. There should be no five-second
gap. Other bare DDS applications may publish their own requests on this shared
topic, so identify this interface by its identity range and payload.

## Debugging Arm Streaming

When testing teleoperation or RL streaming, echo the debug topic:

```bash
ros2 topic echo /g1/debug/arm_command
```

To focus on the first joint in the G1ARM5 list, compare:

- `target_positions[0]`
- `commanded_positions[0]`
- `measured_positions[0]`

Interpretation:

- If `target_positions[0]` grows smoothly but `commanded_positions[0]` grows in
  jumps, the issue is inside the interface control loop or QoS.
- If `commanded_positions[0]` grows smoothly but `measured_positions[0]` moves
  in delayed jumps, the interface is publishing smoothly and the jump is coming
  from robot-side response, stiffness, friction, or `/arm_sdk` takeover
  behavior.
- If all three grow in jumps, check the upstream command source rate and QoS.

## Safety Notes

- Start with small arm motions and small locomotion commands.
- Ensure enough free space around the robot.
- Do not test large arm motions and walking at the same time as the first
  validation step.
- Use the correct arm model:
  - `G1ARM5`
  - `G1ARM7`
- Joint limits in `joint_limits.yaml` should be reviewed and tuned for your
  exact hardware configuration before aggressive testing.

## Current Scope and Known Limits

This workspace is intended to be a solid starting point, but it is not a full
robot framework yet.

Current limitations:

- locomotion is intentionally limited to the high-level `fsm_id = 500` path
- there is no command-source arbitration yet
- there is no dedicated emergency-stop topic or service yet
- the arm controller currently focuses on position-oriented commands
- simulation backends are not implemented yet, although the interface is meant
  to be reusable for future simulation or policy training

## Recommended Next Steps

For long-term use, the following additions are recommended:

- a teleoperation node for keyboard or joystick input
- a Python client library for testing and training scripts
- a command arbitration layer for multiple command sources
- a dedicated emergency-stop path
- tighter, robot-specific joint limit tuning
- a simulation adapter that preserves the same public ROS 2 interface

## License and Usage

This workspace is intended to be used together with the existing Unitree ROS 2
repository content. Please review and follow the license terms of the upstream
Unitree packages and any local modifications in this repository.
