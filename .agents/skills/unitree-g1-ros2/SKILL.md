---
name: unitree-g1-ros2
description: >
  Use when working inside the unitree_ros2 project, especially for the Unitree G1
  ROS 2 workspace structure, setup/build workflows, official-code boundaries,
  g1_control_ws interface development, RL/task-control integration, DDS/network
  setup, or GitHub publishing of the local G1 control workspace.
context: project
classification: project
category: robotics
version: 0.1.0
---

# Unitree G1 ROS 2 Project

## Project Shape

This skill describes how `g1_control_ws` fits into the parent `unitree_ros2`
project. The expected parent project contains three main ROS 2 workspaces:

- `cyclonedds_ws`: Unitree-provided dependency workspace. Treat as upstream
  official code.
- `example`: Unitree-provided example workspace. Treat as upstream official
  code, except for necessary local setup or configuration.
- `g1_control_ws`: local interface workspace for controlling the Unitree G1 arm
  and locomotion APIs. This is the main development area.

Within `g1_control_ws`, the ROS 2 packages are:

- `src/g1_control_msgs`: custom messages and services.
- `src/g1_control_interface`: runtime node, arm controller, locomotion
  controller, launch files, and configuration.

## Ownership Boundaries

Default to minimal changes in `cyclonedds_ws` and `example`. If changes are
needed there, prefer narrowly scoped configuration or setup edits and explain
why the official workspace must be touched.

Use `g1_control_ws` for new project behavior, especially:

- arm and locomotion interface changes
- RL or policy-facing APIs
- teleoperation helpers
- task-execution bridge nodes
- message/service additions
- safety, command validation, and status reporting

Do not commit generated workspace outputs:

- `build/`
- `install/`
- `log/`

This applies both at the parent `unitree_ros2` level and inside
`g1_control_ws`.

## Environment Setup

The expected setup order is:

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/unitree_ros2/example/install/setup.bash
source ~/unitree_ros2/g1_control_ws/install/setup.bash
```

The parent `setup.sh` should source optional workspaces conditionally, so a
fresh checkout can be sourced before `example` or `g1_control_ws` has been
built.

Keep `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. The CycloneDDS network interface
may need machine-specific configuration in `setup.sh`.

## Build Workflow

When changing `g1_control_ws`, build from the workspace root:

```bash
cd ~/unitree_ros2
source ./setup.sh
cd g1_control_ws
colcon build
```

After the first successful build, new shells can usually use:

```bash
cd ~/unitree_ros2
source ./setup.sh
```

Prefer focused builds when iterating on one package:

```bash
colcon build --packages-select g1_control_msgs g1_control_interface
```

If message or service definitions change, rebuild both `g1_control_msgs` and
all packages that depend on it.

## Runtime Entry Points

Launch the unified control node from the parent project environment:

```bash
cd ~/unitree_ros2
source ./setup.sh
ros2 launch g1_control_interface g1_control_interface.launch.py arm_model:=G1ARM5
```

Use `arm_model:=G1ARM7` for the 7-DoF arm layout.

The intended public interface is the `g1_control_ws` ROS 2 API, not direct
editing or extension of Unitree official examples.

## Control Design Notes

Arm and locomotion control should remain separated:

- Arm control uses `/arm_sdk` and consumes `/lowstate`.
- Locomotion control uses the Unitree high-level sport API request/response
  path.

Keep command validation, joint-limit clamping, velocity limiting, timeout
behavior, and clear status/debug topics in the interface layer. RL, task
execution, and teleoperation code should call the stable `g1_control_ws` API
rather than depending on lower-level Unitree internals.

## Verification

For code changes, at minimum:

```bash
cd ~/unitree_ros2
source ./setup.sh
cd g1_control_ws
colcon build
```

For robot-facing behavior, verify status topics before motion and start with
small commands. Do not combine first-time large arm motions with walking tests.

Useful checks include:

```bash
ros2 topic echo /g1/state/controller_status
ros2 topic echo /g1/debug/arm_command
```

Only proceed with arm commands when `/lowstate` is online.

## GitHub Publishing

`g1_control_ws` is also maintained as its own GitHub repository. When publishing
or updating it, operate from:

```bash
cd ~/unitree_ros2/g1_control_ws
```

Keep its repository independent from the parent `unitree_ros2` repository, and
push only source, configuration, launch, message/service, README, changelog, and
other intentional project files.
