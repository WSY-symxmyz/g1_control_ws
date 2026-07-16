import math
import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from g1_control_msgs.msg import (
    ArmJointCommand,
    ArmJointState,
    ControllerStatus as ControllerStatusMsg,
    LocoCommand,
)

from .arm_command_mapper import ArmCommandMapper
from .arm_ik_adapter import ArmIKAdapter
from .calibration import CalibrationState
from .loco_command_mapper import LocoCommandMapper
from .safety_gate import SafetyGate
from .teleop_types import ArmState, ControllerStatus
from .xr_input_adapter import XRInputAdapter


class G1XRTeleopNode(Node):
    def __init__(self):
        super().__init__("g1_xr_teleop_node")
        self._declare_parameters()

        self.dry_run = self.get_parameter("dry_run").value
        self.enable_arm = self.get_parameter("enable_arm").value
        self.enable_loco = self.get_parameter("enable_loco").value
        self.print_debug = self.get_parameter("print_debug").value
        self.arm_model = self.get_parameter("arm_model").value
        self.arm_joint_names = self._arm_joint_names()

        arm_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.arm_command_pub = self.create_publisher(
            ArmJointCommand, "/g1/command/arm_joints", arm_qos
        )
        self.loco_command_pub = self.create_publisher(LocoCommand, "/g1/command/loco", 10)
        self.arm_state_sub = self.create_subscription(
            ArmJointState,
            "/g1/state/arm",
            self._on_arm_state,
            10,
        )
        self.status_sub = self.create_subscription(
            ControllerStatusMsg,
            "/g1/state/controller_status",
            self._on_status,
            10,
        )

        self.arm_state = ArmState()
        self.status = ControllerStatus()
        self.last_safety_reasons = []
        self.last_loco_allowed = False
        self.debug_tick_count = 0
        self.xr_stuck_timeout_sec = float(self.get_parameter("xr_stuck_timeout_sec").value)
        self.xr_event_counts = {
            "camera_events": None,
            "controller_events": None,
            "hand_events": None,
        }
        self.xr_event_times = {
            "camera_events": None,
            "controller_events": None,
            "hand_events": None,
        }
        self.xr_value_signatures = {
            "head_pose": None,
            "controller": None,
        }
        self.xr_value_times = {
            "head_pose": None,
            "controller": None,
        }
        self.xr_health = {
            "stuck": False,
            "camera_stale": False,
            "controller_stale": False,
            "hand_stale": False,
            "head_pose_stale": False,
            "controller_value_stale": False,
            "camera_delta": 0,
            "controller_delta": 0,
            "hand_delta": 0,
            "camera_age_sec": None,
            "controller_age_sec": None,
            "hand_age_sec": None,
            "head_pose_change_age_sec": None,
            "controller_value_change_age_sec": None,
        }

        self.calibration = CalibrationState()
        self.arm_mapper = ArmCommandMapper(
            self.arm_joint_names,
            self.get_parameter("arm_command_duration_sec").value,
        )
        self.loco_mapper = LocoCommandMapper(
            self.get_parameter("loco_max_vx").value,
            self.get_parameter("loco_max_vy").value,
            self.get_parameter("loco_max_omega").value,
            self.get_parameter("loco_deadzone").value,
            self.get_parameter("loco_command_duration_sec").value,
            self.get_parameter("loco_continuous").value,
        )
        self.safety_gate = SafetyGate(
            self.get_parameter("require_deadman").value,
            self.get_parameter("deadman_source").value,
            self.get_parameter("deadman_threshold").value,
            self.get_parameter("max_abs_joint_step_rad").value,
        )

        self.xr_input = self._make_xr_input()
        self.arm_ik = self._make_arm_ik()

        period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            "g1_xr_teleop started "
            f"(dry_run={self.dry_run}, enable_xr={self.get_parameter('enable_xr').value}, "
            f"enable_ik={self.get_parameter('enable_ik').value}, arm_model={self.arm_model})"
        )

    def destroy_node(self):
        try:
            self.xr_input.close()
        except Exception:
            self.get_logger().warn("Failed to close XR input adapter")
        super().destroy_node()

    def _declare_parameters(self):
        self.declare_parameter("dry_run", True)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("input_mode", "controller")
        self.declare_parameter("display_mode", "pass-through")
        self.declare_parameter("arm_model", "G1ARM5")
        self.declare_parameter("img_server_ip", "192.168.123.164")
        self.declare_parameter("enable_xr", False)
        self.declare_parameter("enable_ik", False)
        self.declare_parameter("enable_arm", True)
        self.declare_parameter("enable_loco", True)
        self.declare_parameter("require_deadman", True)
        self.declare_parameter("deadman_source", "right_trigger")
        self.declare_parameter("deadman_threshold", 0.2)
        self.declare_parameter("max_abs_joint_step_rad", 0.2)
        self.declare_parameter("arm_command_duration_sec", 0.05)
        self.declare_parameter("loco_command_duration_sec", 0.15)
        self.declare_parameter("loco_continuous", True)
        self.declare_parameter("loco_max_vx", 0.2)
        self.declare_parameter("loco_max_vy", 0.1)
        self.declare_parameter("loco_max_omega", 0.25)
        self.declare_parameter("loco_deadzone", 0.08)
        self.declare_parameter("print_debug", True)
        self.declare_parameter("xr_stuck_timeout_sec", 1.0)
        self.declare_parameter(
            "g1arm7_joint_names",
            [
                "left_shoulder_pitch",
                "left_shoulder_roll",
                "left_shoulder_yaw",
                "left_elbow",
                "left_wrist_roll",
                "left_wrist_pitch",
                "left_wrist_yaw",
                "right_shoulder_pitch",
                "right_shoulder_roll",
                "right_shoulder_yaw",
                "right_elbow",
                "right_wrist_roll",
                "right_wrist_pitch",
                "right_wrist_yaw",
            ],
        )
        self.declare_parameter(
            "g1arm5_joint_names",
            [
                "left_shoulder_pitch",
                "left_shoulder_roll",
                "left_shoulder_yaw",
                "left_elbow_pitch",
                "left_elbow_roll",
                "right_shoulder_pitch",
                "right_shoulder_roll",
                "right_shoulder_yaw",
                "right_elbow_pitch",
                "right_elbow_roll",
            ],
        )

    def _make_xr_input(self):
        return XRInputAdapter(
            self.get_parameter("enable_xr").value,
            self.get_parameter("input_mode").value,
            self.get_parameter("display_mode").value,
            self.get_parameter("img_server_ip").value,
            self.get_logger(),
        )

    def _make_arm_ik(self):
        return ArmIKAdapter(
            self.get_parameter("enable_ik").value,
            self.arm_model,
            self.get_logger(),
        )

    def _arm_joint_names(self):
        if self.arm_model == "G1ARM5":
            return list(self.get_parameter("g1arm5_joint_names").value)
        return list(self.get_parameter("g1arm7_joint_names").value)

    def _on_arm_state(self, msg):
        self.arm_state = ArmState(
            joint_names=list(msg.joint_names),
            positions=list(msg.positions),
            velocities=list(msg.velocities),
            has_state=bool(msg.has_state),
        )

    def _on_status(self, msg):
        self.status = ControllerStatus(
            lowstate_online=bool(msg.lowstate_online),
            arm_control_active=bool(msg.arm_control_active),
            loco_control_active=bool(msg.loco_control_active),
            fsm_id=int(msg.fsm_id),
            fsm_mode=int(msg.fsm_mode),
            arm_model=str(msg.arm_model),
            message=str(msg.message),
        )

    def _on_timer(self):
        try:
            self._tick()
        except Exception:
            self.get_logger().error(traceback.format_exc())

    def _tick(self):
        frame = self.xr_input.get_frame()
        stamp = self.get_clock().now().to_msg()
        xr_debug = self.xr_input.debug_snapshot()
        xr_health = self._update_xr_health(frame, xr_debug)

        target_positions = None
        current_positions = self._current_positions_for_ik()
        current_velocities = self._current_velocities_for_ik()

        if self.enable_arm and self.get_parameter("enable_ik").value:
            left_wrist = self.calibration.apply_wrist_pose(frame.left_wrist_pose)
            right_wrist = self.calibration.apply_wrist_pose(frame.right_wrist_pose)
            target_positions = self.arm_ik.solve(
                left_wrist,
                right_wrist,
                current_positions,
                current_velocities,
            )

        safe, reasons = self.safety_gate.evaluate(
            self.status,
            frame,
            current_positions=current_positions,
            target_positions=target_positions,
        )
        self._log_safety_changes(safe, reasons)

        if frame.controller.right_a_button:
            self._publish_loco_stop(stamp)
            self.get_logger().warn("Right A button pressed; teleop commands suppressed")
            return

        if safe:
            if self.enable_arm and target_positions is not None:
                self._publish_arm_target(target_positions, stamp)
            if self.enable_loco:
                self._publish_loco_command(frame, stamp)
        else:
            self._publish_loco_stop(stamp)

        if self.print_debug:
            loco_debug_msg = self.loco_mapper.from_controller(frame.controller, stamp)
            self._debug_tick(
                frame,
                target_positions,
                safe,
                reasons,
                xr_debug,
                xr_health,
                loco_debug_msg,
            )

    def _current_positions_for_ik(self):
        if not self.arm_state.has_state:
            return None
        by_name = dict(zip(self.arm_state.joint_names, self.arm_state.positions))
        if not all(name in by_name for name in self.arm_joint_names):
            return None
        return [by_name[name] for name in self.arm_joint_names]

    def _current_velocities_for_ik(self):
        if not self.arm_state.has_state or not self.arm_state.velocities:
            return None
        by_name = dict(zip(self.arm_state.joint_names, self.arm_state.velocities))
        if not all(name in by_name for name in self.arm_joint_names):
            return None
        return [by_name[name] for name in self.arm_joint_names]

    def _publish_arm_target(self, positions, stamp):
        if not self.arm_mapper.valid_shape(positions):
            self.get_logger().warn("Arm target shape is invalid; command skipped")
            return
        msg = self.arm_mapper.from_joint_targets(positions, stamp)
        if not self.dry_run:
            self.arm_command_pub.publish(msg)

    def _publish_loco_command(self, frame, stamp):
        msg = self.loco_mapper.from_controller(frame.controller, stamp)
        if not self.dry_run:
            self.loco_command_pub.publish(msg)
        self.last_loco_allowed = True

    def _publish_loco_stop(self, stamp):
        if not self.enable_loco or not self.last_loco_allowed:
            return
        msg = self.loco_mapper.stop_command(stamp)
        if not self.dry_run:
            self.loco_command_pub.publish(msg)
        self.last_loco_allowed = False

    def _log_safety_changes(self, safe, reasons):
        if reasons != self.last_safety_reasons:
            if safe:
                self.get_logger().info("Safety gate open")
            else:
                self.get_logger().warn("Safety gate closed: " + "; ".join(reasons))
            self.last_safety_reasons = list(reasons)

    def _debug_tick(
        self,
        frame,
        target_positions,
        safe,
        reasons,
        xr_debug,
        xr_health,
        loco_debug_msg,
    ):
        self.debug_tick_count += 1
        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        if self.debug_tick_count % max(1, int(rate)) != 0:
            return
        head_pose = self._pose_summary(frame.head_pose)
        left_wrist_pose = self._pose_summary(frame.left_wrist_pose)
        right_wrist_pose = self._pose_summary(frame.right_wrist_pose)
        self.get_logger().info(
            "debug "
            f"ready={frame.motion_ready} safe={safe} "
            f"lowstate={self.status.lowstate_online} "
            f"arm_state={self.arm_state.has_state} "
            f"target={'yes' if target_positions is not None else 'no'} "
            f"left_trigger={frame.controller.left_trigger:.3f} "
            f"right_trigger={frame.controller.right_trigger:.3f} "
            f"left_squeeze={frame.controller.left_squeeze:.3f} "
            f"right_squeeze={frame.controller.right_squeeze:.3f} "
            f"left_stick={[round(v, 3) for v in frame.controller.left_thumbstick]} "
            f"right_stick={[round(v, 3) for v in frame.controller.right_thumbstick]} "
            f"left_stick_pressed={frame.controller.left_thumbstick_pressed} "
            f"right_stick_pressed={frame.controller.right_thumbstick_pressed} "
            f"right_a_button={frame.controller.right_a_button} "
            f"loco_cmd=[vx={loco_debug_msg.vx:.3f},vy={loco_debug_msg.vy:.3f},omega={loco_debug_msg.omega:.3f}] "
            f"head={head_pose} "
            f"left_wrist={left_wrist_pose} "
            f"right_wrist={right_wrist_pose} "
            f"xr_health={xr_health} "
            f"xr_events={xr_debug} "
            f"reasons={reasons}"
        )

    def _update_xr_health(self, frame, xr_debug):
        if not self.get_parameter("enable_xr").value or not xr_debug:
            self.xr_health = {
                "stuck": False,
                "camera_stale": False,
                "controller_stale": False,
                "hand_stale": False,
                "head_pose_stale": False,
                "controller_value_stale": False,
                "camera_delta": 0,
                "controller_delta": 0,
                "hand_delta": 0,
                "camera_age_sec": None,
                "controller_age_sec": None,
                "hand_age_sec": None,
                "head_pose_change_age_sec": None,
                "controller_value_change_age_sec": None,
            }
            return self.xr_health

        now_sec = self._now_sec()
        deltas = {}
        ages = {}
        stale = {}
        for key, prefix in (
            ("camera_events", "camera"),
            ("controller_events", "controller"),
            ("hand_events", "hand"),
        ):
            current = int(xr_debug.get(key, -1))
            previous = self.xr_event_counts[key]
            if current < 0:
                delta = 0
            elif previous is None:
                delta = 0
                self.xr_event_times[key] = now_sec
            else:
                delta = max(0, current - previous)
                if delta > 0:
                    self.xr_event_times[key] = now_sec
            if current >= 0:
                self.xr_event_counts[key] = current

            last_time = self.xr_event_times[key]
            age = None if last_time is None else max(0.0, now_sec - last_time)
            active_stream = key != "hand_events" or self.get_parameter("input_mode").value == "hand"
            is_stale = (
                active_stream
                and bool(frame.motion_ready)
                and current >= 0
                and age is not None
                and age > self.xr_stuck_timeout_sec
            )
            deltas[f"{prefix}_delta"] = delta
            ages[f"{prefix}_age_sec"] = self._round_or_none(age)
            stale[f"{prefix}_stale"] = is_stale

        value_health = self._update_xr_value_health(frame, now_sec)
        stuck = bool(
            frame.motion_ready
            and value_health["head_pose_stale"]
            and value_health["controller_value_stale"]
        )
        previous_stuck = bool(self.xr_health.get("stuck", False))
        self.xr_health = {
            "stuck": stuck,
            **stale,
            **deltas,
            **ages,
            **value_health,
        }
        if stuck and not previous_stuck:
            self.get_logger().warn(
                "XR data appears stuck; adjust or re-wear the headset "
                f"(health={self.xr_health})"
            )
        elif previous_stuck and not stuck:
            self.get_logger().info("XR data recovered")
        return self.xr_health

    def _update_xr_value_health(self, frame, now_sec):
        signatures = {
            "head_pose": self._pose_signature(frame.head_pose),
            "controller": self._controller_signature(frame.controller),
        }
        result = {}
        for key, prefix in (
            ("head_pose", "head_pose"),
            ("controller", "controller_value"),
        ):
            signature = signatures[key]
            previous = self.xr_value_signatures[key]
            if signature is None:
                age = None
            else:
                if previous != signature:
                    self.xr_value_signatures[key] = signature
                    self.xr_value_times[key] = now_sec
                elif previous is None:
                    self.xr_value_times[key] = now_sec
                age = (
                    None
                    if self.xr_value_times[key] is None
                    else max(0.0, now_sec - self.xr_value_times[key])
                )
            result[f"{prefix}_stale"] = (
                bool(frame.motion_ready)
                and age is not None
                and age > self.xr_stuck_timeout_sec
            )
            result[f"{prefix}_change_age_sec"] = self._round_or_none(age)
        return result

    def _pose_signature(self, pose):
        if pose is None:
            return None
        try:
            values = []
            for row in range(3):
                for col in range(4):
                    values.append(round(float(pose[row][col]), 3))
            return tuple(values)
        except Exception:
            return None

    @staticmethod
    def _controller_signature(controller):
        return (
            round(float(controller.left_trigger), 3),
            round(float(controller.right_trigger), 3),
            round(float(controller.left_squeeze), 3),
            round(float(controller.right_squeeze), 3),
            round(float(controller.left_thumbstick[0]), 3),
            round(float(controller.left_thumbstick[1]), 3),
            round(float(controller.right_thumbstick[0]), 3),
            round(float(controller.right_thumbstick[1]), 3),
            bool(controller.left_thumbstick_pressed),
            bool(controller.right_thumbstick_pressed),
            bool(controller.right_a_button),
        )

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    @staticmethod
    def _round_or_none(value):
        if value is None:
            return None
        return round(float(value), 3)

    def _pose_summary(self, pose):
        if pose is None:
            return "none"
        try:
            pos = [round(float(pose[i][3]), 3) for i in range(3)]
            rpy = [round(value, 3) for value in self._matrix_to_rpy(pose)]
            return f"pos={pos},rpy={rpy}"
        except Exception:
            return "invalid"

    @staticmethod
    def _matrix_to_rpy(pose):
        r00 = float(pose[0][0])
        r10 = float(pose[1][0])
        r20 = float(pose[2][0])
        r21 = float(pose[2][1])
        r22 = float(pose[2][2])
        r11 = float(pose[1][1])
        r12 = float(pose[1][2])

        pitch = math.asin(max(-1.0, min(1.0, -r20)))
        cp = math.cos(pitch)
        if abs(cp) > 1e-6:
            roll = math.atan2(r21, r22)
            yaw = math.atan2(r10, r00)
        else:
            roll = math.atan2(-r12, r11)
            yaw = 0.0
        return [roll, pitch, yaw]


def main(args=None):
    rclpy.init(args=args)
    node = G1XRTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
