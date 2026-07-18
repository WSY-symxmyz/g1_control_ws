import math


class SafetyGate:
    def __init__(self, require_deadman=True, deadman_source="right_trigger",
                 deadman_threshold=0.2, max_abs_joint_step_rad=0.2):
        self.require_deadman = bool(require_deadman)
        self.deadman_source = str(deadman_source)
        self.deadman_threshold = float(deadman_threshold)
        self.max_abs_joint_step_rad = float(max_abs_joint_step_rad)
        self.last_loco_allowed = False

    def evaluate(self, status, frame, current_positions=None, target_positions=None):
        reasons = []
        if status is None:
            reasons.append("controller status unavailable")
        elif not status.lowstate_online:
            reasons.append("lowstate offline")

        if frame is None:
            reasons.append("XR frame unavailable")
        elif not frame.motion_ready:
            reasons.append("XR motion data not ready")

        if self.require_deadman and frame is not None:
            if not self._deadman_pressed(frame.controller):
                reasons.append("deadman not pressed")

        if target_positions is not None:
            if any(not math.isfinite(float(value)) for value in target_positions):
                reasons.append("arm target contains non-finite value")
            if current_positions and len(current_positions) == len(target_positions):
                largest_step = max(
                    abs(float(target) - float(current))
                    for target, current in zip(target_positions, current_positions)
                )
                if largest_step > self.max_abs_joint_step_rad:
                    reasons.append(
                        f"arm target jump {largest_step:.3f} rad exceeds limit"
                    )

        return len(reasons) == 0, reasons

    def deadman_pressed(self, controller):
        if not self.require_deadman:
            return True
        if controller is None:
            return False
        return self._deadman_pressed(controller)

    def _deadman_pressed(self, controller):
        if self.deadman_source == "right_trigger":
            return self._trigger_pressed(controller.right_trigger)
        if self.deadman_source == "left_trigger":
            return self._trigger_pressed(controller.left_trigger)
        if self.deadman_source == "right_squeeze":
            return controller.right_squeeze >= self.deadman_threshold
        if self.deadman_source == "left_squeeze":
            return controller.left_squeeze >= self.deadman_threshold
        if self.deadman_source == "right_thumbstick":
            return controller.right_thumbstick_pressed
        if self.deadman_source == "left_thumbstick":
            return controller.left_thumbstick_pressed
        return False

    def _trigger_pressed(self, value):
        value = float(value)
        # Unitree TeleVuer reports trigger depth as 10.0 when released and 0.0
        # when fully pressed. Keep the safety decision aligned with that
        # convention because this package currently consumes TeleVuer output.
        normalized = max(0.0, min(1.0, (10.0 - value) / 10.0))
        return normalized >= self.deadman_threshold
