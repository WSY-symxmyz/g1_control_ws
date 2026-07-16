from builtin_interfaces.msg import Duration
from g1_control_msgs.msg import LocoCommand


class LocoCommandMapper:
    def __init__(self, max_vx, max_vy, max_omega, deadzone, duration_sec, continuous):
        self.max_vx = float(max_vx)
        self.max_vy = float(max_vy)
        self.max_omega = float(max_omega)
        self.deadzone = float(deadzone)
        self.duration_sec = float(duration_sec)
        self.continuous = bool(continuous)

    def from_controller(self, controller, stamp):
        msg = LocoCommand()
        msg.header.stamp = stamp
        msg.vx = -self._axis(controller.left_thumbstick[1]) * self.max_vx
        msg.vy = -self._axis(controller.left_thumbstick[0]) * self.max_vy
        msg.omega = -self._axis(controller.right_thumbstick[0]) * self.max_omega
        msg.duration = self._duration(self.duration_sec)
        msg.continuous = self.continuous
        msg.enable = True
        return msg

    def stop_command(self, stamp):
        msg = LocoCommand()
        msg.header.stamp = stamp
        msg.vx = 0.0
        msg.vy = 0.0
        msg.omega = 0.0
        msg.duration = self._duration(0.05)
        msg.continuous = False
        msg.enable = True
        return msg

    def _axis(self, value):
        value = float(value)
        if abs(value) < self.deadzone:
            return 0.0
        return max(-1.0, min(1.0, value))

    @staticmethod
    def _duration(seconds):
        msg = Duration()
        msg.sec = int(seconds)
        msg.nanosec = int((seconds - int(seconds)) * 1_000_000_000)
        return msg
