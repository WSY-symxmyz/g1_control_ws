import math

from builtin_interfaces.msg import Duration
from g1_control_msgs.msg import ArmJointCommand


class ArmCommandMapper:
    def __init__(self, joint_names, duration_sec):
        self.joint_names = list(joint_names)
        self.duration_sec = float(duration_sec)

    def from_joint_targets(self, positions, stamp):
        msg = ArmJointCommand()
        msg.header.stamp = stamp
        msg.joint_names = list(self.joint_names)
        msg.is_delta = False
        msg.positions = [float(value) for value in positions]
        msg.deltas = []
        msg.velocities = []
        msg.kp = []
        msg.kd = []
        msg.duration = self._duration(self.duration_sec)
        msg.hold = True
        return msg

    def valid_shape(self, positions):
        return len(positions) == len(self.joint_names) and all(
            math.isfinite(float(value)) for value in positions
        )

    @staticmethod
    def _duration(seconds):
        msg = Duration()
        msg.sec = int(seconds)
        msg.nanosec = int((seconds - int(seconds)) * 1_000_000_000)
        return msg
