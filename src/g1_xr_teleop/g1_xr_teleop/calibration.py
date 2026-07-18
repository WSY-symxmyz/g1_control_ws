import math

import numpy as np


class CalibrationState:
    """Map Unitree-normalized XR wrist poses into robot IK targets."""

    MODES = ("unitree_absolute", "relative_clutch")

    def __init__(
        self,
        mode="relative_clutch",
        position_scale=1.0,
        rotation_scale=1.0,
    ):
        self.mode = str(mode)
        self.position_scale = float(position_scale)
        self.rotation_scale = float(rotation_scale)
        if self.mode not in self.MODES:
            raise ValueError(
                f"Unknown arm tracking mode '{self.mode}'; expected one of {self.MODES}"
            )
        if self.position_scale < 0.0 or self.rotation_scale < 0.0:
            raise ValueError("Arm tracking scales must be non-negative")

        self.state = "disarmed"
        self.capture_count = 0
        self.last_event = "initialized"
        self._left_xr_anchor = None
        self._right_xr_anchor = None
        self._left_robot_anchor = None
        self._right_robot_anchor = None

    def update(
        self,
        left_xr_pose,
        right_xr_pose,
        left_robot_pose,
        right_robot_pose,
        engaged,
        motion_ready,
        stream_healthy=True,
    ):
        if not engaged:
            self.reset("deadman_released")
            return None, None, False
        if not motion_ready:
            self.reset("xr_motion_not_ready")
            return None, None, False
        if not stream_healthy:
            self.reset("xr_stream_unhealthy")
            return None, None, False

        left_xr = self._validated_pose(left_xr_pose)
        right_xr = self._validated_pose(right_xr_pose)
        if left_xr is None or right_xr is None:
            self.reset("invalid_xr_pose")
            return None, None, False

        if self.mode == "unitree_absolute":
            self.state = "absolute_tracking"
            self.last_event = "tracking"
            return left_xr, right_xr, False

        left_robot = self._validated_pose(left_robot_pose)
        right_robot = self._validated_pose(right_robot_pose)
        if left_robot is None or right_robot is None:
            self.reset("robot_pose_unavailable")
            return None, None, False

        captured = False
        if not self.calibrated:
            self._left_xr_anchor = left_xr
            self._right_xr_anchor = right_xr
            self._left_robot_anchor = left_robot
            self._right_robot_anchor = right_robot
            self.capture_count += 1
            self.state = "relative_tracking"
            self.last_event = "anchors_captured"
            captured = True

        return (
            self._relative_target(
                left_xr, self._left_xr_anchor, self._left_robot_anchor
            ),
            self._relative_target(
                right_xr, self._right_xr_anchor, self._right_robot_anchor
            ),
            captured,
        )

    @property
    def calibrated(self):
        return self._left_xr_anchor is not None

    def reset(self, event="reset"):
        had_state = self.state != "disarmed" or self.calibrated
        self.state = "disarmed"
        self._left_xr_anchor = None
        self._right_xr_anchor = None
        self._left_robot_anchor = None
        self._right_robot_anchor = None
        if had_state or self.last_event != event:
            self.last_event = event

    def snapshot(self):
        return {
            "mode": self.mode,
            "state": self.state,
            "calibrated": self.calibrated,
            "capture_count": self.capture_count,
            "last_event": self.last_event,
        }

    def _relative_target(self, current_xr, xr_anchor, robot_anchor):
        result = np.eye(4)
        result[:3, 3] = robot_anchor[:3, 3] + self.position_scale * (
            current_xr[:3, 3] - xr_anchor[:3, 3]
        )
        rotation_delta = current_xr[:3, :3] @ xr_anchor[:3, :3].T
        rotation_delta = self._scale_rotation(rotation_delta, self.rotation_scale)
        result[:3, :3] = rotation_delta @ robot_anchor[:3, :3]
        return result

    @staticmethod
    def _validated_pose(pose):
        if pose is None:
            return None
        try:
            value = np.asarray(pose, dtype=float)
        except (TypeError, ValueError):
            return None
        if value.shape != (4, 4) or not np.all(np.isfinite(value)):
            return None
        rotation = value[:3, :3]
        if np.linalg.det(rotation) < 0.5:
            return None
        if np.linalg.norm(rotation.T @ rotation - np.eye(3)) > 0.1:
            return None
        u, _, vt = np.linalg.svd(rotation)
        projected = u @ vt
        if np.linalg.det(projected) < 0.0:
            u[:, -1] *= -1.0
            projected = u @ vt
        result = value.copy()
        result[:3, :3] = projected
        result[3, :] = [0.0, 0.0, 0.0, 1.0]
        return result

    @staticmethod
    def _scale_rotation(rotation, scale):
        if scale == 1.0:
            return rotation
        trace = float(np.trace(rotation))
        angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) * 0.5)))
        if angle < 1e-9 or scale == 0.0:
            return np.eye(3)

        axis_skew = rotation - rotation.T
        axis = np.array(
            [axis_skew[2, 1], axis_skew[0, 2], axis_skew[1, 0]], dtype=float
        )
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-8:
            eigenvalues, eigenvectors = np.linalg.eig(rotation)
            index = int(np.argmin(np.abs(eigenvalues - 1.0)))
            axis = np.real(eigenvectors[:, index])
            axis_norm = np.linalg.norm(axis)
        axis /= axis_norm

        scaled_angle = scale * angle
        skew = np.array(
            [
                [0.0, -axis[2], axis[1]],
                [axis[2], 0.0, -axis[0]],
                [-axis[1], axis[0], 0.0],
            ]
        )
        return (
            np.eye(3)
            + math.sin(scaled_angle) * skew
            + (1.0 - math.cos(scaled_angle)) * (skew @ skew)
        )
