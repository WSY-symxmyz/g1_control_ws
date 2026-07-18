from dataclasses import asdict, dataclass
from pathlib import Path
import time

import numpy as np


@dataclass(frozen=True)
class IKModelConfig:
    urdf_name: str
    locked_joints: tuple
    left_ee_parent: str
    right_ee_parent: str
    ee_offset_m: float
    rotation_weight: float


@dataclass
class IKDiagnostics:
    success: bool = False
    accepted: bool = False
    status: str = "not_run"
    elapsed_ms: float = 0.0
    cost: float = 0.0
    initial_cost: float = 0.0
    left_position_error_m: float = 0.0
    right_position_error_m: float = 0.0
    left_rotation_error_rad: float = 0.0
    right_rotation_error_rad: float = 0.0
    solver_cost: float = 0.0
    solver_left_position_error_m: float = 0.0
    solver_right_position_error_m: float = 0.0
    solver_left_rotation_error_rad: float = 0.0
    solver_right_rotation_error_rad: float = 0.0
    raw_max_joint_step_rad: float = 0.0
    output_max_joint_step_rad: float = 0.0
    nfev: int = 0
    njev: int = 0

    def snapshot(self):
        result = asdict(self)
        for key, value in result.items():
            if isinstance(value, float):
                result[key] = round(value, 5)
        return result


_LEGS = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)

_G1_HANDS = (
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
)

_MODEL_CONFIGS = {
    "G1ARM5": IKModelConfig(
        urdf_name="g1_body23.urdf",
        locked_joints=_LEGS + ("waist_yaw_joint",),
        left_ee_parent="left_wrist_roll_joint",
        right_ee_parent="right_wrist_roll_joint",
        ee_offset_m=0.20,
        rotation_weight=0.5,
    ),
    "G1ARM7": IKModelConfig(
        urdf_name="g1_body29_hand14.urdf",
        locked_joints=_LEGS
        + ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint")
        + _G1_HANDS,
        left_ee_parent="left_wrist_yaw_joint",
        right_ee_parent="right_wrist_yaw_joint",
        ee_offset_m=0.05,
        rotation_weight=1.0,
    ),
}


class NativeOptimizationIK:
    """Unitree-style bounded dual-arm IK using numeric Pinocchio kinematics."""

    def __init__(
        self,
        arm_model,
        position_weight=50.0,
        regularization_weight=0.02,
        smooth_weight=0.1,
        max_nfev=30,
        ftol=1e-4,
        xtol=1e-4,
        gtol=1e-4,
        max_output_step_rad=0.10,
        max_acceptable_position_error_m=0.30,
        max_acceptable_rotation_error_rad=2.0,
        filter_weights=(0.4, 0.3, 0.2, 0.1),
    ):
        try:
            import pinocchio as pin
            from scipy.optimize import least_squares
        except Exception as exc:
            raise RuntimeError(
                "Native optimization IK requires compatible pinocchio and scipy "
                "installations in the ROS Python environment"
            ) from exc

        if arm_model not in _MODEL_CONFIGS:
            raise ValueError(f"Unsupported arm model for IK: {arm_model}")

        self.pin = pin
        self.least_squares = least_squares
        self.arm_model = arm_model
        self.config = _MODEL_CONFIGS[arm_model]
        self.position_weight = float(position_weight)
        self.rotation_weight = float(self.config.rotation_weight)
        self.regularization_weight = float(regularization_weight)
        self.smooth_weight = float(smooth_weight)
        self.max_nfev = int(max_nfev)
        self.ftol = float(ftol)
        self.xtol = float(xtol)
        self.gtol = float(gtol)
        self.max_output_step_rad = float(max_output_step_rad)
        self.max_acceptable_position_error_m = float(
            max_acceptable_position_error_m
        )
        self.max_acceptable_rotation_error_rad = float(
            max_acceptable_rotation_error_rad
        )
        self.filter_weights = np.asarray(filter_weights, dtype=float)
        self._validate_options()

        self.robot, self.reduced_robot = self._build_robot()
        self.model = self.reduced_robot.model
        self.data = self.model.createData()
        self.left_frame_id = self.model.getFrameId("L_ee")
        self.right_frame_id = self.model.getFrameId("R_ee")
        self.lower = np.asarray(self.model.lowerPositionLimit, dtype=float)
        self.upper = np.asarray(self.model.upperPositionLimit, dtype=float)
        self.last_solution = np.zeros(self.model.nq, dtype=float)
        self._filter_queue = []
        self.diagnostics = IKDiagnostics()

    @property
    def joint_names(self):
        return list(self.model.names[1:])

    def solve_ik(
        self,
        left_wrist,
        right_wrist,
        current_lr_arm_motor_q=None,
        current_lr_arm_motor_dq=None,
    ):
        del current_lr_arm_motor_dq
        started = time.perf_counter()
        current_q = self._validated_current_q(current_lr_arm_motor_q)
        if current_q is None:
            return self._fallback(
                None, started, "invalid_current_joint_state", "current q is invalid"
            )

        try:
            left_target = self._validated_pose(left_wrist)
            right_target = self._validated_pose(right_wrist)
        except ValueError as exc:
            return self._fallback(current_q, started, "invalid_target_pose", str(exc))

        initial_residual = self._residual(current_q, left_target, right_target, current_q)
        initial_cost = float(np.dot(initial_residual, initial_residual))

        try:
            result = self.least_squares(
                self._residual,
                current_q,
                args=(left_target, right_target, current_q),
                bounds=(self.lower, self.upper),
                method="trf",
                jac="2-point",
                x_scale="jac",
                max_nfev=self.max_nfev,
                ftol=self.ftol,
                xtol=self.xtol,
                gtol=self.gtol,
            )
        except Exception as exc:
            return self._fallback(current_q, started, "solver_exception", str(exc))

        candidate = np.asarray(result.x, dtype=float)
        solver_errors = self.pose_errors(candidate, left_target, right_target)
        solver_cost = float(2.0 * result.cost)
        raw_max_step = float(np.max(np.abs(candidate - current_q)))
        finite = bool(np.all(np.isfinite(candidate)))
        improved = solver_cost <= initial_cost + 1e-9
        within_error_limit = (
            max(
                solver_errors["left_position_m"],
                solver_errors["right_position_m"],
            )
            <= self.max_acceptable_position_error_m
            and max(
                solver_errors["left_rotation_rad"],
                solver_errors["right_rotation_rad"],
            )
            <= self.max_acceptable_rotation_error_rad
        )
        accepted = finite and improved and within_error_limit

        if not accepted:
            self.diagnostics = IKDiagnostics(
                success=bool(result.success),
                accepted=False,
                status="rejected_solution",
                elapsed_ms=(time.perf_counter() - started) * 1000.0,
                cost=solver_cost,
                initial_cost=initial_cost,
                left_position_error_m=solver_errors["left_position_m"],
                right_position_error_m=solver_errors["right_position_m"],
                left_rotation_error_rad=solver_errors["left_rotation_rad"],
                right_rotation_error_rad=solver_errors["right_rotation_rad"],
                solver_cost=solver_cost,
                solver_left_position_error_m=solver_errors["left_position_m"],
                solver_right_position_error_m=solver_errors["right_position_m"],
                solver_left_rotation_error_rad=solver_errors["left_rotation_rad"],
                solver_right_rotation_error_rad=solver_errors["right_rotation_rad"],
                raw_max_joint_step_rad=raw_max_step,
                nfev=int(result.nfev),
                njev=int(result.njev or 0),
            )
            self.last_solution = current_q.copy()
            self._filter_queue.clear()
            return current_q.copy(), np.zeros(self.model.nv)

        candidate = self._limit_step(candidate, current_q)
        filtered = self._filter(candidate)
        filtered = np.clip(filtered, self.lower, self.upper)
        output_residual = self._residual(
            filtered, left_target, right_target, current_q
        )
        output_errors = self.pose_errors(filtered, left_target, right_target)
        output_max_step = float(np.max(np.abs(filtered - current_q)))
        self.diagnostics = IKDiagnostics(
            success=bool(result.success),
            accepted=True,
            status="accepted",
            elapsed_ms=(time.perf_counter() - started) * 1000.0,
            cost=float(np.dot(output_residual, output_residual)),
            initial_cost=initial_cost,
            left_position_error_m=output_errors["left_position_m"],
            right_position_error_m=output_errors["right_position_m"],
            left_rotation_error_rad=output_errors["left_rotation_rad"],
            right_rotation_error_rad=output_errors["right_rotation_rad"],
            solver_cost=solver_cost,
            solver_left_position_error_m=solver_errors["left_position_m"],
            solver_right_position_error_m=solver_errors["right_position_m"],
            solver_left_rotation_error_rad=solver_errors["left_rotation_rad"],
            solver_right_rotation_error_rad=solver_errors["right_rotation_rad"],
            raw_max_joint_step_rad=raw_max_step,
            output_max_joint_step_rad=output_max_step,
            nfev=int(result.nfev),
            njev=int(result.njev or 0),
        )
        self.last_solution = filtered.copy()
        return filtered, np.zeros(self.model.nv)

    def forward_kinematics(self, q):
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.size != self.model.nq:
            raise ValueError(f"Expected {self.model.nq} joints, got {q.size}")
        self.pin.framesForwardKinematics(self.model, self.data, q)
        left = self.data.oMf[self.left_frame_id]
        right = self.data.oMf[self.right_frame_id]
        return self._se3_matrix(left), self._se3_matrix(right)

    def pose_errors(self, q, left_target, right_target):
        left_actual, right_actual = self.forward_kinematics(q)
        return {
            "left_position_m": float(
                np.linalg.norm(left_actual[:3, 3] - left_target[:3, 3])
            ),
            "right_position_m": float(
                np.linalg.norm(right_actual[:3, 3] - right_target[:3, 3])
            ),
            "left_rotation_rad": float(
                np.linalg.norm(
                    self.pin.log3(left_actual[:3, :3] @ left_target[:3, :3].T)
                )
            ),
            "right_rotation_rad": float(
                np.linalg.norm(
                    self.pin.log3(right_actual[:3, :3] @ right_target[:3, :3].T)
                )
            ),
        }

    def diagnostic_snapshot(self):
        return self.diagnostics.snapshot()

    def reset(self, current_q=None):
        self._filter_queue.clear()
        validated = self._validated_current_q(current_q)
        if validated is not None:
            self.last_solution = validated.copy()
        self.diagnostics = IKDiagnostics(status="reset")

    def _build_robot(self):
        asset_dir = Path(__file__).resolve().parent / "assets" / "g1"
        urdf_path = asset_dir / self.config.urdf_name
        if not urdf_path.is_file():
            raise FileNotFoundError(f"G1 IK URDF not found: {urdf_path}")

        robot = self.pin.RobotWrapper.BuildFromURDF(str(urdf_path), str(asset_dir))
        reduced = robot.buildReducedRobot(
            list_of_joints_to_lock=list(self.config.locked_joints),
            reference_configuration=np.zeros(robot.model.nq),
        )
        offset = self.pin.SE3(
            np.eye(3), np.array([self.config.ee_offset_m, 0.0, 0.0])
        )
        reduced.model.addFrame(
            self.pin.Frame(
                "L_ee",
                reduced.model.getJointId(self.config.left_ee_parent),
                offset,
                self.pin.FrameType.OP_FRAME,
            )
        )
        reduced.model.addFrame(
            self.pin.Frame(
                "R_ee",
                reduced.model.getJointId(self.config.right_ee_parent),
                offset,
                self.pin.FrameType.OP_FRAME,
            )
        )
        return robot, reduced

    def _residual(self, q, left_target, right_target, q_last):
        left_actual, right_actual = self.forward_kinematics(q)
        position_error = np.concatenate(
            (
                left_actual[:3, 3] - left_target[:3, 3],
                right_actual[:3, 3] - right_target[:3, 3],
            )
        )
        rotation_error = np.concatenate(
            (
                self.pin.log3(
                    left_actual[:3, :3] @ left_target[:3, :3].T
                ),
                self.pin.log3(
                    right_actual[:3, :3] @ right_target[:3, :3].T
                ),
            )
        )
        return np.concatenate(
            (
                np.sqrt(self.position_weight) * position_error,
                np.sqrt(self.rotation_weight) * rotation_error,
                np.sqrt(self.regularization_weight) * np.asarray(q),
                np.sqrt(self.smooth_weight) * (np.asarray(q) - q_last),
            )
        )

    def _validated_current_q(self, q):
        if q is None:
            return None
        value = np.asarray(q, dtype=float).reshape(-1)
        if value.size != self.model.nq or not np.all(np.isfinite(value)):
            return None
        return np.clip(value, self.lower, self.upper)

    @staticmethod
    def _validated_pose(pose):
        value = np.asarray(pose, dtype=float)
        if value.shape != (4, 4) or not np.all(np.isfinite(value)):
            raise ValueError("wrist pose must be a finite 4x4 matrix")
        rotation = value[:3, :3]
        if np.linalg.det(rotation) < 0.5:
            raise ValueError("wrist pose rotation has an invalid determinant")
        if np.linalg.norm(rotation.T @ rotation - np.eye(3)) > 0.1:
            raise ValueError("wrist pose rotation is not sufficiently orthonormal")
        u, _, vt = np.linalg.svd(rotation)
        result = value.copy()
        result[:3, :3] = u @ vt
        result[3, :] = [0.0, 0.0, 0.0, 1.0]
        return result

    def _limit_step(self, candidate, current_q):
        if self.max_output_step_rad <= 0.0:
            return candidate
        delta = candidate - current_q
        largest_step = float(np.max(np.abs(delta)))
        motion_scale = largest_step / self.max_output_step_rad
        return current_q + delta / max(motion_scale, 1.0)

    def _filter(self, q):
        self._filter_queue.append(np.asarray(q, dtype=float).copy())
        if len(self._filter_queue) > self.filter_weights.size:
            self._filter_queue.pop(0)
        if len(self._filter_queue) < self.filter_weights.size:
            return self._filter_queue[-1].copy()
        samples = np.asarray(self._filter_queue)
        # Unitree's convolution gives the newest sample the first (largest) weight.
        return np.sum(samples * self.filter_weights[::-1, None], axis=0)

    def _fallback(self, current_q, started, status, message):
        del message
        size = self.model.nq if hasattr(self, "model") else 0
        fallback = (
            np.asarray(current_q, dtype=float).copy()
            if current_q is not None
            else np.zeros(size, dtype=float)
        )
        self.diagnostics = IKDiagnostics(
            success=False,
            accepted=False,
            status=status,
            elapsed_ms=(time.perf_counter() - started) * 1000.0,
        )
        if hasattr(self, "_filter_queue"):
            self._filter_queue.clear()
        return fallback, np.zeros(size, dtype=float)

    def _validate_options(self):
        positive = {
            "position_weight": self.position_weight,
            "rotation_weight": self.rotation_weight,
            "regularization_weight": self.regularization_weight,
            "smooth_weight": self.smooth_weight,
            "ftol": self.ftol,
            "xtol": self.xtol,
            "gtol": self.gtol,
        }
        if any(value <= 0.0 for value in positive.values()):
            raise ValueError(f"IK weights and tolerances must be positive: {positive}")
        if self.max_nfev < 1:
            raise ValueError("max_nfev must be at least 1")
        if self.filter_weights.ndim != 1 or self.filter_weights.size < 1:
            raise ValueError("filter_weights must be a non-empty vector")
        if not np.isclose(np.sum(self.filter_weights), 1.0):
            raise ValueError("filter_weights must sum to 1.0")

    @staticmethod
    def _se3_matrix(transform):
        result = np.eye(4)
        result[:3, :3] = transform.rotation
        result[:3, 3] = transform.translation
        return result
