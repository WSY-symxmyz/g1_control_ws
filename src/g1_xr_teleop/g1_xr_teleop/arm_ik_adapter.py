class ArmIKAdapter:
    def __init__(self, enabled, arm_model, logger, backend, options=None):
        self.enabled = bool(enabled)
        self.arm_model = arm_model
        self.logger = logger
        self.backend = str(backend)
        self.options = dict(options or {})
        self.ik = None
        self.failure_count = 0

        if self.enabled:
            self._initialize_ik()

    def solve(self, left_wrist_pose, right_wrist_pose, current_q, current_dq=None):
        if not self.enabled or self.ik is None:
            return None
        if left_wrist_pose is None or right_wrist_pose is None:
            return None
        if current_q is None:
            return None
        try:
            sol_q, _sol_tauff = self.ik.solve_ik(
                left_wrist_pose,
                right_wrist_pose,
                current_q,
                current_dq,
            )
        except Exception as exc:
            self.failure_count += 1
            if self.failure_count == 1 or self.failure_count % 30 == 0:
                self.logger.error(f"Arm IK solve failed: {type(exc).__name__}: {exc}")
            return None
        self.failure_count = 0
        return [float(value) for value in sol_q]

    def diagnostic_snapshot(self):
        if self.ik is None or not hasattr(self.ik, "diagnostic_snapshot"):
            return {"backend": self.backend, "status": "unavailable"}
        return {"backend": self.backend, **self.ik.diagnostic_snapshot()}

    def forward_kinematics(self, current_q):
        if not self.enabled or self.ik is None or current_q is None:
            return None, None
        if not hasattr(self.ik, "forward_kinematics"):
            return None, None
        try:
            return self.ik.forward_kinematics(current_q)
        except Exception as exc:
            self.logger.error(
                f"Arm forward kinematics failed: {type(exc).__name__}: {exc}"
            )
            return None, None

    def reset(self, current_q=None):
        if self.ik is not None and hasattr(self.ik, "reset"):
            self.ik.reset(current_q)

    def _initialize_ik(self):
        if self.backend == "native_optimization":
            from g1_xr_teleop.native_optimization_ik import NativeOptimizationIK

            self.ik = NativeOptimizationIK(self.arm_model, **self.options)
            self._validate_joint_count()
            self.logger.info(
                f"Native optimization IK initialized for {self.arm_model} "
                f"({self.ik.model.nq} joints)"
            )
            return

        if self.backend != "unitree_casadi":
            raise ValueError(
                f"Unknown IK backend '{self.backend}'; expected native_optimization "
                "or unitree_casadi"
            )

        try:
            from g1_xr_teleop.third_party.unitree_xr.robot_arm_ik import (
                G1_23_ArmIK,
                G1_29_ArmIK,
            )
        except Exception as exc:
            raise RuntimeError(
                "Failed to import vendored Unitree arm IK. Install IK "
                "dependencies such as pinocchio, casadi, and meshcat first."
            ) from exc

        if self.arm_model == "G1ARM5":
            self.ik = G1_23_ArmIK()
        else:
            self.ik = G1_29_ArmIK()
        self.logger.info(f"Unitree CasADi IK initialized for {self.arm_model}")

    def _validate_joint_count(self):
        expected = 10 if self.arm_model == "G1ARM5" else 14
        if self.ik.model.nq != expected:
            raise RuntimeError(
                f"IK model has {self.ik.model.nq} joints; expected {expected} "
                f"for {self.arm_model}"
            )
