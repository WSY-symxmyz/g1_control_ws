class ArmIKAdapter:
    def __init__(self, enabled, arm_model, logger):
        self.enabled = bool(enabled)
        self.arm_model = arm_model
        self.logger = logger
        self.ik = None

        if self.enabled:
            self._initialize_ik()

    def solve(self, left_wrist_pose, right_wrist_pose, current_q, current_dq=None):
        if not self.enabled or self.ik is None:
            return None
        if left_wrist_pose is None or right_wrist_pose is None:
            return None
        if current_q is None:
            return None
        sol_q, _sol_tauff = self.ik.solve_ik(
            left_wrist_pose,
            right_wrist_pose,
            current_q,
            current_dq,
        )
        return [float(value) for value in sol_q]

    def _initialize_ik(self):
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
        self.logger.info(f"Arm IK adapter initialized for {self.arm_model}")
