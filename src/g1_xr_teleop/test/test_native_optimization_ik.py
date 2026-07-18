import numpy as np

from g1_xr_teleop.native_optimization_ik import NativeOptimizationIK


def make_ik():
    return NativeOptimizationIK(
        "G1ARM5",
        max_output_step_rad=0.0,
        filter_weights=(1.0,),
    )


def test_g1arm5_reduced_model_joint_order():
    ik = make_ik()
    assert ik.model.nq == 10
    assert ik.joint_names == [
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
    ]


def test_reachable_dual_arm_target_reduces_pose_error():
    ik = make_ik()
    current = np.zeros(ik.model.nq)
    goal = np.array(
        [0.15, 0.10, -0.10, -0.25, 0.08, -0.15, -0.10, 0.10, -0.25, -0.08]
    )
    left_target, right_target = ik.forward_kinematics(goal)
    initial = ik.pose_errors(current, left_target, right_target)

    solution, _ = ik.solve_ik(left_target, right_target, current)
    final = ik.pose_errors(solution, left_target, right_target)

    assert ik.diagnostics.accepted
    assert max(final["left_position_m"], final["right_position_m"]) < 0.02
    assert max(final["left_rotation_rad"], final["right_rotation_rad"]) < 0.08
    assert final["left_position_m"] < initial["left_position_m"]
    assert final["right_position_m"] < initial["right_position_m"]
    assert np.all(solution >= ik.lower)
    assert np.all(solution <= ik.upper)


def test_output_step_is_limited():
    ik = NativeOptimizationIK(
        "G1ARM5",
        max_output_step_rad=0.03,
        filter_weights=(1.0,),
    )
    current = np.zeros(ik.model.nq)
    goal = np.full(ik.model.nq, 0.2)
    left_target, right_target = ik.forward_kinematics(goal)

    solution, _ = ik.solve_ik(left_target, right_target, current)

    assert ik.diagnostics.accepted
    assert np.max(np.abs(solution - current)) <= 0.0300001


def test_step_limiter_scales_the_whole_joint_vector_uniformly():
    ik = NativeOptimizationIK(
        "G1ARM5",
        max_output_step_rad=0.1,
        filter_weights=(1.0,),
    )
    current = np.zeros(ik.model.nq)
    candidate = np.array([0.2, -0.1, 0.05, 0.0, 0.02] * 2)

    limited = ik._limit_step(candidate, current)

    assert np.allclose(limited, candidate * 0.5)
    assert np.max(np.abs(limited)) == 0.1


def test_invalid_target_returns_current_joint_state():
    ik = make_ik()
    current = np.linspace(-0.1, 0.1, ik.model.nq)
    invalid_pose = np.full((4, 4), np.nan)
    valid_pose, _ = ik.forward_kinematics(current)

    solution, effort = ik.solve_ik(invalid_pose, valid_pose, current)

    assert np.array_equal(solution, current)
    assert np.array_equal(effort, np.zeros(ik.model.nv))
    assert not ik.diagnostics.accepted
    assert ik.diagnostics.status == "invalid_target_pose"
