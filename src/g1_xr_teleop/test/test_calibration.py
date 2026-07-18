import math

import numpy as np

from g1_xr_teleop.calibration import CalibrationState


def pose(position=(0.0, 0.0, 0.0), yaw=0.0):
    result = np.eye(4)
    c = math.cos(yaw)
    s = math.sin(yaw)
    result[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    result[:3, 3] = position
    return result


def test_unitree_absolute_passes_through_valid_xr_poses_only_while_engaged():
    state = CalibrationState("unitree_absolute")
    left = pose((0.3, 0.2, 0.1), yaw=0.2)
    right = pose((0.3, -0.2, 0.1), yaw=-0.2)

    output_left, output_right, captured = state.update(
        left, right, None, None, True, True, True
    )

    assert np.allclose(output_left, left)
    assert np.allclose(output_right, right)
    assert not captured
    assert state.snapshot()["state"] == "absolute_tracking"

    output_left, output_right, _ = state.update(
        left, right, None, None, False, True, True
    )
    assert output_left is None
    assert output_right is None
    assert state.snapshot()["state"] == "disarmed"


def test_relative_clutch_captures_current_robot_pose_without_initial_jump():
    state = CalibrationState("relative_clutch")
    left_xr = pose((0.4, 0.25, 0.2), yaw=0.3)
    right_xr = pose((0.4, -0.25, 0.2), yaw=-0.3)
    left_robot = pose((0.25, 0.18, 0.15), yaw=0.1)
    right_robot = pose((0.25, -0.18, 0.15), yaw=-0.1)

    output_left, output_right, captured = state.update(
        left_xr, right_xr, left_robot, right_robot, True, True, True
    )

    assert captured
    assert np.allclose(output_left, left_robot)
    assert np.allclose(output_right, right_robot)
    assert state.snapshot()["capture_count"] == 1


def test_relative_clutch_applies_translation_and_rotation_deltas():
    state = CalibrationState(
        "relative_clutch", position_scale=0.5, rotation_scale=0.5
    )
    xr_anchor = pose((0.4, 0.2, 0.1), yaw=0.2)
    robot_anchor = pose((0.3, 0.1, 0.2), yaw=-0.1)
    state.update(
        xr_anchor, xr_anchor, robot_anchor, robot_anchor, True, True, True
    )

    moved_xr = pose((0.6, 0.1, 0.3), yaw=0.6)
    left, _, captured = state.update(
        moved_xr, moved_xr, robot_anchor, robot_anchor, True, True, True
    )

    assert not captured
    assert np.allclose(left[:3, 3], [0.4, 0.05, 0.3])
    assert np.allclose(left[:3, :3], pose(yaw=0.1)[:3, :3], atol=1e-7)


def test_relative_clutch_releases_and_recaptures_new_robot_anchor():
    state = CalibrationState("relative_clutch")
    xr = pose((0.4, 0.2, 0.1))
    first_robot = pose((0.3, 0.1, 0.2))
    second_robot = pose((0.2, 0.15, 0.25), yaw=0.2)
    state.update(xr, xr, first_robot, first_robot, True, True, True)

    left, right, _ = state.update(
        xr, xr, first_robot, first_robot, False, True, True
    )
    assert left is None and right is None

    left, right, captured = state.update(
        xr, xr, second_robot, second_robot, True, True, True
    )
    assert captured
    assert np.allclose(left, second_robot)
    assert np.allclose(right, second_robot)
    assert state.snapshot()["capture_count"] == 2


def test_unhealthy_stream_disarms_relative_clutch():
    state = CalibrationState("relative_clutch")
    identity = pose()
    state.update(identity, identity, identity, identity, True, True, True)

    left, right, captured = state.update(
        identity, identity, identity, identity, True, True, False
    )

    assert left is None and right is None
    assert not captured
    assert not state.calibrated
    assert state.snapshot()["last_event"] == "xr_stream_unhealthy"
