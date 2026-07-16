class CalibrationState:
    """Placeholder for XR-to-robot calibration state.

    The first scaffold keeps wrist poses in the coordinate convention returned
    by the Unitree XR adapter. Explicit calibration transforms will be added
    once real XR frames are available for inspection.
    """

    def __init__(self):
        self.active = False

    def apply_wrist_pose(self, wrist_pose):
        return wrist_pose
