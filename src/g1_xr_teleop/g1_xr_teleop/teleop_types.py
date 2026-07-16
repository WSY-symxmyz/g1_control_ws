from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class ControllerInput:
    left_thumbstick: List[float] = field(default_factory=lambda: [0.0, 0.0])
    right_thumbstick: List[float] = field(default_factory=lambda: [0.0, 0.0])
    left_thumbstick_pressed: bool = False
    right_thumbstick_pressed: bool = False
    left_trigger: float = 0.0
    right_trigger: float = 0.0
    left_squeeze: float = 0.0
    right_squeeze: float = 0.0
    right_a_button: bool = False


@dataclass
class XRFrame:
    motion_ready: bool = False
    head_pose: Optional[object] = None
    left_wrist_pose: Optional[object] = None
    right_wrist_pose: Optional[object] = None
    controller: ControllerInput = field(default_factory=ControllerInput)


@dataclass
class ArmState:
    joint_names: List[str] = field(default_factory=list)
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    has_state: bool = False


@dataclass
class ControllerStatus:
    lowstate_online: bool = False
    arm_control_active: bool = False
    loco_control_active: bool = False
    fsm_id: int = 0
    fsm_mode: int = 0
    arm_model: str = ""
    message: str = ""
