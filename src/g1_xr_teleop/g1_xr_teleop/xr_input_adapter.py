from .teleop_types import ControllerInput, XRFrame


class XRInputAdapter:
    def __init__(self, enabled, input_mode, display_mode, img_server_ip, logger):
        self.enabled = bool(enabled)
        self.input_mode = input_mode
        self.display_mode = display_mode
        self.img_server_ip = img_server_ip
        self.logger = logger
        self.wrapper = None

        if self.enabled:
            self._initialize_wrapper()

    def get_frame(self):
        if not self.enabled or self.wrapper is None:
            return XRFrame()

        data = self.wrapper.get_tele_data()
        controller = ControllerInput(
            left_thumbstick=self._as_pair(getattr(data, "left_ctrl_thumbstickValue", [0.0, 0.0])),
            right_thumbstick=self._as_pair(getattr(data, "right_ctrl_thumbstickValue", [0.0, 0.0])),
            left_thumbstick_pressed=bool(getattr(data, "left_ctrl_thumbstick", False)),
            right_thumbstick_pressed=bool(getattr(data, "right_ctrl_thumbstick", False)),
            left_trigger=float(getattr(data, "left_ctrl_triggerValue", 0.0)),
            right_trigger=float(getattr(data, "right_ctrl_triggerValue", 0.0)),
            left_squeeze=float(getattr(data, "left_ctrl_squeezeValue", 0.0)),
            right_squeeze=float(getattr(data, "right_ctrl_squeezeValue", 0.0)),
            right_a_button=bool(getattr(data, "right_ctrl_aButton", False)),
        )
        return XRFrame(
            motion_ready=bool(getattr(data, "motion_data_ready", False)),
            head_pose=getattr(data, "head_pose", None),
            left_wrist_pose=getattr(data, "left_wrist_pose", None),
            right_wrist_pose=getattr(data, "right_wrist_pose", None),
            controller=controller,
        )

    def close(self):
        if self.wrapper is None:
            return
        try:
            if hasattr(self.wrapper, "close"):
                self.wrapper.close()
        except Exception as exc:
            self.logger.warn(f"Failed to close XR input wrapper cleanly: {exc}")
        finally:
            self.wrapper = None

    def debug_snapshot(self):
        if not self.enabled or self.wrapper is None:
            return {}
        tvuer = getattr(self.wrapper, "tvuer", None)
        if tvuer is None:
            return {}
        return {
            "camera_events": int(getattr(tvuer, "camera_event_count", -1)),
            "controller_events": int(getattr(tvuer, "controller_event_count", -1)),
            "hand_events": int(getattr(tvuer, "hand_event_count", -1)),
            "last_event_error": str(getattr(tvuer, "last_event_error", "")),
        }

    def _initialize_wrapper(self):
        try:
            from g1_xr_teleop.third_party.televuer import TeleVuerWrapper
        except Exception as exc:
            raise RuntimeError(
                "Failed to import vendored TeleVuerWrapper. Install its Python "
                "dependencies such as vuer and opencv-python first."
            ) from exc

        self.wrapper = TeleVuerWrapper(
            use_hand_tracking=self.input_mode == "hand",
            binocular=False,
            img_shape=[480, 640],
            display_mode=self.display_mode,
            zmq=False,
            webrtc=False,
            webrtc_url=f"https://{self.img_server_ip}:60001/offer",
            arm_reference_mode="head_yaw",
        )
        self.logger.info("XR input adapter initialized from Unitree TeleVuerWrapper")

    @staticmethod
    def _as_pair(value):
        try:
            return [float(value[0]), float(value[1])]
        except Exception:
            return [0.0, 0.0]
