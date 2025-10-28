import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory

try:
    import evdev
    from evdev import ecodes
except ImportError:
    evdev = None


def find_gamepad():
    """Find first available gamepad device."""
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        capabilities = device.capabilities()
        # Check if device has gamepad-like capabilities (absolute axes and buttons)
        has_abs = ecodes.EV_ABS in capabilities
        has_key = ecodes.EV_KEY in capabilities
        if has_abs and has_key:
            # Check for common gamepad axes
            abs_events = capabilities.get(ecodes.EV_ABS, [])
            abs_codes = [code for code, _ in abs_events]
            if ecodes.ABS_X in abs_codes or ecodes.ABS_Y in abs_codes:
                return device
    raise RuntimeError("No gamepad detected. Plug one in and check /dev/input permissions (e.g., udev/uaccess).")


class Gamepad:
    __api__ = [
        "get_state",
        "get_all_state",
        "get_motion_state_transformed",
        "is_button_pressed",
    ]
    __pub__ = True
    __req__ = False

    def __init__(
        self,
        device_path: str | None = None,
        axes_to_state: dict | None = None,
        button_map: dict | None = None,
        linear_speed_scale: float = 1.0,
        angular_speed_scale: float = 0.75,
        deadzone: tuple = (0.1, 0.1, 0.1, 0.1, 0.1, 0.1),
        dtype=np.float32,
        *,
        freq: int = 200,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        self.device_path = device_path
        self.axes_to_state = axes_to_state or {
            ecodes.ABS_RX: 0,
            ecodes.ABS_RY: 1,
            ecodes.ABS_Y: 2,
            ecodes.ABS_X: 3,
            ecodes.ABS_HAT0X: 4,
            ecodes.ABS_HAT0Y: 5,
        }
        self.button_map = button_map or {
            ecodes.BTN_SOUTH: 0,  # A/Cross
            ecodes.BTN_NORTH: 1,  # Y/Triangle
            ecodes.BTN_WEST: 2,  # X/Square
            ecodes.BTN_EAST: 3,  # B/Circle
            ecodes.BTN_TL: 4,  # L1/LB
            ecodes.BTN_TR: 5,  # R1/RB
            ecodes.BTN_SELECT: 6,  # Select/Share
            ecodes.BTN_START: 7,  # Start/Options
            ecodes.BTN_MODE: 8,  # Home/PS
            ecodes.BTN_THUMBL: 9,  # L3
            ecodes.BTN_THUMBR: 10,  # R3
        }
        self.n_buttons = len(self.button_map)
        self.axis_codes = {ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY}
        self.hat_codes = {ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y}
        self.trigger_codes = {ecodes.ABS_Z, ecodes.ABS_RZ}
        self.linear_speed_scale = linear_speed_scale
        self.angular_speed_scale = angular_speed_scale
        self.deadzone = deadzone
        self.tx_zup_gamepad = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=dtype)
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_request = None
        self.example_data = {
            "motion_state_transformed": np.zeros((6,), dtype=self.dtype),
            "button_state": np.zeros((self.n_buttons,), dtype=self.dtype),
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        gamepad = None
        try:
            # Find gamepad device
            if self.device_path:
                gamepad = evdev.InputDevice(self.device_path)
            else:
                gamepad = find_gamepad()
            capabilities = gamepad.capabilities()
            self._set_axis_ranges(capabilities)

            motion_event = np.zeros((6,), dtype=self.dtype)
            button_state = np.zeros((self.n_buttons,), dtype=self.dtype)
            motion_event_transformed = np.zeros((6,), dtype=self.dtype)
            timestamp = time.now()

            data = {
                "motion_state_transformed": motion_event,
                "button_state": button_state,
                "timestamp": timestamp,
            }
            self.ring_buffer.put(data)

            # Main loop
            rate = time.Rate(self.freq)
            not_pub_ready = True
            while not self.exit_event.is_set():
                try:
                    # read all available events
                    events = []
                    while True:
                        try:
                            event = gamepad.read_one()
                            if event is None:
                                break
                            events.append(event)
                        except BlockingIOError:
                            break
                except Exception:
                    events = []

                for event in events:
                    self._update_motion_state(event, motion_event, button_state)
                motion_event_transformed = self._get_motion_state_transformed(motion_event)

                # Store current state in ring buffer
                data = {
                    "motion_state_transformed": np.copy(motion_event_transformed),
                    "button_state": np.copy(button_state),
                    "timestamp": time.now(),
                }
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            if gamepad is not None:
                gamepad.close()

    def _set_axis_ranges(self, capabilities):
        axis_ranges = {}
        axis_info = dict(capabilities.get(ecodes.EV_ABS, []))
        for code in self.axes_to_state.keys():
            if code in axis_info:
                info = axis_info[code]
                axis_ranges[code] = {"min": info.min, "max": info.max, "center": (info.min + info.max) // 2}
            else:
                if code in self.axis_codes:
                    axis_ranges[code] = {"min": -32768, "max": 32767, "center": 0}
                elif code in self.hat_codes:
                    axis_ranges[code] = {"min": -1, "max": 1, "center": 0}
                elif code in self.trigger_codes:
                    axis_ranges[code] = {"min": 0, "max": 255, "center": 0}
        self.axis_ranges = axis_ranges

    def _normalize_axis_value(self, code, value):
        """Normalize axis value to [-1, 1] or [0, 1] range."""
        range_info = self.axis_ranges.get(code)
        if not range_info:
            return value

        if code in self.axis_codes:
            # Sticks: normalize to [-1, 1]
            center = range_info["center"]
            if value >= center:
                return (value - center) / (range_info["max"] - center)
            else:
                return (value - center) / (center - range_info["min"])
        elif code in self.hat_codes:
            # D-pad: normalize to [-1, 0, 1]
            if range_info["max"] == range_info["min"]:
                return 0
            return (value - range_info["min"]) / (range_info["max"] - range_info["min"]) * 2 - 1
        elif code in self.trigger_codes:
            # Triggers: normalize to [-1, 1]
            if range_info["max"] == range_info["min"]:
                return 0
            return (value - range_info["min"]) / (range_info["max"] - range_info["min"]) * 2 - 1

        return value

    def _update_motion_state(self, event, motion_state, button_state):
        if event.type == ecodes.EV_ABS:
            if event.code in self.axes_to_state:
                idx = self.axes_to_state[event.code]
                motion_state[idx] = self._normalize_axis_value(event.code, event.value)
                if abs(motion_state[idx]) < self.deadzone[idx]:
                    motion_state[idx] = 0.0
        elif event.type == ecodes.EV_KEY and event.code in self.button_map:
            button_state[self.button_map[event.code]] = float(event.value)

    def _get_motion_state_transformed(self, motion_event):
        state = motion_event.copy()
        state[:3] *= self.linear_speed_scale
        state[3:] *= self.angular_speed_scale
        tf_state = np.zeros_like(state)
        tf_state[:3] = self.tx_zup_gamepad @ state[:3]
        tf_state[3:] = self.tx_zup_gamepad @ state[3:]
        return tf_state

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def get_motion_state_transformed(self):
        state = self.ring_buffer.get()
        return state["motion_state_transformed"]

    def is_button_pressed(self, button_id):
        if 0 <= button_id < self.n_buttons:
            state = self.ring_buffer.get()
            return state["button_state"][button_id] > 0
        return False


def GamepadServer(mw, *args, **kwargs):
    return ServerFactory(mw, Gamepad, *args, **kwargs)


def GamepadClient(mw, *args, **kwargs):
    return ClientFactory(mw, Gamepad, *args, **kwargs)
