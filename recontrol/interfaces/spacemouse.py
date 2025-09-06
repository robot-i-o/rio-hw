import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory

try:
    import spnav
except ImportError:
    spnav = None


class Spacemouse:
    __api__ = [
        "get_state",
        "get_all_state",
        "get_motion_state_transformed",
        "is_button_pressed",
    ]

    def __init__(
        self,
        max_value=500,
        deadzone=(0, 0, 0, 0, 0, 0),
        n_buttons=2,
        dtype=np.float32,
        *,
        freq: int = 200,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        """
        Continuously listen to 3D connection space navigator events and update the latest state.

        Args:
            max_value: {300, 500} 300 for wired version and 500 for wireless
            deadzone: [0,1], number or tuple, axis with value lower than this value will stay at 0
            n_buttons:
            dtype:

        front
        z
        ^   _
        |  (O) space mouse
        |
        *----->x right
        y
        """
        if np.issubdtype(type(deadzone), np.number):
            deadzone = np.full(6, fill_value=deadzone, dtype=dtype)
        else:
            deadzone = np.array(deadzone, dtype=dtype)
        assert (deadzone >= 0).all()
        self.max_value = max_value
        self.deadzone = deadzone
        self.n_buttons = n_buttons
        self.tx_zup_spnav = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]], dtype=dtype)
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_request = None
        self.example_data = {
            # 3 translation, 3 rotation
            "motion_state_transformed": np.zeros((6,), dtype=self.dtype),
            # left and right button
            "button_state": np.zeros((self.n_buttons,), dtype=bool),
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        try:
            spnav.spnav_open()

            # Initialize state
            motion_event = np.zeros((7,), dtype=np.int64)
            motion_state_transformed = np.zeros((6,), dtype=self.dtype)
            button_state = np.zeros((self.n_buttons,), dtype=bool)
            timestamp = time.now()
            # Store initial state in ring buffer
            data_frame = {
                "motion_state_transformed": motion_state_transformed,
                "button_state": button_state,
                "timestamp": timestamp,
            }
            self.ring_buffer.put(data_frame)

            # Main loop
            rate = time.Rate(self.freq)
            self.pub_ready_event.set()
            while not self.exit_event.is_set():
                event = spnav.spnav_poll_event()
                timestamp = time.now()
                if isinstance(event, spnav.SpnavMotionEvent):
                    motion_event[:3] = event.translation
                    motion_event[3:6] = event.rotation
                    motion_event[6] = event.period
                elif isinstance(event, spnav.SpnavButtonEvent):
                    if 0 <= event.bnum < self.n_buttons:
                        button_state[event.bnum] = event.press
                motion_state_transformed = self._get_motion_state_transformed(motion_event)

                # Store current state in ring buffer
                data_frame = {
                    "motion_state_transformed": motion_state_transformed,
                    "button_state": np.copy(button_state),
                    "timestamp": timestamp,
                }
                self.ring_buffer.put(data_frame)

                # print(1 / (time.now() - rate.start_time))  # max actual frequency
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            spnav.spnav_close()

    def _get_motion_state(self, motion_event):
        """Get the current motion state normalized by max_value and apply deadzone"""
        state = np.array(motion_event[:6], dtype=self.dtype) / self.max_value
        is_dead = (-self.deadzone < state) & (state < self.deadzone)
        state[is_dead] = 0
        return state

    def _get_motion_state_transformed(self, motion_event):
        """
        Return in right-handed coordinate
        z
        *------>y right
        |   _
        |  (O) space mouse
        v
        x
        back
        """
        state = self._get_motion_state(motion_event)
        tf_state = np.zeros_like(state)
        tf_state[:3] = self.tx_zup_spnav @ state[:3]
        tf_state[3:] = self.tx_zup_spnav @ state[3:]
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
        """Check if a specific button is pressed"""
        if 0 <= button_id < self.n_buttons:
            state = self.ring_buffer.get()
            button_state = state["button_state"]
            return button_state[button_id]
        return False


def SpacemouseServer(mw, *args, **kwargs):
    return ServerFactory(mw, Spacemouse, *args, **kwargs)


def SpacemouseClient(mw, *args, **kwargs):
    return ClientFactory(mw, Spacemouse, *args, **kwargs)
