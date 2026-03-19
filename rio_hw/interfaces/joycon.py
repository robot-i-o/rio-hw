import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request

try:
    import pyjoycon
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        pyjoycon = None  # type: ignore

JOYCON_KEYS = [
    ("left", "up"),  # 0
    ("left", "down"),  # 1
    ("left", "left"),  # 2
    ("left", "right"),  # 3
    ("left", "l"),  # 4
    ("left", "zl"),  # 5
    ("shared", "minus"),  # 6
    ("shared", "capture"),  # 7
    ("shared", "l-stick"),  # 8
    ("right", "a"),  # 9
    ("right", "b"),  # 10
    ("right", "x"),  # 11
    ("right", "y"),  # 12
    ("right", "r"),  # 13
    ("right", "zr"),  # 14
    ("shared", "plus"),  # 15
    ("shared", "home"),  # 16
    ("shared", "r-stick"),  # 17
]


class RequestType(Enum):
    RUMBLE = auto()
    RUMBLE_STOP = auto()


class Joycon(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "is_button_pressed",
        "rumble",
        "rumble_stop",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        deadzone=0.1,
        stick_center: int = 2048,
        stick_range: int = 4095,
        dtype=np.float32,
        *,
        freq: int = 200,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        if np.issubdtype(type(deadzone), np.number):
            deadzone = np.full(4, fill_value=deadzone, dtype=dtype)
        else:
            deadzone = np.array(deadzone, dtype=dtype)
        assert deadzone.shape == (4,) and (deadzone >= 0).all()
        self.deadzone = deadzone
        self.stick_center = stick_center
        self.half_range = stick_range / 2
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_request = {
            "type": next(iter(RequestType)).value,
            "side": "left",
            "low_freq": 320.0,
            "high_freq": 160.0,
            "amplitude": 0.5,
        }
        self.example_data = {
            "left_stick": np.zeros(2, dtype=self.dtype),
            "right_stick": np.zeros(2, dtype=self.dtype),
            "button_state": np.zeros(18, dtype=bool),
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pubreq
        super().__post_init__()

    def pubreq(self):
        jc_left = None
        jc_right = None
        left_id = pyjoycon.get_L_id()
        right_id = pyjoycon.get_R_id()
        if left_id[0] is not None:
            jc_left = pyjoycon.rumble.RumbleJoyCon(*left_id)
            print(f"Connected to Left JoyCon (serial={left_id[2]})")
        else:
            print("Left JoyCon not found — left stick and buttons will be zeroed.")
        if right_id[0] is not None:
            jc_right = pyjoycon.rumble.RumbleJoyCon(*right_id)
            print(f"Connected to Right JoyCon (serial={right_id[2]})")
        else:
            print("Right JoyCon not found — right stick and buttons will be zeroed.")
        if jc_left is None and jc_right is None:
            raise RuntimeError("No JoyCons found. Pair at least one via Bluetooth.")

        try:
            left_stick = np.zeros(2, dtype=self.dtype)
            right_stick = np.zeros(2, dtype=self.dtype)
            button_state = np.zeros(18, dtype=bool)

            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                if jc_left is not None:
                    status = jc_left.get_status()
                    sticks = status["analog-sticks"]["left"]
                    raw = np.array([sticks["horizontal"], sticks["vertical"]], dtype=self.dtype)
                    left_stick = self._normalize_stick(raw, self.deadzone[:2])
                    buttons = status["buttons"]
                    for i, (group, key) in enumerate(JOYCON_KEYS[:9]):
                        button_state[i] = buttons[group][key]

                if jc_right is not None:
                    status = jc_right.get_status()
                    sticks = status["analog-sticks"]["right"]
                    raw = np.array([sticks["horizontal"], sticks["vertical"]], dtype=self.dtype)
                    right_stick = self._normalize_stick(raw, self.deadzone[2:])
                    buttons = status["buttons"]
                    for i, (group, key) in enumerate(JOYCON_KEYS[9:], start=9):
                        button_state[i] = buttons[group][key]

                # Store current state in ring buffer
                data = {
                    "left_stick": np.copy(left_stick),
                    "right_stick": np.copy(right_stick),
                    "button_state": np.copy(button_state),
                    "timestamp": time.now(),
                }
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False

                # Fetch requests from queue
                try:
                    reqs = self.request_queue.get_all()
                    if isinstance(reqs, dict):
                        reqs = [{k: reqs[k][i] for k in reqs} for i in range(len(reqs["type"]))]
                except queue.Empty:
                    reqs = []
                for r in reqs:
                    req = Request(RequestType(r.pop("type")), r)
                    jc = jc_left if req.data.get("side") == "left" else jc_right
                    if jc is None:
                        continue
                    if req.type == RequestType.RUMBLE:
                        jc.enable_vibration(True)
                        rumble_data = pyjoycon.rumble.RumbleData(
                            req.data["low_freq"],
                            req.data["high_freq"],
                            req.data["amplitude"],
                        ).GetData()
                        jc._send_rumble(rumble_data)
                    elif req.type == RequestType.RUMBLE_STOP:
                        jc.enable_vibration(False)
                    else:
                        raise ValueError(req.type)
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            if jc_left is not None:
                jc_left.disconnect_device()
            if jc_right is not None:
                jc_right.disconnect_device()

    def _normalize_stick(self, raw, deadzone):
        state = np.clip((raw - self.stick_center) / self.half_range, -1.0, 1.0).astype(self.dtype)
        is_dead = (-deadzone < state) & (state < deadzone)
        state[is_dead] = 0
        return state

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def is_button_pressed(self, button_id):
        if 0 <= button_id < 18:
            return self.ring_buffer.get()["button_state"][button_id]
        return False

    def rumble(self, side, low_freq, high_freq, amplitude):
        assert side in ("left", "right")
        req = {
            "type": RequestType.RUMBLE.value,
            "side": side,
            "low_freq": low_freq,
            "high_freq": high_freq,
            "amplitude": amplitude,
        }
        self.request_queue.put(req)

    def rumble_stop(self, side):
        assert side in ("left", "right")
        req = {
            "type": RequestType.RUMBLE_STOP.value,
            "side": side,
        }
        self.request_queue.put(req)


def JoyconServer(mw, *args, **kwargs):
    return ServerFactory(mw, Joycon, *args, **kwargs)


def JoyconClient(mw, *args, **kwargs):
    return ClientFactory(mw, Joycon, *args, **kwargs)
