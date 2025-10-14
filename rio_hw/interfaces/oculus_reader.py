from enum import Enum
from typing import TYPE_CHECKING

import numpy as np

try:
    import oculus_reader
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        oculus_reader = None  # type: ignore

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node


class ButtonInfo(Enum):
    BUTTON_LEFT_TRIG = 0
    BUTTON_RIGHT_TRIG = 1
    BUTTON_LEFT_GRIP = 2
    BUTTON_RIGHT_GRIP = 3
    BUTTON_A = 4
    BUTTON_B = 5
    BUTTON_X = 6
    BUTTON_Y = 7


class OculusReader(Node):
    __api__ = [
        "get_state",
        "get_all_state",
    ]
    __pub__ = True
    __req__ = False

    def __init__(
        self,
        dtype=np.float32,
        *,
        freq: int = 50,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_request = None
        self.example_data = {
            "left_pose_state": np.eye(4, dtype=self.dtype),
            "right_pose_state": np.eye(4, dtype=self.dtype),
            "button_state": np.zeros(8, dtype=self.dtype),  # leftTrig, rightTrig, leftGrip, rightGrip, A, B, X, Y
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        try:
            reader = oculus_reader.OculusReader()

            # Main loop
            rate = time.Rate(self.freq)
            not_pub_ready = True
            while not self.exit_event.is_set():
                pose_data, button_data = reader.get_transformations_and_buttons()
                pose_left, pose_right = np.eye(4, dtype=self.dtype), np.eye(4, dtype=self.dtype)
                if pose_data:
                    pose_left = pose_data["l"]
                    pose_right = pose_data["r"]

                # Button order: [leftTrig, rightTrig, leftGrip, rightGrip, A, B, X, Y]
                buttons = np.zeros(8, dtype=self.dtype)
                if button_data:
                    buttons[ButtonInfo.BUTTON_LEFT_TRIG.value] = button_data["leftTrig"][0]
                    buttons[ButtonInfo.BUTTON_RIGHT_TRIG.value] = button_data["rightTrig"][0]
                    buttons[ButtonInfo.BUTTON_LEFT_GRIP.value] = button_data["leftGrip"][0]
                    buttons[ButtonInfo.BUTTON_RIGHT_GRIP.value] = button_data["rightGrip"][0]
                    buttons[ButtonInfo.BUTTON_A.value] = float(button_data["A"])
                    buttons[ButtonInfo.BUTTON_B.value] = float(button_data["B"])
                    buttons[ButtonInfo.BUTTON_X.value] = float(button_data["X"])
                    buttons[ButtonInfo.BUTTON_Y.value] = float(button_data["Y"])

                # Store current state in ring buffer
                data = {
                    "left_pose_state": np.array(pose_left, dtype=self.dtype),
                    "right_pose_state": np.array(pose_right, dtype=self.dtype),
                    "button_state": buttons,
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
            pass

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()


def OculusReaderServer(mw, *args, **kwargs):
    return ServerFactory(mw, OculusReader, *args, **kwargs)


def OculusReaderClient(mw, *args, **kwargs):
    return ClientFactory(mw, OculusReader, *args, **kwargs)
