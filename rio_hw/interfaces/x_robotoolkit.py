from typing import TYPE_CHECKING

import numpy as np
import scipy.spatial.transform as st

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node

try:
    import xrobotoolkit_sdk as xrt
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        xrt = None  # type: ignore


class XRobotoolkit(Node):
    __api__ = [
        "get_state",
        "get_all_state",
    ]
    __pub__ = True
    __req__ = False

    def __init__(
        self,
        enable_body_tracking: bool = False,
        *,
        freq: int = 100,
        max_buffer_size: int = 30,
        dtype=np.float32,
        **kwargs,
    ):
        self.dtype = dtype
        self.enable_body_tracking = enable_body_tracking
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        left_controller_data = {
            "left_controller": False,
            "left_pos": np.zeros((3,), dtype=np.float32),
            "left_quat": np.zeros((4,), dtype=np.float32),
            "left_x": False,
            "left_y": False,
            "left_trigger": 0.0,
            "left_grip": 0.0,
            "left_joystick": np.zeros((2,), dtype=np.float32),
        }
        right_controller_data = {
            "right_controller": False,
            "right_pos": np.zeros((3,), dtype=np.float32),
            "right_quat": np.zeros((4,), dtype=np.float32),
            "right_a": False,
            "right_b": False,
            "right_trigger": 0.0,
            "right_grip": 0.0,
            "right_joystick": np.zeros((2,), dtype=np.float32),
        }
        headset_data = {
            "headset": False,
            "headset_pos": np.zeros((3,), dtype=np.float32),
            "headset_quat": np.zeros((4,), dtype=np.float32),
            "timestamp": time.now(),
        }
        if self.enable_body_tracking:
            body_data = {
                "body_tracking": False,
                "body_pos": np.zeros((24, 3), dtype=np.float32),
                "body_quat": np.zeros((24, 4), dtype=np.float32),
            }
        else:
            body_data = {}

        self.example_request = None
        self.example_data = {**left_controller_data, **right_controller_data, **headset_data, **body_data}
        self.worker = None
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        xrt.init()
        try:
            # Main loop
            rate = time.Rate(self.freq)
            not_pub_ready = True
            while not self.exit_event.is_set():
                state = self._get_headset_state()
                # Store current state in ring buffer
                data = {**state, "timestamp": time.now()}
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            pass

    def _get_headset_state(self):
        left_pose = xrt.get_left_controller_pose()
        left_pos, left_quat = self._convert_pose(left_pose)
        left_x = xrt.get_X_button()
        left_y = xrt.get_Y_button()
        left_trigger = xrt.get_left_trigger()
        left_grip = xrt.get_left_grip()
        left_axis_pos = xrt.get_left_axis()
        left_controller_data = {
            "left_controller": True,
            "left_pos": left_pos,
            "left_quat": left_quat,
            "left_x": left_x,
            "left_y": left_y,
            "left_trigger": left_trigger,
            "left_grip": left_grip,
            "left_joystick": left_axis_pos,
        }

        right_pose = xrt.get_right_controller_pose()
        right_pos, right_quat = self._convert_pose(right_pose)
        right_a = xrt.get_A_button()
        right_b = xrt.get_B_button()
        right_trigger = xrt.get_right_trigger()
        right_grip = xrt.get_right_grip()
        right_axis_pos = xrt.get_right_axis()
        right_controller_data = {
            "right_controller": True,
            "right_pos": right_pos,
            "right_quat": right_quat,
            "right_a": right_a,
            "right_b": right_b,
            "right_trigger": right_trigger,
            "right_grip": right_grip,
            "right_joystick": right_axis_pos,
        }

        headset_pose = xrt.get_headset_pose()
        headset_pos, headset_quat = self._convert_pose(headset_pose)
        headset_data = {"headset": True, "headset_pos": headset_pos, "headset_quat": headset_quat}

        if self.enable_body_tracking:
            if xrt.is_body_data_available():
                body_joints_pose = xrt.get_body_joints_pose()
                body_pos, body_quat = self._convert_pose(body_joints_pose)
                body_data = {"body_tracking": True, "body_pos": body_pos, "body_quat": body_quat}
            else:
                body_data = self.body_data
        else:
            body_data = {}
        full_body_data = {**left_controller_data, **right_controller_data, **headset_data, **body_data}
        return full_body_data

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def _convert_pose(self, pose: np.array):
        pos = np.array(pose)[..., :3]
        quat = np.array(pose)[..., 3:]

        # transformation from unity coordinate to right-hand coordinate system
        coordinate_transform = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        pos = coordinate_transform @ pos
        # try to convert from unity coordinate frame
        if np.allclose(quat, 0):
            rot = st.Rotation.identity().as_matrix()
        else:
            rot = st.Rotation.from_quat(quat).as_matrix()
        quat = st.Rotation.from_matrix(coordinate_transform @ rot @ coordinate_transform.T).as_quat()
        return pos, quat


def XRobotoolkitServer(mw, *args, **kwargs):
    return ServerFactory(mw, XRobotoolkit, *args, **kwargs)


def XRobotoolkitClient(mw, *args, **kwargs):
    return ClientFactory(mw, XRobotoolkit, *args, **kwargs)
