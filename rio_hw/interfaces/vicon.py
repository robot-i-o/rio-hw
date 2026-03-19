from typing import TYPE_CHECKING

import numpy as np
import scipy.spatial.transform as st

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node

try:
    from pyvicon_datastream.tools import ObjectTracker
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        ObjectTracker = None  # type: ignore


class Vicon(Node):
    __api__ = [
        "get_state",
        "get_all_state",
    ]
    __pub__ = True
    __req__ = False

    def __init__(
        self,
        vicon_ip: str = "192.168.1.111",
        vicon_prefix: str = "",
        vicon_object_names: tuple[str, ...] = ("object",),
        dtype=np.float32,
        *,
        freq: int = 100,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        self.vicon_ip = vicon_ip
        self.vicon_prefix = vicon_prefix
        self.vicon_object_names = vicon_object_names
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_request = None
        self.example_data = {
            **{f"{k}_pose": np.zeros((6,), dtype=self.dtype) for k in self.vicon_object_names},
            "timestamp": time.now(),
        }
        self.worker = self.req
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        tracker = ObjectTracker(self.vicon_ip)
        assert tracker.is_connected

        try:
            # Main loop
            rate = time.Rate(self.freq)
            not_pub_ready = True
            while not self.exit_event.is_set():
                state = {}
                for name in self.vicon_object_names:
                    position = tracker.get_position(f"{self.vicon_prefix}{name}")
                    if not position:
                        pose = np.zeros((6,), dtype=self.dtype)
                    else:
                        obj = position[2][0]
                        _, _, x, y, z, roll, pitch, yaw = obj
                        pos = np.array([x, y, z], dtype=self.dtype) / 1000.0  # convert to meters
                        rot = st.Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_rotvec()
                        pose = np.concatenate([pos, rot], dtype=self.dtype)
                    state[f"{name}_pose"] = pose

                # Store current state in ring buffer
                data = {
                    **state,
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


def ViconServer(mw, *args, **kwargs):
    return ServerFactory(mw, Vicon, *args, **kwargs)


def ViconClient(mw, *args, **kwargs):
    return ClientFactory(mw, Vicon, *args, **kwargs)
