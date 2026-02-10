from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node

try:
    import avp_stream
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        avp_stream = None  # type: ignore


class AvpStream(Node):
    __api__ = [
        "get_state",
        "get_all_state",
    ]
    __pub__ = True
    __req__ = False

    def __init__(
        self,
        avp_ip: str = "192.168.1.111",
        dtype=np.float32,
        *,
        freq: int = 50,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        self.avp_ip = avp_ip
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_avp_state = {
            "head": np.zeros((1, 4, 4), dtype=self.dtype),
            "right_wrist": np.zeros((1, 4, 4), dtype=self.dtype),
            "left_wrist": np.zeros((1, 4, 4), dtype=self.dtype),
            "right_fingers": np.zeros((25, 4, 4), dtype=self.dtype),
            "left_fingers": np.zeros((25, 4, 4), dtype=self.dtype),
            "right_pinch_distance": np.zeros((1,), dtype=self.dtype),
            "left_pinch_distance": np.zeros((1,), dtype=self.dtype),
            "right_wrist_roll": np.zeros((1,), dtype=self.dtype),
            "left_wrist_roll": np.zeros((1,), dtype=self.dtype),
        }

        self.example_request = None
        self.example_data = {
            **example_avp_state,
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        avp = avp_stream.VisionProStreamer(ip=self.avp_ip, record=False)
        try:
            # Main loop
            rate = time.Rate(self.freq)
            not_pub_ready = True
            while not self.exit_event.is_set():
                state = avp.latest
                state = {k: np.array(v, dtype=self.dtype) for k, v in state.items()}
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


def AvpStreamServer(mw, *args, **kwargs):
    return ServerFactory(mw, AvpStream, *args, **kwargs)


def AvpStreamClient(mw, *args, **kwargs):
    return ClientFactory(mw, AvpStream, *args, **kwargs)
