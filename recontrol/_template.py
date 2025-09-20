import queue
from enum import Enum, auto

import numpy as np

from . import time
from .request import Request


class RequestType(Enum):
    METHOD = auto()


class Template:
    __api__ = [
        "get_state",
        "get_all_state",
        "method",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        dtype=np.float64,
        *,
        freq: int = 100,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_request = {
            "type": next(iter(RequestType)).value,
        }
        self.example_data = {
            "timestamp": time.now(),
        }
        self.worker = self.req
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        try:
            # Main loop
            rate = time.Rate(self.freq)
            self.pub_ready_event.set()
            while not self.exit_event.is_set():
                state = {}
                # Store current state in ring buffer
                data = {
                    **state,
                    "timestamp": time.now(),
                }
                self.ring_buffer.put(data)
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            pass

    def req(self):
        try:
            # Main loop
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            while not self.exit_event.is_set():
                # Fetch request from queue
                try:
                    req = self.request_queue.get()
                    if isinstance(req, dict):
                        req = Request(RequestType(req.pop("type")), req)
                except queue.Empty:
                    req = None
                if req:
                    if req.type == RequestType.METHOD:
                        raise NotImplementedError
                    else:
                        raise RuntimeError
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

    def method(self):
        req = {
            "type": RequestType.METHOD.value,
        }
        self.request_queue.put(req)
