import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request

try:
    # import my_package
    pass
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        my_package = None  # type: ignore


class RequestType(Enum):
    METHOD = auto()


class Template(Node):
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
            not_pub_ready = True
            while not self.exit_event.is_set():
                state = {}
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

    def req(self):
        try:
            # Main loop
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            while not self.exit_event.is_set():
                # Fetch requests from queue
                try:
                    reqs = self.request_queue.get_all()
                    if isinstance(reqs, dict):
                        reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
                except queue.Empty:
                    reqs = []
                for r in reqs:
                    req = Request(RequestType(r.pop("type")), r)
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


def TemplateServer(mw, *args, **kwargs):
    return ServerFactory(mw, Template, *args, **kwargs)


def TemplateClient(mw, *args, **kwargs):
    return ClientFactory(mw, Template, *args, **kwargs)
