import queue
import threading as th

from ..ring_buffer import RingBuffer
from ._middleware import Node


class ThreadServer(Node):
    def __init__(self, *args, **kwargs):
        self.__post_init__()

    def __post_init__(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass


class ThreadClient(th.Thread, Node):
    def __init__(
        self,
        *,
        freq: int = 100,
        max_queue_size: int = 100,
        max_buffer_size: int = 30,
        timeout: float = 5.0,
        verbose=True,
        **kwargs,
    ):
        super().__init__()
        self.freq = freq
        self.max_queue_size = max_queue_size
        self.max_buffer_size = max_buffer_size
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        if self.example_request is not None:
            self.input_queue = queue.Queue(self.max_queue_size)
        assert self.example_data is not None
        self.ring_buffer = RingBuffer(self.max_buffer_size)
        self.ready_event = th.Event()
        self.exit_event = th.Event()

    def start(self):
        super().start()
        self.ready_event.wait(self.timeout)
        assert self.is_alive()

    def stop(self):
        self.exit_event.set()
        self.join(self.timeout)
