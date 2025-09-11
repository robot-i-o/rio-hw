import multiprocessing as mp
import threading as th
from multiprocessing.managers import SharedMemoryManager
from urllib.parse import urlparse

from ._middleware import Node
from .shared_memory import SharedMemoryQueue, SharedMemoryRingBuffer


class ShmServer(Node):
    def __init__(self, *args, **kwargs):
        self.__post_init__()

    def __post_init__(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass


class ShmClient(mp.Process, Node):
    def __init__(
        self,
        daemon: bool = True,
        shm_addr: str = "127.0.0.1:5555",  # NOTE: use same addr across all node processes
        *,
        freq: int = 100,
        max_buffer_size: int = 30,
        max_queue_size: int = 100,
        timeout: float = 5.0,
        verbose=True,
        **kwargs,
    ):
        super().__init__(daemon=daemon)
        self.shm_addr = shm_addr
        self.freq = freq
        self.max_buffer_size = max_buffer_size
        self.max_queue_size = max_queue_size
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        o = urlparse(self.shm_addr)
        self.smm = SharedMemoryManager(address=(o.scheme, int(o.path)), authkey=b"abc")
        try:
            self.smm.connect()
        except ConnectionRefusedError:
            self.smm.start()

        if self.has_pub:
            assert self.example_data is not None
            self.ring_buffer = SharedMemoryRingBuffer.create_from_examples(
                shm_manager=self.smm,
                examples=self.example_data,
                get_max_k=self.max_buffer_size,
                get_time_budget=0.2,
                put_desired_frequency=self.freq,
            )
        else:
            self.ring_buffer = None
        if self.has_req:
            assert self.example_request is not None
            self.request_queue = SharedMemoryQueue.create_from_examples(
                shm_manager=self.smm,
                examples=self.example_request,
                buffer_size=self.max_queue_size,
            )
        else:
            self.request_queue = None
        self.pub_ready_event = mp.Event() if self.has_pub else None
        self.req_ready_event = mp.Event() if self.has_req else None
        self.exit_event = mp.Event()
        self.worker_thread = th.Thread(target=self.worker, daemon=self.daemon) if self.worker is not None else None
        self.main_process = super()  # self.run

    def start(self):
        self.worker_thread.start() if self.worker_thread is not None else None
        self.main_process.start()
        self.pub_ready_event.wait(timeout=self.timeout) if self.pub_ready_event is not None else None
        self.req_ready_event.wait(timeout=self.timeout) if self.req_ready_event is not None else None
        assert self.worker_thread.is_alive() if self.worker_thread is not None else True
        assert self.main_process.is_alive()

    def stop(self):
        self.exit_event.set()
        self.worker_thread.join(self.timeout) if self.worker_thread is not None else None
        self.main_process.join(self.timeout)

        try:
            self.smm.shutdown()
        except AttributeError:
            pass
