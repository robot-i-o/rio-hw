import multiprocessing as mp
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
        shm_addr: str = "127.0.0.1:5555",  # NOTE: use same addr across all node processes
        *,
        freq: int = 100,
        max_queue_size: int = 100,
        max_buffer_size: int = 30,
        timeout: float = 5.0,
        verbose=True,
        **kwargs,
    ):
        super().__init__()
        self.shm_addr = shm_addr
        self.freq = freq
        self.max_queue_size = max_queue_size
        self.max_buffer_size = max_buffer_size
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

        if self.example_request is not None:
            self.input_queue = SharedMemoryQueue.create_from_examples(
                shm_manager=self.smm,
                examples=self.example_request,
                buffer_size=self.max_queue_size,
            )
        assert self.example_data is not None
        self.ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=self.smm,
            examples=self.example_data,
            get_max_k=self.max_buffer_size,
            get_time_budget=0.2,
            put_desired_frequency=self.freq,
        )
        self.ready_event = mp.Event()
        self.exit_event = mp.Event()

    def start(self):
        super().start()
        self.ready_event.wait(self.timeout)
        assert self.is_alive()

    def stop(self):
        self.exit_event.set()
        self.join(self.timeout)
        try:
            self.smm.shutdown()
        except AttributeError:
            pass
