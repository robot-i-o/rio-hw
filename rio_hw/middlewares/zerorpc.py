import threading as th
from typing import TYPE_CHECKING

from ..node import Node
from ..serializers import PickleSerializer
from ._serialize import get_fn, wrap_fn_pack, wrap_fn_unpack
from ._storage import Queue, RingBuffer

try:
    import zerorpc
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        zerorpc = None  # type: ignore


class ZeroRpcServer(th.Thread, Node):
    def __init__(
        self,
        daemon: bool = True,
        transport: str = "tcp",
        *,
        addr: str = "127.0.0.1:5555",
        freq: int = 100,
        max_buffer_size: int = 30,
        max_queue_size: int = 100,
        timeout: float = 5.0,  # should be same as client
        verbose=True,
        **kwargs,
    ):
        super().__init__(daemon=daemon)
        assert transport in ("tcp", "ipc")
        self.transport = transport
        self.addr = addr
        self.freq = freq
        self.max_buffer_size = max_buffer_size
        self.max_queue_size = max_queue_size
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        def run_server():
            server = zerorpc.Server(self, heartbeat=self.timeout)
            server.bind(f"{self.transport}://{self.addr}")
            server.run()

        self.server_thread = th.Thread(target=run_server, daemon=self.daemon)

        self.ring_buffer = RingBuffer(self.max_buffer_size) if self.__pub__ else None
        self.request_queue = Queue(self.max_queue_size) if self.__req__ else None
        self.pub_ready_event = th.Event() if self.__pub__ else None
        self.req_ready_event = th.Event() if self.__req__ else None
        self.exit_event = th.Event()
        self.worker_thread = th.Thread(target=self.worker, daemon=self.daemon) if self.worker is not None else None
        self.main_thread = super()  # self.run

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        # create wrappers to pickle output of api methods
        for fn_name in cls.__api__:
            fn_descriptor, fn = get_fn(cls, fn_name)

            def fn_wrapper(*args, __fn__=fn, **kwargs):
                return PickleSerializer.pack(__fn__(*args, **kwargs))

            wrap_fn_pack(cls, fn_name, fn_descriptor, fn, fn_wrapper)

    def start(self):
        self.worker_thread.start() if self.worker_thread is not None else None
        self.main_thread.start()
        self.pub_ready_event.wait(timeout=self.timeout) if self.pub_ready_event is not None else None
        self.req_ready_event.wait(timeout=self.timeout) if self.req_ready_event is not None else None
        assert self.worker_thread.is_alive() if self.worker_thread is not None else True
        assert self.main_thread.is_alive()

        self.server_thread.start()
        assert self.server_thread.is_alive()

    def stop(self):
        self.exit_event.set()
        self.worker_thread.join(self.timeout) if self.worker_thread is not None else None
        self.main_thread.join(self.timeout)

        self.server_thread.join(self.timeout)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()


class ZeroRpcClient(Node):
    def __init__(
        self,
        transport: str = "tcp",
        *,
        addr: str = "127.0.0.1:5555",
        timeout: float = 5.0,  # should be same as server
        verbose=True,
        **kwargs,
    ):
        assert transport in ("tcp", "ipc")
        self.transport = transport
        self.addr = addr
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        self.proxy = zerorpc.Client(heartbeat=self.timeout)

        # create wrappers to unpickle output of api methods
        for fn_name in self.__api__:

            def fn_wrapper(self, *args, __fn_name__=fn_name, **kwargs):
                return PickleSerializer.unpack(self.proxy(__fn_name__, *args, **kwargs))

            wrap_fn_unpack(self, fn_name, fn_wrapper)

    def start(self):
        self.proxy.connect(f"{self.transport}://{self.addr}")

    def stop(self):
        self.proxy.close()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()
