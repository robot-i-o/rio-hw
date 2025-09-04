import functools
import queue
import threading as th

from ..ring_buffer import RingBuffer
from ..serializers import PickleSerializer
from ._middleware import Node

try:
    import zerorpc
except ImportError:
    zerorpc = None


class ZeroRpcServer(th.Thread, Node):
    def __init__(
        self,
        *,
        transport: str = "tcp",
        addr: str = "127.0.0.1:5555",
        freq: int = 100,
        max_queue_size: int = 100,
        max_buffer_size: int = 30,
        timeout: float = 5.0,  # should be same as client
        verbose=True,
        **kwargs,
    ):
        super().__init__()
        assert transport in ("tcp", "ipc")
        self.transport = transport
        self.addr = addr
        self.freq = freq
        self.max_queue_size = max_queue_size
        self.max_buffer_size = max_buffer_size
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        def run_server():
            server = zerorpc.Server(self, heartbeat=self.timeout)
            server.bind(f"{self.transport}://{self.addr}")
            server.run()

        self.server_thread = th.Thread(target=run_server, daemon=True)

        self.ring_buffer = RingBuffer(self.max_buffer_size) if self.example_data is not None else None
        self.request_queue = queue.Queue(self.max_queue_size) if self.example_request is not None else None
        self.pub_ready_event = th.Event() if self.example_data is not None else None
        self.req_ready_event = th.Event() if self.example_request is not None else None
        self.exit_event = th.Event()
        self.worker_thread = th.Thread(target=self.worker, daemon=True) if self.worker is not None else None
        self.main_thread = super()  # self.run

    def __init_subclass__(cls, **kwargs):  # create wrappers to pickle output of api methods
        super().__init_subclass__(**kwargs)
        for fn_name in cls.__api__:
            attr = getattr(cls, fn_name)
            if isinstance(attr, classmethod) or isinstance(attr, staticmethod):
                fn = attr.__func__
            else:
                fn = attr
            assert callable(fn)

            def fn_wrapper(*args, __f=fn, **kwargs):
                return PickleSerializer.pack(__f(*args, **kwargs))

            wrapped = functools.wraps(fn)(fn_wrapper)
            if isinstance(attr, classmethod):
                wrapped = classmethod(wrapped)
            elif isinstance(attr, staticmethod):
                wrapped = staticmethod(wrapped)
            setattr(cls, fn_name, wrapped)

    def start(self):
        if self.worker_thread is not None:
            self.worker_thread.start()
        self.main_thread.start()
        if self.pub_ready_event is not None:
            self.pub_ready_event.wait(timeout=self.timeout)
        if self.req_ready_event is not None:
            self.req_ready_event.wait(timeout=self.timeout)
        if self.worker_thread is not None:
            assert self.worker_thread.is_alive()
        assert self.main_thread.is_alive()

        self.server_thread.start()
        assert self.server_thread.is_alive()

    def stop(self):
        self.exit_event.set()
        if self.worker_thread is not None:
            self.worker_thread.join(self.timeout)
        self.main_thread.join(self.timeout)

        self.server_thread.join(self.timeout)


class ZeroRpcClient(Node):
    def __init__(
        self,
        *,
        transport: str = "tcp",
        addr: str = "127.0.0.1:5555",
        timeout: float = 5.0,  # should be same as server
        verbose=True,
        **kwargs,
    ):
        assert transport in ("ipc", "tcp")
        self.transport = transport
        self.addr = addr
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        self.proxy = zerorpc.Client(heartbeat=self.timeout)
        self.proxy.connect(f"{self.transport}://{self.addr}")

        # create wrappers to unpickle output of api methods
        for fn_name in self.__api__:

            def fn(n):
                return lambda *args, **kwargs: PickleSerializer.unpack(self.proxy(n, *args, **kwargs))

            setattr(self, fn_name, fn(fn_name))

    def start(self):
        pass

    def stop(self):
        self.proxy.close()
