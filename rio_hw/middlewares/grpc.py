import threading as th
from concurrent.futures import ThreadPoolExecutor
from typing import TYPE_CHECKING

from ..node import Node
from ..serializers import Serializer
from ._serialize import wrap_fn_unpack
from ._storage import Queue, RingBuffer

try:
    import grpc
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        grpc = None  # type: ignore


class GrpcServer(th.Thread, Node):
    def __init__(
        self,
        daemon: bool = True,
        topic: str | None = None,
        num_workers: int = 2,
        *,
        addr: str = "127.0.0.1:5555",
        freq: int = 100,
        max_buffer_size: int = 30,
        max_queue_size: int = 100,
        serializer: str = "msgpack",
        timeout: float = 5.0,
        verbose=True,
        **kwargs,
    ):
        super().__init__(daemon=daemon)
        if topic is None:
            port = addr.split(":")[-1]
            topic = f"{self.__nodename__}/{port}"
        self.topic = topic
        self.num_workers = num_workers
        self.addr = addr
        self.freq = freq
        self.max_buffer_size = max_buffer_size
        self.max_queue_size = max_queue_size
        self.serializer = serializer
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        self._serializer = Serializer.make(self.serializer)
        handlers = {}
        for fn_name in self.__api__:
            path = f"/{self.topic}/{fn_name}"
            method = getattr(self, fn_name)

            def _handler(request, context, _method=method):
                args, kwargs = self._serializer.unpack(request)
                result = _method(*args, **kwargs)
                return self._serializer.pack(result)

            handlers[path] = grpc.unary_unary_rpc_method_handler(
                _handler,
                request_deserializer=None,
                response_serializer=None,
            )

        class _Router(grpc.GenericRpcHandler):
            def service(self, handler_call_details):
                return handlers.get(handler_call_details.method)

        self._rpc_handler = _Router()

        self.ring_buffer = RingBuffer(self.max_buffer_size) if self.__pub__ else None
        self.request_queue = Queue(self.max_queue_size) if self.__req__ else None
        self.pub_ready_event = th.Event() if self.__pub__ else None
        self.req_ready_event = th.Event() if self.__req__ else None
        self.exit_event = th.Event()
        self.worker_thread = th.Thread(target=self.worker, daemon=self.daemon) if self.worker is not None else None
        self.main_thread = super()  # self.run

    def start(self):
        self.worker_thread.start() if self.worker_thread is not None else None
        self.main_thread.start()
        self.pub_ready_event.wait(timeout=self.timeout) if self.pub_ready_event is not None else None
        self.req_ready_event.wait(timeout=self.timeout) if self.req_ready_event is not None else None
        assert self.worker_thread.is_alive() if self.worker_thread is not None else True
        assert self.main_thread.is_alive()

        self._server = grpc.server(ThreadPoolExecutor(max_workers=self.num_workers))
        self._server.add_generic_rpc_handlers([self._rpc_handler])
        self._server.add_insecure_port(self.addr)
        self._server.start()

    def stop(self):
        self.exit_event.set()
        self.worker_thread.join(self.timeout) if self.worker_thread is not None else None
        self.main_thread.join(self.timeout)
        self._server.stop(grace=1.0)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()


class GrpcClient(Node):
    def __init__(
        self,
        topic: str | None = None,
        *,
        addr: str = "127.0.0.1:5555",
        serializer: str = "msgpack",
        timeout: float = 5.0,
        verbose: bool = True,
        **kwargs,
    ):
        if topic is None:
            port = addr.split(":")[-1]
            topic = f"{self.__nodename__}/{port}"
        self.topic = topic
        self.addr = addr
        self.serializer = serializer
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        self._serializer = Serializer.make(self.serializer)
        self._stubs = {}

        for fn_name in self.__api__:

            def fn_wrapper(self, *args, __fn_name=fn_name, **kwargs):
                payload = self._serializer.pack((args, kwargs))
                response = self._stubs[__fn_name](payload)
                return self._serializer.unpack(response)

            wrap_fn_unpack(self, fn_name, fn_wrapper)

    def start(self):
        self._channel = grpc.insecure_channel(self.addr)
        for fn_name in self.__api__:
            path = f"/{self.topic}/{fn_name}"
            self._stubs[fn_name] = self._channel.unary_unary(
                path,
                request_serializer=None,
                response_deserializer=None,
            )

    def stop(self):
        self._channel.close()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()
