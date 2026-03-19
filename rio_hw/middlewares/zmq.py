import threading as th
from typing import TYPE_CHECKING

from ..node import Node
from ..serializers import Serializer
from ._serialize import wrap_fn_unpack
from ._storage import Queue, RingBuffer

try:
    import zmq
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        zmq = None  # type: ignore


class ZmqServer(th.Thread, Node):
    def __init__(
        self,
        daemon: bool = True,
        topic: str | None = None,
        transport: str = "tcp",
        *,
        addr: str = "127.0.0.1:5555",
        freq: int = 100,
        max_buffer_size: int = 30,
        max_queue_size: int = 100,
        frames_per_publish: int = 1,
        serializer: str = "msgpack",
        timeout: float = 5.0,
        verbose=True,
        **kwargs,
    ):
        super().__init__(daemon=daemon)
        assert transport in ("ipc", "tcp")
        if transport == "tcp":
            assert int(addr.rsplit(":", 1)[1]) % 2 == 1, "port must be odd (each node uses port and port+1)"
        assert frames_per_publish > 0 and frames_per_publish <= max_buffer_size
        if topic is None:
            port = addr.split(":")[-1]
            topic = f"{self.__nodename__}/{port}"
        self.topic = topic
        self.transport = transport
        self.addr = addr
        self.freq = freq
        self.max_buffer_size = max_buffer_size
        self.max_queue_size = max_queue_size
        self.frames_per_publish = frames_per_publish
        self.serializer = serializer
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        self._serializer = Serializer.make(self.serializer)
        self.b_topic = self.topic.encode()
        if self.transport == "ipc":
            self.addr = f"/tmp/{self.topic}_pub"
            self.req_addr = f"{self.addr}_req"
        else:
            host, port = self.addr.rsplit(":", 1)
            self.req_addr = f"{host}:{int(port) + 1}"

        # PUB socket setup
        if self.__pub__:
            self.pub_context = zmq.Context()
            self.pub_socket = self.pub_context.socket(zmq.PUB)
            self.pub_socket.setsockopt(zmq.SNDHWM, 1)
            self.pub_socket.setsockopt(zmq.LINGER, 0)
            self.pub_socket.bind(f"{self.transport}://{self.addr}")
        else:
            self.pub_context = None
            self.pub_socket = None

        # ROUTER socket setup (always needed for ALL API calls)
        self.req_context = zmq.Context()
        self.req_socket = self.req_context.socket(zmq.ROUTER)
        self.req_socket.setsockopt(zmq.LINGER, 0)
        self.req_socket.bind(f"{self.transport}://{self.req_addr}")

        self.ring_buffer = RingBuffer(self.max_buffer_size) if self.__pub__ else None
        self.request_queue = Queue(self.max_queue_size) if self.__req__ else None
        self.pub_ready_event = th.Event() if self.__pub__ else None
        self.req_ready_event = th.Event() if self.__req__ else None
        self.exit_event = th.Event()
        self.worker_thread = th.Thread(target=self.worker, daemon=True) if self.worker is not None else None
        self.main_thread = super()  # self.run

        if self.__pub__:
            # monkeypatch to send data to zmq socket
            self.ring_buffer._put = self.ring_buffer.put
            self.ring_buffer.put = self._put

        # Always create request thread (needed for ALL API calls)
        self.request_thread = th.Thread(target=self._request_loop, daemon=True)

    def _put(self, data):
        self.ring_buffer._put(data)
        frames = self.ring_buffer.get_last_k(k=self.frames_per_publish)
        publish_data = self._serializer.pack(frames)
        self.pub_socket.send_multipart([self.b_topic, publish_data])

    def _request_loop(self):
        try:
            while not self.exit_event.is_set():
                if self.req_socket.poll(timeout=100, flags=zmq.POLLIN):
                    message_parts = self.req_socket.recv_multipart()
                    if len(message_parts) != 3:
                        continue

                    identity, _, request_msg = message_parts
                    fn_name, args, kwargs = self._serializer.unpack(request_msg)
                    result = getattr(self, fn_name)(*args, **kwargs)
                    self.req_socket.send_multipart([identity, b"", self._serializer.pack(result)])
        except KeyboardInterrupt:
            pass

    def start(self):
        self.worker_thread.start() if self.worker_thread is not None else None
        self.main_thread.start()
        self.pub_ready_event.wait(self.timeout) if self.pub_ready_event is not None else None
        self.req_ready_event.wait(self.timeout) if self.req_ready_event is not None else None
        assert self.worker_thread.is_alive() if self.worker_thread is not None else True
        assert self.main_thread.is_alive()

        self.request_thread.start()
        assert self.request_thread.is_alive()

    def stop(self):
        self.exit_event.set()
        self.worker_thread.join(self.timeout) if self.worker_thread is not None else None
        self.main_thread.join(self.timeout)

        self.request_thread.join(self.timeout)
        self.pub_socket.close() if self.pub_socket is not None else None
        self.req_socket.close()
        self.pub_context.term() if self.pub_context is not None else None
        self.req_context.term()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()


class ZmqClient(Node):
    def __init__(
        self,
        topic: str | None = None,
        transport: str = "tcp",
        *,
        addr: str = "127.0.0.1:5555",
        serializer: str = "msgpack",
        timeout: float = 5.0,
        verbose=True,
        **kwargs,
    ):
        assert transport in ("ipc", "tcp")
        if transport == "tcp":
            assert int(addr.rsplit(":", 1)[1]) % 2 == 1, "port must be odd (each node uses port and port+1)"
        if topic is None:
            port = addr.split(":")[-1]
            topic = f"{self.__nodename__}/{port}"
        self.topic = topic
        self.transport = transport
        self.addr = addr
        self.serializer = serializer
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        self._serializer = Serializer.make(self.serializer)
        if self.transport == "ipc":
            self.addr = f"/tmp/{self.topic}_pub"
            self.req_addr = f"{self.addr}_req"
        else:
            host, port = self.addr.rsplit(":", 1)
            self.req_addr = f"{host}:{int(port) + 1}"

        self.req_context = zmq.Context()
        self.req_socket = self.req_context.socket(zmq.DEALER)
        self.req_socket.setsockopt(zmq.LINGER, 0)
        self._lock = th.Lock()

        for fn_name in self.__api__:

            def fn_wrapper(self, *args, __fn_name=fn_name, **kwargs):
                with self._lock:
                    self.req_socket.send_multipart([b"", self._serializer.pack((__fn_name, args, kwargs))])
                    _, response_msg = self.req_socket.recv_multipart()
                    return self._serializer.unpack(response_msg)

            wrap_fn_unpack(self, fn_name, fn_wrapper)

    def start(self):
        self.req_socket.connect(f"{self.transport}://{self.req_addr}")

    def stop(self):
        self.req_socket.close()
        self.req_context.term()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()
