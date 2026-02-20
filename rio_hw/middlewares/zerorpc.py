import pickle
import threading as th
from typing import TYPE_CHECKING

import numpy as np

from ..node import Node
from ..serializers import PickleSerializer
from ._serialize import get_fn, wrap_fn_pack, wrap_fn_unpack
from ._storage import Queue, RingBuffer

try:
    import msgpack
    import zerorpc
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        zerorpc = None  # type: ignore
        msgpack = None  # type: ignore


# Custom msgpack encoder/decoder for numpy arrays
def _msgpack_encode_numpy(obj):
    """Encode numpy arrays as msgpack extension type"""
    if isinstance(obj, np.ndarray):
        return msgpack.ExtType(42, pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL))
    return obj


def _msgpack_decode_numpy(code, data):
    """Decode numpy arrays from msgpack extension type"""
    if code == 42:
        return pickle.loads(data)
    return msgpack.ExtType(code, data)


_original_Packer = msgpack.Packer
_original_Unpacker = msgpack.Unpacker


class NumpyPacker(_original_Packer):
    def __init__(self, **kwargs):
        kwargs.setdefault("default", _msgpack_encode_numpy)
        kwargs.setdefault("use_bin_type", True)
        super().__init__(**kwargs)


class NumpyUnpacker(_original_Unpacker):
    def __init__(self, **kwargs):
        kwargs.setdefault("ext_hook", _msgpack_decode_numpy)
        kwargs.setdefault("raw", False)
        super().__init__(**kwargs)


# Apply the monkey-patch globally for this module
msgpack.Packer = NumpyPacker
msgpack.Unpacker = NumpyUnpacker


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

        self.ring_buffer = RingBuffer(self.max_buffer_size) if self.has_pub else None
        self.request_queue = Queue(self.max_queue_size) if self.has_req else None
        self.pub_ready_event = th.Event() if self.has_pub else None
        self.req_ready_event = th.Event() if self.has_req else None
        self.exit_event = th.Event()
        self.worker_thread = th.Thread(target=self.worker, daemon=self.daemon) if self.worker is not None else None
        self.main_thread = super()  # self.run

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        # create wrappers to pickle output of api methods
        for fn_name in cls.__api__:
            fn_descriptor, fn = get_fn(cls, fn_name)

            def fn_wrapper(*args, __fn=fn, **kwargs):
                return PickleSerializer.pack(__fn(*args, **kwargs))

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

            def fn_wrapper(self, *args, __fn_name=fn_name, **kwargs):
                return PickleSerializer.unpack(self.proxy(__fn_name, *args, **kwargs))

            wrap_fn_unpack(self, fn_name, fn_wrapper)

    def start(self):
        self.proxy.connect(f"{self.transport}://{self.addr}")

    def stop(self):
        self.proxy.close()
