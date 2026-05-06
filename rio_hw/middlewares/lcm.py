import platform
import subprocess
import threading as th
import uuid
from typing import TYPE_CHECKING

from ..node import Node
from ..serializers import Serializer
from ._serialize import wrap_fn_unpack
from ._storage import Queue, RingBuffer

try:
    import lcm
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        lcm = None  # type: ignore


def _to_multicast_addr(addr: str) -> str:
    """Ensure addr uses a multicast IP, replacing the host if needed."""
    host, port = addr.rsplit(":", 1)
    first_octet = int(host.split(".")[0])
    if not (224 <= first_octet <= 239):
        host = "239.255.76.67"
    return f"{host}:{port}"


def _check_multicast():
    """Verify multicast routing is configured for loopback."""
    system = platform.system()
    if system == "Linux":
        cmd = ["ip", "route", "get", "224.0.0.1"]
        loopback = "lo"
        fix = "sudo ip route add 224.0.0.0/4 dev lo"
    elif system == "Darwin":
        cmd = ["route", "-n", "get", "224.0.0.1"]
        loopback = "lo0"
        fix = "sudo route delete -net 224.0.0.0/4 && sudo route add -net 224.0.0.0/4 -interface lo0"
    else:
        return

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5, check=False)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return

    # parse interface from output
    iface = None
    if system == "Linux":
        parts = result.stdout.split()
        if "dev" in parts:
            iface = parts[parts.index("dev") + 1]
    elif system == "Darwin":
        for line in result.stdout.splitlines():
            if "interface:" in line:
                iface = line.split(":")[-1].strip()

    if iface == loopback:
        return
    if iface:
        raise RuntimeError(f"LCM multicast routes to '{iface}' instead of '{loopback}'. Fix: {fix}")
    raise RuntimeError(f"No multicast route found. Fix: {fix}")


class LcmServer(th.Thread, Node):
    def __init__(
        self,
        daemon: bool = True,
        topic: str | None = None,
        *,
        addr: str = "239.255.55.55:5555",
        freq: int = 100,
        max_buffer_size: int = 30,
        max_queue_size: int = 100,
        serializer: str = "msgpack",
        timeout: float = 5.0,
        verbose=True,
        **kwargs,
    ):
        super().__init__(daemon=daemon)
        addr = _to_multicast_addr(addr)
        if topic is None:
            topic = f"{self.__nodename__}/{addr.split(':')[-1]}"
        self.topic = topic
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
        _check_multicast()
        self._lc = lcm.LCM(f"udpm://{self.addr}?ttl=255")

        self.ring_buffer = RingBuffer(self.max_buffer_size) if self.__pub__ else None
        self.request_queue = Queue(self.max_queue_size) if self.__req__ else None
        self.pub_ready_event = th.Event() if self.__pub__ else None
        self.req_ready_event = th.Event() if self.__req__ else None
        self.exit_event = th.Event()
        self.worker_thread = th.Thread(target=self.worker, daemon=True) if self.worker is not None else None
        self.main_thread = super()  # self.run

        if self.__pub__:
            self.ring_buffer._put = self.ring_buffer.put
            self.ring_buffer.put = self._put

        self._lc.subscribe(f"{self.topic}/api_request", self._on_api_request)
        self._lcm_thread = th.Thread(target=self._request_loop, daemon=True)

    def _put(self, data, **kwargs):
        self.ring_buffer._put(data, **kwargs)
        self._lc.publish(f"{self.topic}/state", self._serializer.pack(data))

    def _on_api_request(self, channel, data):
        request_id, fn_name, args, kwargs = self._serializer.unpack(data)
        try:
            result = getattr(self, fn_name)(*args, **kwargs)
            response = (request_id, result, None)
        except Exception as e:
            response = (request_id, None, str(e))
        self._lc.publish(f"{self.topic}/api_response", self._serializer.pack(response))

    def _request_loop(self):
        try:
            while not self.exit_event.is_set():
                self._lc.handle_timeout(100)
        except KeyboardInterrupt:
            pass

    def start(self):
        self.worker_thread.start() if self.worker_thread is not None else None
        self.main_thread.start()
        self.pub_ready_event.wait(self.timeout) if self.pub_ready_event is not None else None
        self.req_ready_event.wait(self.timeout) if self.req_ready_event is not None else None
        assert self.worker_thread.is_alive() if self.worker_thread is not None else True
        assert self.main_thread.is_alive()
        self._lcm_thread.start()
        assert self._lcm_thread.is_alive()

    def stop(self):
        self.exit_event.set()
        self.worker_thread.join(self.timeout) if self.worker_thread is not None else None
        self.main_thread.join(self.timeout)
        self._lcm_thread.join(self.timeout)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()


class LcmClient(Node):
    def __init__(
        self,
        topic: str | None = None,
        *,
        addr: str = "239.255.55.55:5555",
        serializer: str = "msgpack",
        timeout: float = 5.0,
        verbose=True,
        **kwargs,
    ):
        addr = _to_multicast_addr(addr)
        if topic is None:
            topic = f"{self.__nodename__}/{addr.split(':')[-1]}"
        self.topic = topic
        self.addr = addr
        self.serializer = serializer
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        self._serializer = Serializer.make(self.serializer)
        _check_multicast()
        self._lc = lcm.LCM(f"udpm://{self.addr}?ttl=255")
        self._pending = {}  # request_id -> Event
        self._results = {}  # request_id -> (result, error)
        self._pending_lock = th.Lock()
        self._recv_running = False

        self._lc.subscribe(f"{self.topic}/api_response", self._on_api_response)

        for fn_name in self.__api__:

            def fn_wrapper(self, *args, __fn_name=fn_name, **kwargs):
                return self._rpc_call(__fn_name, *args, **kwargs)

            wrap_fn_unpack(self, fn_name, fn_wrapper)

    def _on_api_response(self, channel, data):
        request_id, result, error = self._serializer.unpack(data)
        with self._pending_lock:
            if request_id in self._pending:
                self._results[request_id] = (result, error)
                self._pending[request_id].set()

    def _rpc_call(self, fn_name, *args, **kwargs):
        request_id = str(uuid.uuid4())
        event = th.Event()
        with self._pending_lock:
            self._pending[request_id] = event
        self._lc.publish(
            f"{self.topic}/api_request",
            self._serializer.pack((request_id, fn_name, args, kwargs)),
        )
        if not event.wait(timeout=self.timeout):
            with self._pending_lock:
                self._pending.pop(request_id, None)
                self._results.pop(request_id, None)
            raise TimeoutError(f"RPC call '{fn_name}' timed out")
        with self._pending_lock:
            self._pending.pop(request_id, None)
            result, error = self._results.pop(request_id)
        if error is not None:
            raise RuntimeError(error)
        return result

    def _recv_loop(self):
        try:
            while self._recv_running:
                self._lc.handle_timeout(100)
        except KeyboardInterrupt:
            pass

    def start(self):
        self._recv_running = True
        self._recv_thread = th.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def stop(self):
        self._recv_running = False
        self._recv_thread.join(self.timeout)
        with self._pending_lock:
            for event in self._pending.values():
                event.set()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()
