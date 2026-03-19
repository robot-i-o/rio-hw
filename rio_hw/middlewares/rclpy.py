import re
import threading as th
import time as _time
import traceback
import uuid
from typing import TYPE_CHECKING

import numpy as np

from ..node import Node
from ..serializers import Serializer
from ._serialize import wrap_fn_unpack
from ._storage import Queue, RingBuffer

try:
    import rclpy
    import rclpy.executors
    import rclpy.node
    import std_msgs.msg as std_msgs
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        rclpy = None  # type: ignore
        std_msgs = None  # type: ignore


class RosStdMsgs:
    """Mapping between Python/numpy types and native rclpy std_msgs."""

    if std_msgs is not None:
        NUMPY = {
            np.dtype("float32"): std_msgs.Float32MultiArray,
            np.dtype("float64"): std_msgs.Float64MultiArray,
            np.dtype("int8"): std_msgs.Int8MultiArray,
            np.dtype("int16"): std_msgs.Int16MultiArray,
            np.dtype("int32"): std_msgs.Int32MultiArray,
            np.dtype("int64"): std_msgs.Int64MultiArray,
            np.dtype("uint8"): std_msgs.UInt8MultiArray,
            np.dtype("uint16"): std_msgs.UInt16MultiArray,
            np.dtype("uint32"): std_msgs.UInt32MultiArray,
            np.dtype("uint64"): std_msgs.UInt64MultiArray,
        }

        TO_NUMPY = {v: k for k, v in NUMPY.items()}

        SCALAR = {
            float: std_msgs.Float64,
            int: std_msgs.Int64,
            bool: std_msgs.Bool,
            str: std_msgs.String,
        }

        BYTES = std_msgs.UInt8MultiArray

        TYPE_STR = {f"std_msgs/msg/{cls.__name__}": cls for cls in [*NUMPY.values(), *SCALAR.values()]}

    @staticmethod
    def infer(value):
        if isinstance(value, np.ndarray):
            msg_class = RosStdMsgs.NUMPY.get(value.dtype)
            if msg_class is None:
                raise TypeError(f"Unsupported numpy dtype: {value.dtype}")
            return msg_class
        for py_type, msg_class in RosStdMsgs.SCALAR.items():
            if isinstance(value, py_type):
                return msg_class
        raise TypeError(f"Unsupported type for ROS: {type(value)}")

    @staticmethod
    def to_msg(value):
        """Convert a Python value to a native rclpy message."""
        if isinstance(value, np.ndarray):
            dims = [
                std_msgs.MultiArrayDimension(
                    label=f"dim{i}",
                    size=int(s),
                    stride=int(np.prod(value.shape[i:])),
                )
                for i, s in enumerate(value.shape)
            ]
            msg = RosStdMsgs.NUMPY[value.dtype]()
            msg.layout = std_msgs.MultiArrayLayout(dim=dims, data_offset=0)
            msg.data = value.flatten().tolist()
            return msg
        msg_class = RosStdMsgs.infer(value)
        msg = msg_class()
        msg.data = value
        return msg

    @staticmethod
    def from_msg(msg_class, msg):
        """Convert a native rclpy message back to a Python value."""
        dtype = RosStdMsgs.TO_NUMPY.get(msg_class)
        if dtype is not None:
            shape = tuple(d.size for d in msg.layout.dim)
            return np.array(msg.data, dtype=dtype).reshape(shape)
        return msg.data

    @staticmethod
    def from_type_str(type_str):
        """Resolve a ROS type string (e.g. 'std_msgs/msg/Float64') to msg class."""
        cls = RosStdMsgs.TYPE_STR.get(type_str)
        if cls is None:
            raise ValueError(f"Unknown ROS type string: {type_str}")
        return cls


def _sanitize_node_name(name: str) -> str:
    """Sanitize a string to be a valid ROS 2 node name."""
    name = re.sub(r"[^a-zA-Z0-9_]", "_", name)
    if name and name[0].isdigit():
        name = "_" + name
    return name


def _wait_for_discovery(node, publisher, response_topic, timeout=5.0):
    """Block until the publisher has a subscriber and the response topic has a publisher."""
    deadline = _time.monotonic() + timeout
    while _time.monotonic() < deadline:
        has_sub = publisher.get_subscription_count() > 0
        has_pub = node.count_publishers(response_topic) > 0
        if has_sub and has_pub:
            return
        _time.sleep(0.01)
    raise RuntimeError(
        f"Timed out waiting for DDS discovery on '{response_topic}' "
        f"(pub matched: {publisher.get_subscription_count() > 0}, "
        f"sub matched: {node.count_publishers(response_topic) > 0})"
    )


class RclpyServer(th.Thread, Node):
    def __init__(
        self,
        daemon: bool = True,
        topic: str | None = None,
        qos_depth: int = 10,
        *,
        addr: str = "127.0.0.1:5555",
        freq: int = 100,
        max_buffer_size: int = 30,
        max_queue_size: int = 100,
        serializer: str = "pickle",
        timeout: float = 5.0,
        verbose=True,
        **kwargs,
    ):
        super().__init__(daemon=daemon)
        port = addr.split(":")[-1]
        if topic is None:
            topic = f"{self.__nodename__}/p{port}"
        self.topic = topic
        self.addr = addr
        self.qos_depth = qos_depth
        self.freq = freq
        self.max_buffer_size = max_buffer_size
        self.max_queue_size = max_queue_size
        self.serializer = serializer
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        self._serializer = Serializer.make(self.serializer)

        port = self.addr.split(":")[-1]
        node_name = _sanitize_node_name(f"{self.__nodename__}_p{port}_server")
        self._context = rclpy.Context()
        self._context.init()
        self._ros_node = rclpy.node.Node(node_name, context=self._context)
        self._executor = rclpy.executors.SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._ros_node)

        # State: one publisher per field with native ROS type
        self._state_pubs = {}
        self._state_msg_classes = {}
        if self.__pub__:
            for field, value in self.example_data.items():
                msg_class = RosStdMsgs.infer(value)
                pub = self._ros_node.create_publisher(msg_class, f"{self.topic}/state/{field}", self.qos_depth)
                self._state_pubs[field] = pub
                self._state_msg_classes[field] = msg_class

        # API: serialized RPC over UInt8MultiArray topics
        self._api_response_pub = self._ros_node.create_publisher(RosStdMsgs.BYTES, f"{self.topic}/api_response", self.qos_depth)
        self._api_request_sub = self._ros_node.create_subscription(
            RosStdMsgs.BYTES,
            f"{self.topic}/api_request",
            self._on_api_request,
            self.qos_depth,
        )

        # Standard middleware setup
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

        self._spin_thread = th.Thread(target=self._spin_loop, daemon=True)

    def _put(self, data):
        self.ring_buffer._put(data)
        for field, value in data.items():
            msg_class = self._state_msg_classes[field]
            expected_dtype = RosStdMsgs.TO_NUMPY.get(msg_class)
            if expected_dtype is not None and isinstance(value, np.ndarray) and value.dtype != expected_dtype:
                value = value.astype(expected_dtype)  # noqa: PLW2901
            self._state_pubs[field].publish(RosStdMsgs.to_msg(value))

    def _on_api_request(self, msg):
        try:
            request_id, fn_name, args, kwargs = self._serializer.unpack(bytes(msg.data))
            try:
                result = getattr(self, fn_name)(*args, **kwargs)
                response = (request_id, result, None)
            except Exception:
                response = (request_id, None, traceback.format_exc())
            resp_msg = RosStdMsgs.BYTES()
            resp_msg.data = list(self._serializer.pack(response))
            self._api_response_pub.publish(resp_msg)
        except Exception:
            traceback.print_exc()

    def _spin_loop(self):
        self._executor.spin()

    def start(self):
        self.worker_thread.start() if self.worker_thread is not None else None
        self.main_thread.start()
        self.pub_ready_event.wait(self.timeout) if self.pub_ready_event is not None else None
        self.req_ready_event.wait(self.timeout) if self.req_ready_event is not None else None
        assert self.worker_thread.is_alive() if self.worker_thread is not None else True
        assert self.main_thread.is_alive()
        self._spin_thread.start()

    def stop(self):
        self.exit_event.set()
        self.worker_thread.join(self.timeout) if self.worker_thread is not None else None
        self.main_thread.join(self.timeout)
        self._executor.shutdown()
        self._spin_thread.join(self.timeout)
        self._ros_node.destroy_node()
        self._context.shutdown()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()


class RclpyClient(Node):
    def __init__(
        self,
        topic: str | None = None,
        qos_depth: int = 10,
        *,
        addr: str = "127.0.0.1:5555",
        freq: int = 100,
        max_buffer_size: int = 30,
        max_queue_size: int = 100,
        serializer: str = "pickle",
        timeout: float = 5.0,
        verbose=True,
        **kwargs,
    ):
        port = addr.split(":")[-1]
        if topic is None:
            topic = f"{self.__nodename__}/p{port}"
        self.topic = topic
        self.addr = addr
        self.qos_depth = qos_depth
        self.freq = freq
        self.max_buffer_size = max_buffer_size
        self.max_queue_size = max_queue_size
        self.serializer = serializer
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        self._serializer = Serializer.make(self.serializer)

        # RPC state
        self._pending = {}  # request_id -> Event
        self._results = {}  # request_id -> (result, error)
        self._pending_lock = th.Lock()

        # ROS 2 node
        port = self.addr.split(":")[-1]
        node_name = _sanitize_node_name(f"{self.__nodename__}_p{port}_client")
        self._context = rclpy.Context()
        self._context.init()
        self._ros_node = rclpy.node.Node(node_name, context=self._context)
        self._executor = rclpy.executors.SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._ros_node)

        # Middleware buffers
        self.ring_buffer = RingBuffer(self.max_buffer_size)
        self.request_queue = Queue(self.max_queue_size)
        self.exit_event = th.Event()
        self.pub_ready_event = th.Event()

        # State: per-field subscription tracking
        self._latest_state = {}
        self._state_lock = th.Lock()
        self._msg_classes = {}

        # API: serialized RPC over UInt8MultiArray topics
        self._api_request_pub = self._ros_node.create_publisher(RosStdMsgs.BYTES, f"{self.topic}/api_request", self.qos_depth)
        self._api_response_sub = self._ros_node.create_subscription(
            RosStdMsgs.BYTES,
            f"{self.topic}/api_response",
            self._on_api_response,
            self.qos_depth,
        )

        # Bind __api__ methods to RPC
        for fn_name in self.__api__:
            if hasattr(self, fn_name):
                continue  # get_state, get_all_state already local

            def fn_wrapper(self, *args, __fn_name=fn_name, **kwargs):
                return self._rpc_call(__fn_name, *args, **kwargs)

            wrap_fn_unpack(self, fn_name, fn_wrapper)

        self._spin_thread = th.Thread(target=self._spin_loop, daemon=True)

    def _on_field_update(self, field, msg_class, msg):
        value = RosStdMsgs.from_msg(msg_class, msg)
        with self._state_lock:
            self._latest_state[field] = value
            snapshot = dict(self._latest_state)
        self.ring_buffer.put(snapshot)
        self.request_queue.put(snapshot)

    def _on_api_response(self, msg):
        request_id, result, error = self._serializer.unpack(bytes(msg.data))
        with self._pending_lock:
            if request_id in self._pending:
                self._results[request_id] = (result, error)
                self._pending[request_id].set()

    def _rpc_call(self, fn_name, *args, **kwargs):
        request_id = str(uuid.uuid4())
        event = th.Event()
        with self._pending_lock:
            self._pending[request_id] = event
        req_msg = RosStdMsgs.BYTES()
        req_msg.data = list(self._serializer.pack((request_id, fn_name, args, kwargs)))
        self._api_request_pub.publish(req_msg)
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

    def _discover_and_subscribe(self):
        """Discover state topics published by the server and create subscriptions."""
        prefix = f"/{self.topic}/state/"
        deadline = _time.monotonic() + self.timeout
        discovered = {}
        while _time.monotonic() < deadline:
            for topic_name, type_strs in self._ros_node.get_topic_names_and_types():
                if topic_name.startswith(prefix) and topic_name not in discovered:
                    field = topic_name[len(prefix) :]
                    type_str = type_strs[0]
                    msg_class = RosStdMsgs.from_type_str(type_str)
                    discovered[topic_name] = (field, msg_class)
            if discovered:
                break
            _time.sleep(0.01)
        if not discovered:
            raise RuntimeError(f"Timed out waiting for state topics under '{prefix}'")
        for topic_name, (field, msg_class) in discovered.items():
            self._msg_classes[field] = msg_class
            self._ros_node.create_subscription(
                msg_class,
                topic_name,
                lambda msg, _field=field, _cls=msg_class: self._on_field_update(_field, _cls, msg),
                self.qos_depth,
            )

    def _spin_loop(self):
        self._executor.spin()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def start(self):
        self._spin_thread.start()
        if self.__pub__:
            self._discover_and_subscribe()
            deadline = _time.monotonic() + self.timeout
            while _time.monotonic() < deadline:
                if len(self.ring_buffer) > 0:
                    break
                _time.sleep(0.01)
        self.pub_ready_event.set()
        _wait_for_discovery(self._ros_node, self._api_request_pub, f"{self.topic}/api_response", self.timeout)

    def stop(self):
        self.exit_event.set()
        with self._pending_lock:
            for event in self._pending.values():
                event.set()
        self._executor.shutdown()
        self._spin_thread.join(self.timeout)
        self._ros_node.destroy_node()
        self._context.shutdown()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()
