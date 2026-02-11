import threading as th
from typing import TYPE_CHECKING

from ..node import Node
from ..serializers import PickleSerializer
from ._serialize import wrap_fn_unpack
from ._storage import Queue, RingBuffer

try:
    import zenoh
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        zenoh = None  # type: ignore


class ZenohServer(th.Thread, Node):
    def __init__(
        self,
        daemon: bool = True,
        topic: str | None = None,
        zenoh_kwargs: dict | None = None,
        transport: str = "tcp",
        *,
        addr: str = "127.0.0.1:5555",
        freq: int = 100,
        max_buffer_size: int = 30,
        max_queue_size: int = 100,
        timeout: float = 5.0,
        verbose=True,
        **kwargs,
    ):
        super().__init__(daemon=daemon)
        if topic is None:
            port = addr.split(":")[-1]
            topic = f"{self.__nodename__}/{port}"
        self.topic = topic
        self.zenoh_kwargs = zenoh_kwargs
        assert transport in ("tcp",)
        self.transport = transport
        self.addr = addr
        self.freq = freq
        self.max_buffer_size = max_buffer_size
        self.max_queue_size = max_queue_size
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        def _on_query_factory(fn_name: str):
            method = getattr(self, fn_name)

            def _on_query(q):
                with q:
                    try:
                        raw = getattr(q, "payload", None)
                        if raw is None:
                            args, kwargs = (), {}  # no args
                        else:
                            args, kwargs = PickleSerializer.unpack(raw.to_bytes())
                        result = method(*args, **kwargs)
                        payload = PickleSerializer.pack(result)
                        q.reply(q.key_expr, zenoh.ZBytes(payload), encoding=zenoh.Encoding.APPLICATION_OCTET_STREAM)
                    except Exception as e:
                        q.reply_err(str(e).encode("utf-8"), encoding=zenoh.Encoding.TEXT_PLAIN)

            return _on_query

        def make_queryables(session):
            queryables = {}
            for fn_name in self.__api__:
                key = f"{self.topic}/{fn_name}"
                qbl = session.declare_queryable(key, _on_query_factory(fn_name))
                queryables[fn_name] = qbl
            return queryables

        def run_server():
            config = zenoh.Config()
            config.insert_json5("mode", "'peer'")
            config.insert_json5("listen/endpoints", f'["{self.transport}/{self.addr}"]')
            if self.zenoh_kwargs is not None:
                for k, v in self.zenoh_kwargs.items():
                    config.insert_json5(k, v)

            with zenoh.open(config) as session:
                queryables = make_queryables(session)  # rpc style
                self.exit_event.wait()
                for q in queryables.values():  # shutdown queryables
                    q.undeclare()

        self.server_thread = th.Thread(target=run_server, daemon=self.daemon)

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

        self.server_thread.start()
        assert self.server_thread.is_alive()

    def stop(self):
        self.exit_event.set()
        if self.worker_thread is not None:
            self.worker_thread.join(self.timeout)
        self.main_thread.join(self.timeout)

        self.server_thread.join(self.timeout)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()


class ZenohClient(Node):
    def __init__(
        self,
        topic: str | None = None,
        zenoh_kwargs: dict | None = None,
        transport: str = "tcp",
        *,
        addr: str = "127.0.0.1:5555",
        timeout: float = 5.0,
        verbose: bool = True,
        **kwargs,
    ):
        if topic is None:
            port = addr.split(":")[-1]
            topic = f"{self.__nodename__}/{port}"
        self.topic = topic
        self.zenoh_kwargs = zenoh_kwargs
        assert transport in ("tcp",)
        self.transport = transport
        self.addr = addr
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def _proxy(self, fn_name: str, *args, **kwargs):
        key = f"{self.topic}/{fn_name}"
        payload = PickleSerializer.pack((args, kwargs))
        payload = zenoh.ZBytes(payload)
        sample = self._session.get(key, payload=payload, target=zenoh.QueryTarget.BEST_MATCHING, timeout=self.timeout)
        if sample is None:
            raise TimeoutError(f"No reply received for remote call '{fn_name}'")
        msg = None
        for rep in sample:
            if rep.ok is not None:
                msg = rep.ok.payload
        if msg is None:
            raise RuntimeError(f"Invalid reply received for remote call '{fn_name}'")
        return PickleSerializer.unpack(msg.to_bytes())

    def __post_init__(self):
        conf = zenoh.Config()
        conf.insert_json5("mode", "'client'")
        conf.insert_json5("connect/endpoints", f'["{self.transport}/{self.addr}"]')
        if self.zenoh_kwargs is not None:
            for k, v in self.zenoh_kwargs.items():
                conf.insert_json5(k, v)
        self.zenoh_config = conf

        # create wrappers to query zenoh session
        for fn_name in self.__api__:

            def fn_wrapper(self, *args, __fn_name=fn_name, **kwargs):
                return self._proxy(__fn_name, *args, **kwargs)

            wrap_fn_unpack(self, fn_name, fn_wrapper)

    def start(self):
        self._session_cm = zenoh.open(self.zenoh_config)
        self._session = self._session_cm.__enter__()

    def stop(self):
        self._session_cm.__exit__(None, None, None)  # close session context manager
        self._session_cm = None
        self._session = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()
