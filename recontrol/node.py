from collections.abc import Callable
from typing import Any, Protocol


class Node(Protocol):
    __nodename__: str
    __api__: list[str]
    __pub__: bool
    __req__: bool

    example_data: dict | None
    example_request: dict | None
    worker: Callable | None
    run: Callable

    ring_buffer: Any
    request_queue: Any
    pub_ready_event: Any
    req_ready_event: Any
    exit_event: Any
    worker_thread: Any
    main_process: Any

    freq: int
    max_buffer_size: int
    max_queue_size: int
    timeout: float
    verbose: bool

    def __init__(
        self,
        *,
        freq: int = 100,
        max_buffer_size: int = 30,
        max_queue_size: int = 100,
        timeout: float = 5.0,
        verbose: bool = True,
        **kwargs,
    ):
        self.freq = freq
        self.max_buffer_size = max_buffer_size
        self.max_queue_size = max_queue_size
        self.timeout = timeout
        self.verbose = verbose
        self.__post_init__()

    def __post_init__(self):
        # set by node
        self.example_data = {}
        self.example_request = {}
        self.worker = self.req
        self.run = self.pub

        # set by middleware
        self.ring_buffer = ... if self.has_pub else None
        self.request_queue = ... if self.has_req else None
        self.pub_ready_event = ... if self.has_pub else None
        self.req_ready_event = ... if self.has_req else None
        self.exit_event = ...
        self.worker_thread = ... if self.worker is not None else None
        self.main_process = ...

    def pubreq(self) -> None:
        """Optional, discouraged."""
        ...

    def pub(self) -> None:
        """Optional."""
        ...

    def req(self) -> None:
        """Optional."""
        ...

    @property
    def has_pub(self) -> bool:
        return self.__pub__

    @property
    def has_req(self) -> bool:
        return self.__req__

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()
