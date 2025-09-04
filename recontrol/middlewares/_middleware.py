from collections.abc import Callable
from typing import Any, Protocol


class Node(Protocol):
    __api__: list[str]
    example_data: Any
    example_request: Any
    ring_buffer: Any
    request_queue: Any
    worker: Callable | None
    run: Callable

    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def pub(self):
        """Optional."""
        raise NotImplementedError

    def req(self):
        """Optional."""
        raise NotImplementedError

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()


class Server(Node):
    def __init__(self, *args, **kwargs):
        self.__post_init__()

    def __post_init__(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass


class Client(Node):
    def __init__(self, *args, **kwargs):
        self.__post_init__()

    def __post_init__(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass
