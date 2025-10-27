import threading as th
from collections import deque
from itertools import islice
from typing import Generic, TypeVar

T = TypeVar("T")


class RingBuffer(Generic[T]):
    def __init__(self, maxsize: int | None):
        self.maxsize = maxsize
        # Use a single deque as the main storage
        self.buffer: deque[T | None] = deque(maxlen=maxsize)
        self.write_lock = th.Lock()
        self.read_lock = th.Lock()

    def put(self, item: T, wait: bool = True) -> None:
        """Write item to buffer with minimal locking"""
        with self.write_lock:
            self.buffer.append(item)

    def get(self, out=None) -> T:
        with self.read_lock:
            if len(self.buffer) == 0:
                return []
            return list(self.buffer)[-1]

    def get_last_k(self, k: int, out=None) -> list[T]:
        """
        Read last k items from buffer efficiently using deque's optimized operations
        and avoiding unnecessary copies
        """
        with self.read_lock:
            buffer_len = len(self.buffer)
            if k > buffer_len:
                k = buffer_len
            if k == 0:
                return []

            # For small k, direct slicing is fine
            if k <= 100:
                return list(self.buffer)[-k:]

            # For large k, use islice which is more memory efficient
            return list(islice(self.buffer, buffer_len - k, buffer_len))

    def get_all(self, out=None) -> list[T]:
        with self.read_lock:
            return list(self.buffer)

    def clear(self) -> None:
        """Clear the buffer"""
        with self.write_lock, self.read_lock:
            self.buffer.clear()

    def __len__(self) -> int:
        return len(self.buffer)


class Queue(Generic[T]):
    def __init__(self, maxsize: int | None):
        self.maxsize = maxsize
        self.buffer: deque[T | None] = deque(maxlen=maxsize)
        self.mutex = th.Lock()

    def qsize(self) -> int:
        with self.read_lock:
            return len(self.buffer)

    def empty(self) -> bool:
        with self.read_lock:
            return len(self.buffer) == 0

    def clear(self) -> None:
        with self.mutex:
            self.buffer.clear()

    def put(self, item: T) -> None:
        with self.mutex:
            self.buffer.append(item)

    def get(self, out=None, timeout=None) -> T | None:
        with self.mutex:
            if len(self.buffer) == 0:
                return None
            return self.buffer.popleft()

    def get_k(self, k, out=None, timeout=None) -> list[T]:
        if out is None:
            out = []
        with self.mutex:
            if len(self.buffer) == 0:
                return []
            for _ in range(k):
                out.append(self.buffer.popleft())
            return out

    def get_all(self, out=None, timeout=None) -> list[T]:
        if out is None:
            out = []
        with self.mutex:
            out.extend(list(self.buffer))
            self.buffer.clear()
            return out

    def __len__(self) -> int:
        return self.qsize()
