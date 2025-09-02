from collections import deque
from itertools import islice
from threading import RLock
from typing import Generic, TypeVar

T = TypeVar("T")


class RingBuffer(Generic[T]):
    def __init__(self, size: int):
        self.size = size
        # Use a single deque as the main storage
        self.buffer: deque[T | None] = deque(maxlen=size)
        self.write_lock = RLock()
        self.read_lock = RLock()

    def put(self, item: T) -> None:
        """Write item to buffer with minimal locking"""
        with self.write_lock:
            self.buffer.append(item)

    def get(self, out=None) -> T:
        with self.read_lock:
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
