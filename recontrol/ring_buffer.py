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

    def write(self, item: T) -> None:
        """Write item to buffer with minimal locking"""
        with self.write_lock:
            self.buffer.append(item)

    def read_last(self, n: int) -> list[T]:
        """
        Read last n items from buffer efficiently using deque's optimized operations
        and avoiding unnecessary copies
        """
        with self.read_lock:
            buffer_len = len(self.buffer)
            if n > buffer_len:
                n = buffer_len
            if n == 0:
                return []

            # For small n, direct slicing is fine
            if n <= 100:
                return list(self.buffer)[-n:]

            # For large n, use islice which is more memory efficient
            return list(islice(self.buffer, buffer_len - n, buffer_len))

    def clear(self) -> None:
        """Clear the buffer"""
        with self.write_lock, self.read_lock:
            self.buffer.clear()

    def __len__(self) -> int:
        return len(self.buffer)
