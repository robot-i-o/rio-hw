import multiprocessing as mp
import struct
from dataclasses import dataclass
from multiprocessing.managers import SharedMemoryManager

import numpy as np


@dataclass
class ArraySpec:
    name: str
    shape: tuple[int]
    dtype: np.dtype


class SharedAtomicCounter:
    """A shared-memory 64-bit unsigned integer counter.

    Backed by a ``SharedMemoryManager`` buffer so the value is visible across
    processes, with a ``multiprocessing.Lock`` guarding read-modify-write
    operations to keep them atomic. This is a stdlib-only replacement for the
    previous ``atomics``-based implementation and preserves the same public API
    (``load``, ``store``, ``add``).
    """

    # Little-endian unsigned 64-bit integer.
    _STRUCT = struct.Struct("<Q")
    _MASK = (1 << 64) - 1

    def __init__(
        self,
        shm_manager: SharedMemoryManager,
        size: int = 8,  # 64bit int
    ):
        assert size >= self._STRUCT.size, f"size must be >= {self._STRUCT.size} bytes for a 64-bit counter"
        shm = shm_manager.SharedMemory(size=size)
        self.shm = shm
        self.size = size
        self._lock = mp.Lock()
        self.store(0)  # initialize

    @property
    def buf(self):
        return self.shm.buf[: self.size]

    def _read(self) -> int:
        return self._STRUCT.unpack_from(self.shm.buf, 0)[0]

    def _write(self, value: int):
        self._STRUCT.pack_into(self.shm.buf, 0, value & self._MASK)

    def load(self) -> int:
        with self._lock:
            return self._read()

    def store(self, value: int):
        with self._lock:
            self._write(value)

    def add(self, value: int):
        with self._lock:
            self._write(self._read() + value)
