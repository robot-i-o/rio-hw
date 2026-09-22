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

    The value lives in a ``SharedMemoryManager`` buffer, so it is visible across
    processes and survives being pickled and sent to another process (only the
    shared-memory handle is transferred, not the data). This is a stdlib-only
    replacement for the previous ``atomics``-based implementation and preserves
    the same public API (``load``, ``store``, ``add``).

    Access is lock-free. Each counter is written by a single producer at a time
    (the ring buffer advances one ``add``-ing writer; the queue keeps separate
    single-writer read/write counters), and CPython performs the 8-byte
    ``struct`` pack/unpack on an aligned buffer slot without releasing the GIL
    mid-operation, so reads never observe a torn value. No lock is held, which
    keeps the object picklable for transport over ``multiprocessing`` pipes.
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
        self.store(0)  # initialize

    @property
    def buf(self):
        return self.shm.buf[: self.size]

    def load(self) -> int:
        return self._STRUCT.unpack_from(self.shm.buf, 0)[0]

    def store(self, value: int):
        self._STRUCT.pack_into(self.shm.buf, 0, value & self._MASK)

    def add(self, value: int):
        self.store(self.load() + value)
