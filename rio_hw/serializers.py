import pickle
import sys
from typing import TYPE_CHECKING, Protocol

import numpy as np

try:
    import cloudpickle
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        cloudpickle = None  # type: ignore

try:
    import msgpack
    import msgpack_numpy

    msgpack_numpy.patch()
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        msgpack = None  # type: ignore
        msgpack_numpy = None  # type: ignore

try:
    import ormsgpack
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        ormsgpack = None  # type: ignore


class CloudpickleSerializer:
    @staticmethod
    def pack(data) -> bytes:
        return cloudpickle.dumps(data)

    @staticmethod
    def unpack(b_data: bytes):
        return cloudpickle.loads(b_data)


class MsgpackSerializer:
    @staticmethod
    def _make_writeable(obj):
        """Recursively make all numpy arrays in a deserialized object writable."""
        if isinstance(obj, np.ndarray):
            if not obj.flags.writeable:
                return obj.copy()
            return obj
        if isinstance(obj, dict):
            return {k: MsgpackSerializer._make_writeable(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return type(obj)(MsgpackSerializer._make_writeable(item) for item in obj)
        return obj

    @staticmethod
    def pack(data) -> bytes:
        return msgpack.packb(data)

    @staticmethod
    def unpack(b_data: bytes):
        return MsgpackSerializer._make_writeable(msgpack.unpackb(b_data))


class OrmsgpackSerializer:
    @staticmethod
    def _encode(obj):
        if isinstance(obj, np.ndarray):
            return {"__np__": obj.tobytes(), "d": str(obj.dtype), "s": obj.shape}
        if isinstance(obj, dict):
            return {k: OrmsgpackSerializer._encode(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [OrmsgpackSerializer._encode(v) for v in obj]
        return obj

    @staticmethod
    def _decode(obj):
        if isinstance(obj, dict):
            if "__np__" in obj:
                return np.frombuffer(obj["__np__"], dtype=obj["d"]).reshape(obj["s"]).copy()
            return {k: OrmsgpackSerializer._decode(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [OrmsgpackSerializer._decode(v) for v in obj]
        return obj

    @staticmethod
    def pack(data) -> bytes:
        return ormsgpack.packb(OrmsgpackSerializer._encode(data))

    @staticmethod
    def unpack(b_data: bytes):
        return OrmsgpackSerializer._decode(ormsgpack.unpackb(b_data))


class PickleSerializer:
    @staticmethod
    def pack(data) -> bytes:
        return pickle.dumps(data)

    @staticmethod
    def unpack(b_data: bytes):
        return pickle.loads(b_data)


__all__ = [
    "CloudpickleSerializer",
    "MsgpackSerializer",
    "OrmsgpackSerializer",
    "PickleSerializer",
]


class Serializer(Protocol):
    @staticmethod
    def pack(data) -> bytes: ...

    @staticmethod
    def unpack(b_data: bytes): ...

    def make(name: str = "pickle"):
        cls = getattr(sys.modules[__name__], f"{name.capitalize()}Serializer", None)
        if cls is None:
            raise ValueError(name)
        return cls
