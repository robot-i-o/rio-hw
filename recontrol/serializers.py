import pickle
from typing import TYPE_CHECKING, Protocol

try:
    import cloudpickle
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        cloudpickle = None  # type: ignore

try:
    import ormsgpack
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        ormsgpack = None  # type: ignore


class Serializer(Protocol):
    @staticmethod
    def pack(data) -> bytes: ...

    @staticmethod
    def unpack(b_data: bytes): ...


class CloudpickleSerializer:
    @staticmethod
    def pack(data) -> bytes:
        return cloudpickle.dumps(data)

    @staticmethod
    def unpack(b_data: bytes):
        return cloudpickle.loads(b_data)


class OrmsgpackSerializer:
    @staticmethod
    def pack(data) -> bytes:
        return ormsgpack.packb(data, option=ormsgpack.OPT_SERIALIZE_NUMPY)

    @staticmethod
    def unpack(b_data: bytes):
        return ormsgpack.unpackb(b_data)


class PickleSerializer:
    @staticmethod
    def pack(data) -> bytes:
        return pickle.dumps(data)

    @staticmethod
    def unpack(b_data: bytes):
        return pickle.loads(b_data)
