import pickle
from typing import Protocol

try:
    import cloudpickle
except ImportError:
    cloudpickle = None

try:
    import ormsgpack
except ImportError:
    ormsgpack = None


class Serializer(Protocol):
    @staticmethod
    def pack(data) -> bytes:
        pass

    @staticmethod
    def unpack(b_data: bytes):
        pass


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
