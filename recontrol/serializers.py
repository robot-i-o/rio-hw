import pickle
from typing import Protocol


class Serializer(Protocol):
    def pack(self, data) -> bytes:
        pass

    def unpack(self, b_data: bytes):
        pass


class PickleSerializer:
    @staticmethod
    def pack(data) -> bytes:
        return pickle.dumps(data)

    @staticmethod
    def unpack(b_data: bytes):
        return pickle.loads(b_data)
