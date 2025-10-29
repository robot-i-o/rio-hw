import uuid
from dataclasses import dataclass
from enum import Enum


@dataclass
class Request:
    """Generic request container."""

    type: Enum
    params: dict
    id: str | None = None

    def __post_init__(self):
        if self.id is None:
            self.id = str(uuid.uuid4())
