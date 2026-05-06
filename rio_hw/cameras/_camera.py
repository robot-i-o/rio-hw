from typing import Protocol


class Camera(Protocol):
    def __init__(
        self,
        serial: int | str,
        model: str,
        resolution: tuple[int, int] | None = (720, 1280),
        resolution_depth: tuple[int, int] | None = None,
        enable_color: bool = True,
        enable_depth: bool = False,
        bgr: bool = False,
    ): ...

    def set_default_settings(self): ...
