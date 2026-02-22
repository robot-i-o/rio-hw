from typing import Protocol


class Gripper(Protocol):
    def moveG(self, target_pos, target_time) -> None:
        """Move gripper position in normalized range [0, 1] -> [closed, open]."""
        ...
