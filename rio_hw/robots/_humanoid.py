from typing import Protocol


class Humanoid(Protocol):
    def moveJ(self, target_joint_q, target_time) -> None:
        """Move humanoid joint positions."""
        ...
