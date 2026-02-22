from typing import Protocol


class Arm(Protocol):
    def moveL(self, target_tcp_pose, target_time) -> None:
        """Move arm tcp pose (position and axis-angle rotation)."""
        ...

    def moveJ(self, target_joint_q, target_time) -> None:
        """Move arm joint positions."""
        ...

    def speedL(self, target_tcp_twist, target_time) -> None:
        """Move arm tcp velocities (twist)."""
        ...

    def speedJ(self, target_joint_qd, target_time) -> None:
        """Move arm joint velocities."""
        ...
