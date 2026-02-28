from typing import Protocol


class Arm(Protocol):
    def moveL(self, target_eef_pose, target_time) -> None:
        """Move arm eef pose (position and axis-angle rotation)."""
        ...

    def moveJ(self, target_joint_q, target_time) -> None:
        """Move arm joint positions."""
        ...

    def speedL(self, target_eef_twist, target_time) -> None:
        """Move arm eef twist (linear and angular velocity)."""
        ...

    def speedJ(self, target_joint_qd, target_time) -> None:
        """Move arm joint velocities."""
        ...
