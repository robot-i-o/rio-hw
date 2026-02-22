from enum import Enum
from typing import TYPE_CHECKING

import numpy as np

try:
    import pyrobotiqgripper
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        pyrobotiqgripper = None  # type: ignore


class ControlMode(Enum):
    MILLIMETERS = "mm"
    BITS = "bits"


class RobotiqDriver:
    def __init__(
        self,
        port: str = "auto",
        mm_range: tuple = (0, 40),
        calibrate: bool = True,
        control_mode: str = "bits",
    ):
        self.port = port
        self.mm_range = mm_range
        self.calibrate = calibrate
        self.control_mode = ControlMode(control_mode)

        if self.port == "auto":
            # TODO: debug auto port detection to free up other serial devices
            print(
                "Using 'auto' port can lead to locks in the whole serial bus. It is recommended to specify the exact port!!!",
            )

    def _move_gripper(self, cmd: float, mode=ControlMode.BITS, speed: int = 255, force: int = 255):
        """
        Moves gripper in a non-blocking way.
        """
        if mode == ControlMode.BITS:
            pos = int(255 - cmd * 255)
            self.gripper.write_registers(
                1000,
                [
                    0b0000100100000000,
                    pos,  # Position
                    speed * 0b100000000 + force,
                ],
            )
        elif mode == ControlMode.MILLIMETERS:
            mm_pos = np.clip(cmd, self.mm_range[0], self.mm_range[1])
            pos = int(self.gripper._mmToBit(mm_pos))
            self.gripper.write_registers(
                1000,
                [
                    0b0000100100000000,
                    pos,  # Position
                    speed * 0b100000000 + force,
                ],
            )

        else:
            raise ValueError(f"Unknown control mode: {mode}")

    def start(self):
        try:
            print(f"Initializing RobotiqGripper on port {self.port}")
            self.gripper = pyrobotiqgripper.RobotiqGripper(portname=self.port)
        except Exception as e:
            raise RuntimeError(f"Failed to initialize RobotiqGripper on port {self.port}") from e
        self.gripper.activate()
        if self.calibrate:
            self.gripper.calibrate(self.mm_range[0], self.mm_range[1])

    def stop(self):
        try:
            self.gripper.reset()
        except Exception:
            self.gripper = None

    def is_grasping(self):
        finger_states = self.gripper.paramDic["gOBJ"]
        if finger_states == 1 or finger_states == 2:
            return True
        else:
            return False

    def state(self):
        pos = self.gripper.getPosition()
        pos = 1 - pos / 255
        robot_state = {
            "gripper_position": pos,
        }
        return robot_state

    def moveG(self, target_pos):
        self._move_gripper(target_pos, mode=self.control_mode)
