from typing import TYPE_CHECKING

import numpy as np

try:
    import dynamixel_sdk

    from .leap_hand_dynamixel_client import DynamixelClient
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        dynamixel_sdk = None  # type: ignore

        DynamixelClient = None  # type: ignore


# Servo-specific mappings and limits
TORQUE_TO_CURRENT_MAPPING = {
    "XC330_M288_T": 1000.0 / 1.0,
    "XL330_M288_T": 1000.0 / 1.0,
    "XC330_T288_T": 1000.0 / 1.0,
    "XM430_W210_T": 1000 / 2.69,
}


# Servo specifications for current limits (in mA)
SERVO_CURRENT_LIMITS = {
    "XC330_M288_T": 2352,
    "XL330_M288_T": 1750,
    "XC330_T288_T": 910,
    "XM430_W210_T": 1193,
}


class LeapHandV1Driver:
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 4000000,
        motor: str = "XC330_M288_T",
        kP: int = 600,
        kI: int = 0,
        kD: int = 200,
        curr_lim: int | None = None,
        latency_timer: int = 1,
    ):
        if curr_lim is None:
            if motor == "XC330_M288_T":  # full
                curr_lim = 550
            elif motor == "XL330_M288_T":  # lite
                curr_lim = 350
            else:
                raise ValueError(motor)
        self.port = port
        self.baudrate = baudrate
        self.motor = motor
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.curr_lim = curr_lim
        self.motors = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

    def start(self):
        dynamixel_sdk.port_handler.LATENCY_TIMER = self.latency_timer

        motors = self.motors
        self.dxl_client = DynamixelClient(motors, self.port, self.baudrate)
        self.dxl_client.connect()

        # Enables position-current control mode and the default parameters
        # it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2)  # Pgain stiffness
        # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write([0, 4, 8], np.ones(3) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2)  # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2)  # Dgain damping
        # Dgain damping for side to side should be a bit less
        self.dxl_client.sync_write([0, 4, 8], np.ones(3) * (self.kD * 0.75), 80, 2)
        # Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def close(self):
        self.dxl_client.disconnect()

    def state(self):
        pos, vel, cur = self.read_pos_vel_cur()
        torque = cur * TORQUE_TO_CURRENT_MAPPING[self.motor]
        return {
            "joint_q": np.array(pos),
            "joint_qd": np.array(vel),
            "joint_current": np.array(cur),
            "joint_torque": np.array(torque),
        }

    def moveJ(self, joint_q):
        return self.set_leap(joint_q)

    # Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    # read position of the robot
    def read_pos(self):
        return self.dxl_client.read_pos()

    # read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()

    # read current
    def read_cur(self):
        return self.dxl_client.read_cur()

    # These combined commands are faster FYI and return a list of data
    def pos_vel(self):
        return self.dxl_client.read_pos_vel()

    # These combined commands are faster FYI and return a list of data
    def pos_vel_eff_srv(self):
        return self.dxl_client.read_pos_vel_cur()
