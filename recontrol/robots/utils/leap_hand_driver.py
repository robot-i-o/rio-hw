import time
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


class LeapHandUtils:
    """
    Embodiments:

    LEAPhand: Real LEAP hand (180 for the motor is actual zero)
    LEAPsim:  Leap hand in sim (has allegro-like zero positions)
    one_range: [-1, 1] for all joints to facilitate RL
    allegro:  Allegro hand in real or sim
    """

    # Safety clips all joints so nothing unsafe can happen. Highly recommend using this before commanding
    @staticmethod
    def angle_safety_clip(joints):
        sim_min, sim_max = LeapHandUtils.LEAPsim_limits()
        real_min = LeapHandUtils.LEAPsim_to_LEAPhand(sim_min)
        real_max = LeapHandUtils.LEAPsim_to_LEAPhand(sim_max)
        return np.clip(joints, real_min, real_max)

    # Sometimes it's useful to constrain the thumb more heavily(you have to implement here), but regular usually works good.
    @staticmethod
    def LEAPsim_limits(type="regular"):
        if type == "regular":
            sim_min = -1.0 * np.array(
                [1.047, 0.314, 0.506, 0.366, 1.047, 0.314, 0.506, 0.366, 1.047, 0.314, 0.506, 0.366, 0.349, 0.47, 1.20, 1.34]
            )
            sim_max = np.array(
                [1.047, 2.23, 1.885, 2.042, 1.047, 2.23, 1.885, 2.042, 1.047, 2.23, 1.885, 2.042, 2.094, 2.443, 1.90, 1.88]
            )
        return sim_min, sim_max

    # this goes from [-1, 1] to [lower, upper]
    @staticmethod
    def scale(x, lower, upper):
        return 0.5 * (x + 1.0) * (upper - lower) + lower

    # this goes from [lower, upper] to [-1, 1]
    @staticmethod
    def unscale(x, lower, upper):
        return (2.0 * x - upper - lower) / (upper - lower)

    # -----------------------------------------------------------------------------------
    # Isaac has custom ranges from -1 to 1 so we convert that to LEAPHand real world
    @staticmethod
    def sim_ones_to_LEAPhand(joints, hack_thumb=False):
        sim_min, sim_max = LeapHandUtils.LEAPsim_limits(type=hack_thumb)
        joints = LeapHandUtils.scale(joints, sim_min, sim_max)
        joints = LeapHandUtils.LEAPsim_to_LEAPhand(joints)
        return joints

    # LEAPHand real world to Isaac has custom ranges from -1 to 1
    @staticmethod
    def LEAPhand_to_sim_ones(joints, hack_thumb=False):
        joints = LeapHandUtils.LEAPhand_to_LEAPsim(joints)
        sim_min, sim_max = LeapHandUtils.LEAPsim_limits(type=hack_thumb)
        joints = LeapHandUtils.unscale(joints, sim_min, sim_max)
        return joints

    # -----------------------------------------------------------------------------------
    # Sim LEAP hand to real leap hand  Sim is allegro-like but all 16 joints are usable.
    @staticmethod
    def LEAPsim_to_LEAPhand(joints):
        joints = np.array(joints)
        ret_joints = joints + 3.14159
        return ret_joints

    # Real LEAP hand to sim leap hand  Sim is allegro-like but all 16 joints are usable.
    @staticmethod
    def LEAPhand_to_LEAPsim(joints):
        joints = np.array(joints)
        ret_joints = joints - 3.14159
        return ret_joints

    # -----------------------------------------------------------------------------------
    # Converts allegrohand radians to LEAP (radians)
    # Only converts the joints that match, all 4 of the thumb and the outer 3 for each of the other fingers
    # All the clockwise/counterclockwise signs are the same between the two hands. Just the offset (mostly 180 degrees off)
    @staticmethod
    def allegro_to_LEAPhand(joints, teleop=False, zeros=True):
        joints = np.array(joints)
        ret_joints = joints + 3.14159
        if zeros:
            ret_joints[0] = ret_joints[4] = ret_joints[8] = 3.14
        if teleop:
            ret_joints[12] = joints[12] + 0.2
            ret_joints[14] = joints[14] - 0.2
        return ret_joints

    # Converts LEAP to allegrohand (radians)
    @staticmethod
    def LEAPhand_to_allegro(joints, teleop=False, zeros=True):
        joints = np.array(joints)
        ret_joints = joints - 3.14159
        if zeros:
            ret_joints[0] = ret_joints[4] = ret_joints[8] = 0
        if teleop:
            ret_joints[12] = joints[12] - 0.2
            ret_joints[14] = joints[14] + 0.2
        return ret_joints


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
        self.latency_timer = latency_timer
        self.motors = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        self.num_joints = len(self.motors)

    def start(self):
        dynamixel_sdk.port_handler.LATENCY_TIMER = self.latency_timer

        motors = self.motors
        self.dxl_client = DynamixelClient(motors, self.port, self.baudrate)
        self.dxl_client.connect()
        self.prev_pos = self.curr_pos = LeapHandUtils.allegro_to_LEAPhand(np.zeros(self.num_joints))

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

        time.sleep(2.0)  # wait for hand to move to initial position

    def stop(self):
        self.dxl_client.disconnect(force=True)

    def state(self):
        # pos = self.read_pos()
        # vel, cur = np.zeros(self.num_joints), np.zeros(self.num_joints)

        # pos, vel = self.pos_vel()
        # cur = np.zeros(self.num_joints)

        pos, vel, cur = self.pos_vel_eff_srv()

        pos = LeapHandUtils.LEAPhand_to_allegro(pos, zeros=False)
        # convert mA -> A -> Nm
        cur = cur / 1000.0
        torque = cur * TORQUE_TO_CURRENT_MAPPING[self.motor]

        s = {
            "joint_q": np.array(pos),
            "joint_qd": np.array(vel),
            "joint_current": np.array(cur),
            "joint_torque": np.array(torque),
        }
        return s

    def moveJ(self, joint_q, wait=False):
        # self.set_leap(joint_q)
        self.set_allegro(joint_q)
        # self.set_ones(joint_q)
        if wait:
            time.sleep(2.0)  # wait for hand to move

    # Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    # allegro compatibility joint angles.  It adds 180 to make the fully open position at 0 instead of 180
    def set_allegro(self, pose):
        pose = LeapHandUtils.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    # Sim compatibility for policies, it assumes the ranges are [-1,1] and then convert to leap hand ranges.
    def set_ones(self, pose):
        pose = LeapHandUtils.sim_ones_to_LEAPhand(np.array(pose))
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
