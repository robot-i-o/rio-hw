import struct
import time
from typing import TYPE_CHECKING

try:
    from xarm.wrapper import XArmAPI
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        XArmAPI = None  # type: ignore


class XArmSocket:
    # https://docs.supportarticle.ufactory.cc/support_articles/developer/firmware/how-to-get-the-real-time-data-via-tcp-30000-port.html
    # Frequency: 250HZ (200HZ with FT sensor)
    P30000 = {
        "eef_pose": [473, 496],  # mm & rad, [x,y,z,rx,ry,rz]
        "eef_twist": [497, 520],  # mm/s & rad/s
        "joint_q": [117, 144],  # rad
        "joint_qd": [145, 172],  # rad/s
        "joint_qdd": [173, 200],  # rad/s^2
        "target_eef_pose": [425, 448],  # mm & rad, [x,y,z,rx,ry,rz]
        "target_eef_twist": [449, 472],  # mm/s & rad/s
        "target_joint_q": [33, 60],  # rad
        "target_joint_qd": [61, 88],  # rad/s
        "target_joint_qdd": [89, 116],  # rad/s^2
        "joint_torque": [229, 256],  # N·m
        "eef_torque": [521, 544],  # N & N·m
        "ft_sensor_raw": [689, 712],  # N & N·m, [Fx,Fy,Fz,Tx,Ty,Tz]
        "ft_sensor_filtered": [713, 736],  # N & N·m, [Fx,Fy,Fz,Tx,Ty,Tz]
    }
    PORT = 30000
    FREQ = 250  # Hz
    FREQ_FT = 200  # Hz, with FT sensor attached

    # for get_servo_all_pids()
    SERVO_PIDS = [
        "POS_KP",
        "POS_FWDKP",
        "POS_PWDTC",
        "SPD_KP",
        "SPD_KI",
        "CURR_KP",
        "CURR_KI",
        "SPD_IFILT",
        "SPD_OFILT",
        "CURR_IFILT",
        "POS_KD",
        "POS_CMDILT",
        "GET_TEMP",
        "OVER_TEMP",
    ]

    @staticmethod
    def bytes_to_fp32(bytes_data, is_big_endian=False):
        return struct.unpack(">f" if is_big_endian else "<f", bytes_data)[0]

    @staticmethod
    def bytes_to_fp32_list(bytes_data, n=0, is_big_endian=False):
        ret = []
        count = n if n > 0 else len(bytes_data) // 4
        for i in range(count):
            ret.append(XArmSocket.bytes_to_fp32(bytes_data[i * 4 : i * 4 + 4], is_big_endian))
        return ret

    @staticmethod
    def bytes_to_u8(data):
        data_u8_0 = int.from_bytes(data[:], byteorder="little")
        return data_u8_0

    @staticmethod
    def bytes_to_u16(data):
        data_u16 = data[0] << 8 | data[1]
        return data_u16

    @staticmethod
    def bytes_to_u32(data):
        data_u32 = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]
        return data_u32

    @staticmethod
    def bytes_to_u64(data):
        if len(data) != 8:
            raise ValueError("Input data must be exactly 8 bytes long")
        u64 = struct.unpack(">Q", data)[0]
        return u64

    @staticmethod
    def bytes_to_state(data, num_joints):
        state = {}
        for key, (start, end) in XArmSocket.P30000.items():
            value = XArmSocket.bytes_to_fp32_list(data[start - 1 : end])
            if "joint" in key:
                value = value[:num_joints]
            state[key] = value
        return state


class XArmGripperDriver:
    def __init__(self, robot_ip, robot_model: str, home_to_open: bool = True):
        assert robot_model in ("lite6", "g1", "g2", "robotiq_2f85", "robotiq_2f140")
        self.robot_ip = robot_ip
        self.robot_model = robot_model
        self.home_to_open = home_to_open

    def start(self):
        arm = XArmAPI(self.robot_ip, is_radian=True, do_not_open=True)
        arm.connect()
        arm.clean_error()
        arm.clean_warn()
        arm.motion_enable(True)
        if arm.has_err_warn:
            _, err_warn = arm.get_err_warn_code()
            if err_warn[0] != 0:
                raise RuntimeError("Check whether e-stop button is pressed.")

        self.gripper = arm
        if self.robot_model == "lite6":
            # self.gripper.set_mode(0)
            self.gripper.set_state(0)
        elif self.robot_model in ("g1", "g2"):
            self.gripper.set_gripper_mode(0)
            self.gripper.set_gripper_enable(True)
            # self.gripper.set_collision_tool_model(1)
        elif self.robot_model in ("robotiq_2f85", "robotiq_2f140"):
            self.gripper.set_mode(0)
            self.gripper.set_state(0)
            self.gripper.robotiq_reset()
            self.gripper.robotiq_set_activate()
        else:
            raise ValueError(self.robot_model)
        time.sleep(0.1)

        if self.home_to_open:
            self.moveG(1.0, wait=True)  # open

    def stop(self):
        if self.robot_model == "lite6":
            self.gripper.stop_lite6_gripper()
        self.gripper.disconnect()

    def state(self):
        # get state from robot
        if self.robot_model == "lite6":
            pos = self._lite6_gripper_pos
            robot_state = {
                "gripper_position": pos,
            }
        elif self.robot_model == "g1":
            _, pos = self.gripper.get_gripper_position()
            # [-10, 850] -> [0, 1]
            pos = (pos + 10) / 860
            robot_state = {
                "gripper_position": pos,
            }
        elif self.robot_model == "g2":
            _, pos = self.gripper.get_gripper_g2_position()
            # [0, 84] -> [0, 1]
            pos = pos / 84
            robot_state = {
                "gripper_position": pos,
            }
        elif self.robot_model in ("robotiq_2f85", "robotiq_2f140"):
            _, result = self.gripper.robotiq_get_status()
            pos = result[6]  # [0, 255]
            # [255, 0] -> [0, 1]
            pos = 1 - pos / 255  # 0 is open and 255 is closed
            robot_state = {
                "gripper_position": pos,
            }
        else:
            raise ValueError(self.robot_model)
        return robot_state

    def moveG(self, target_pos: float, wait=False):
        target_pos = max(0.0, min(1.0, target_pos))  # clamp to [0, 1] range
        if self.robot_model == "lite6":
            # assert self.gripper.mode == 0
            if target_pos > 0.5:
                self.gripper.open_lite6_gripper(sync=wait)
                self._lite6_gripper_pos = 1.0
            else:
                self.gripper.close_lite6_gripper(sync=wait)
                self._lite6_gripper_pos = 0.0
        elif self.robot_model == "g1":
            # [0, 1] -> [-10, 850]
            pos = int(target_pos * 860 - 10)
            self.gripper.set_gripper_position(pos, speed=5000, wait=wait)  # speed: [0, 5000]
        elif self.robot_model == "g2":
            # [0, 1] -> [0, 84]
            pos = int(target_pos * 84)
            self.gripper.set_gripper_g2_position(pos, speed=225, force=50, wait=wait)  # speed: [15, 225], force: [1, 100]
        elif self.robot_model in ("robotiq_2f85", "robotiq_2f140"):
            # [0, 1] -> [255, 0]
            pos = int(255 - target_pos * 255)  # 0 is open and 255 is closed
            self.gripper.robotiq_set_position(pos, speed=255, force=255, wait=wait)  # speed: [0, 255], force: [0, 255]
        else:
            raise ValueError(self.robot_model)
