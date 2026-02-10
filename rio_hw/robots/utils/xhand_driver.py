import time
from typing import TYPE_CHECKING

import numpy as np

try:
    from xhand_controller import xhand_control
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        xhand_control = None  # type: ignore


# Ignore known hardware sensor errors that don't affect movement
IGNORED_ERRORS = [
    "Sensor fails to read the combined force",
    "Sensor fails to read the distributed force",
    "Sensor fails to read temperature",
    "Communication data CRC error",
    "This hardware version does not support force control mode",
]


class XhandDriver:
    """
    Refs:
    - https://github.com/wengmister/LeFranX/blob/main/src/lerobot/robots/xhand/xhand.py
    - https://github.com/real-stanford/DexUMI/blob/main/dexumi/hand_sdk/xhand/hand_api_cls.py
    - https://github.com/DinoMini00/KineDex_code/blob/main/diffusion_policy/real_world/xhand_interpolation_controller.py
    """

    def __init__(
        self,
        hand_id=0,
        port="/dev/ttyUSB0",
        protocol="RS485",
        mode=3,  # 0: powerless, 3: position control mode, 5: force control mode, 19: guide mode
        num_joints=12,
        # tor_max=300,
        # kp=100,
        tor_max=400,
        kp=150,
        ki=0,
        kd=0,
    ):
        assert protocol in ("RS485", "EtherCAT")
        self._hand_id = hand_id
        self.port = port
        self.protocol = protocol
        self.mode = mode
        self.num_joints = num_joints
        self.tor_max = tor_max
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def start(self):
        self._device = xhand_control.XHandControl()
        self._hand_command = xhand_control.HandCommand_t()
        for i in range(self.num_joints):
            self._hand_command.finger_command[i].position = 0.0  # Start at 0

        device_identifier = {}
        if self.protocol == "RS485":
            device_identifier["protocol"] = self.protocol
            device_identifier["serial_port"] = self.port
            device_identifier["baud_rate"] = 3000000
        elif self.protocol == "EtherCAT":
            device_identifier["protocol"] = self.protocol
        else:
            raise ValueError(self.protocol)
        self._open_device(device_identifier)

        self._print_info()
        self.set_hand_mode(self.mode)

    def _open_device(self, device_identifier: dict):
        # RS485
        if device_identifier["protocol"] == "RS485":
            serial_port = self._device.enumerate_devices(self.protocol)
            serial_port = list(filter(lambda x: x.startswith("/dev/ttyUSB"), serial_port))
            print(f"=@= xhand devices port: {serial_port}")

            device_identifier["baud_rate"] = int(device_identifier["baud_rate"])
            rsp = self._device.open_serial(
                device_identifier["serial_port"],
                device_identifier["baud_rate"],
            )
            print(f"=@= open RS485 result: {rsp.error_code == 0}")
        # EtherCAT
        elif device_identifier["protocol"] == "EtherCAT":
            ether_cat = self._device.enumerate_devices("EtherCAT")
            print(f"enumerate_devices_ethercat ether_cat= {ether_cat}")
            if ether_cat is None or not ether_cat:
                print("enumerate_devices_ethercat get empty")

            rsp = self._device.open_ethercat(ether_cat[0])
        else:
            raise ValueError(device_identifier["protocol"])
        if rsp.error_code != 0:
            print(f"=@= open device error: {rsp.error_message}. Please check serial_port and connection")
            raise ConnectionError

    def _print_info(self):
        # # Get hand ID
        # hands_id = self._device.list_hands_id()

        # Read software SDK version
        print(f"=@= xhand software SDK version: {self._device.get_sdk_version()}")

        # Read hardware SDK version
        joint_id = 0
        error_struct, version = self._device.read_version(self._hand_id, joint_id)
        if error_struct.error_code != 0:
            print(f"=@= xhand read_version error: {error_struct.error_message}")
        print(f"=@= xhand hardware SDK version: {version}")

        # Read hand device information
        error_struct, info = self._device.read_device_info(self._hand_id)
        if error_struct.error_code != 0:
            print(f"=@= xhand read_device_info error: {error_struct.error_message}")
        print(f"=@= xhand serial_number: {info.serial_number[0:16]}")  # sn is 16 bytes
        print(f"=@= xhand hand_id: {info.hand_id}")
        print(f"=@= xhand ev_hand: {info.ev_hand}")

        # Get hand left/right type
        error_struct, hand_type = self._device.get_hand_type(self._hand_id)
        if error_struct.error_code != 0:
            print(f"=@= xhand get_hand_type error: {error_struct.error_message}")
        print(f"=@= xhand hand_type: {hand_type}")

        # Read hand serial number
        error_struct, serial_number = self._device.get_serial_number(self._hand_id)
        if error_struct.error_code != 0:
            print(f"=@= xhand get_serial_number error: {error_struct.error_message}")
        print(f"=@= xhand serial_number: {serial_number}")

    def stop(self):
        if self._device is None:
            return
        self.set_hand_mode(0)
        self._device.close_device()
        self._device = None
        self._hand_command = None

    def set_hand_mode(self, mode: int):
        assert mode in (0, 3, 5, 19)
        kp, ki, kd, tor_max = self.kp, self.ki, self.kd, self.tor_max
        if mode in (0, 19):
            kp, ki, kd, tor_max = 0, 0, 0, 0

        for i in range(self.num_joints):
            self._hand_command.finger_command[i].id = i
            self._hand_command.finger_command[i].kp = kp
            self._hand_command.finger_command[i].ki = ki
            self._hand_command.finger_command[i].kd = kd
            self._hand_command.finger_command[i].tor_max = tor_max
            self._hand_command.finger_command[i].mode = mode
        self._send_position_command()
        time.sleep(1.0)
        self.mode = mode

    def state(self, force_update=True):
        error_struct, state = self._device.read_state(self._hand_id, force_update)
        if error_struct.error_code != 0:
            ignored = any(ignored_error in error_struct.error_message for ignored_error in IGNORED_ERRORS)
            if not ignored:
                print(f"=@= xhand read_state error: {error_struct.error_message}")
                return None
        s = {
            "joint_q": [],
            "joint_torque": [],
        }
        for i in range(self.num_joints):
            finger_state = state.finger_state[i]
            s["joint_q"].append(finger_state.position)
            s["joint_torque"].append(finger_state.torque)
        s = {k: np.array(v) for k, v in s.items()}
        return s

    def moveJ(self, joint_q):
        if isinstance(joint_q, np.ndarray):
            joint_q = joint_q.tolist()
        for i in range(self.num_joints):
            self._hand_command.finger_command[i].position = float(joint_q[i])
        return self._send_position_command()

    def _send_position_command(self):
        if self._device is None or self._hand_command is None:
            return False

        error_struct = self._device.send_command(self._hand_id, self._hand_command)
        if error_struct.error_code != 0:
            ignored = any(ignored_error in error_struct.error_message for ignored_error in IGNORED_ERRORS)
            if not ignored:
                print(f"=@= xhand send_command result: {error_struct.error_code == 0}, message: {error_struct.error_message}")
                return False
        return True
