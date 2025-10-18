import os
import queue
import socket
import struct
from enum import Enum, auto

import numpy as np

from .. import time
from ..filters import LowPassFilter
from ..middleware import ClientFactory, ServerFactory
from ..pose_trajectory_interpolator import PoseTrajectoryInterpolator
from ..request import Request

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    XArmAPI = None


class ArmModel(Enum):
    XARM6 = auto()
    XARM7 = auto()
    XARM850 = auto()
    LITE6 = auto()


ArmInfo = {
    ArmModel.XARM6: {"num_joints": 6},
    ArmModel.XARM7: {"num_joints": 7},
    ArmModel.XARM850: {"num_joints": 6},
    ArmModel.LITE6: {"num_joints": 6},
}


class ArmController(Enum):
    TASK_POS = auto()
    JOINT_POS = auto()
    TASK_VEL = auto()
    JOINT_VEL = auto()


class RequestType(Enum):
    MOVEL = auto()
    MOVEJ = auto()
    SPEEDL = auto()
    SPEEDJ = auto()


class XArm:
    __api__ = [
        "get_state",
        "get_all_state",
        "moveL",
        "moveJ",
        "speedL",
        "speedJ",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        robot_ip: str = "192.168.1.111",
        robot_model: str = "xarm7",
        robot_controller: str = "task_pos",
        max_pos_speed=0.25,  # 25% of max speed, 1 m/s
        max_rot_speed=0.785,  # 25% of max speed, 180 deg/s
        tcp_offset_pose=None,
        payload_mass=None,
        payload_cog=None,
        joints_init=None,
        joints_init_speed=1.05,
        joint_lowpass_alpha=0.1,
        soft_real_time=False,
        dtype=np.float64,
        *,
        freq: int = 250,
        max_buffer_size: int | None = None,
        **kwargs,
    ):
        """
        Args:
            max_pos_speed: m/s
            max_rot_speed: rad/s
            tcp_offset_pose: 6d pose
            payload_mass: float
            payload_cog: 3d position, center of gravity
            joint_init:
            joint_init_speed:
            joint_lowpass_alpha:
            soft_real_time: enables round-robin scheduling and real-time priority
            dtype:
        """
        assert 0 < freq <= 250
        assert 0 < max_pos_speed
        assert 0 < max_rot_speed
        robot_model = ArmModel[robot_model.upper()]
        robot_controller = ArmController[robot_controller.upper()]
        num_joints = ArmInfo[robot_model]["num_joints"]
        if max_buffer_size is None:
            max_buffer_size = int(freq * 5)
        if tcp_offset_pose is not None:
            tcp_offset_pose = np.array(tcp_offset_pose, dtype=dtype)
            assert tcp_offset_pose.shape == (6,)
        if payload_mass is not None:
            assert 0 <= payload_mass <= 5
        if payload_cog is not None:
            payload_cog = np.array(payload_cog, dtype=dtype)
            assert payload_cog.shape == (3,)
            assert payload_mass is not None
        if joints_init is not None:
            joints_init = np.array(joints_init, dtype=dtype)
            assert joints_init.shape == (num_joints,)
        self.robot_ip = robot_ip
        self.robot_model = robot_model
        self.robot_controller = robot_controller
        self.num_joints = num_joints
        self.max_pos_speed = max_pos_speed
        self.max_rot_speed = max_rot_speed
        self.tcp_offset_pose = tcp_offset_pose
        self.payload_mass = payload_mass
        self.payload_cog = payload_cog
        self.joints_init = joints_init
        self.joints_init_speed = joints_init_speed
        self.joint_lowpass_alpha = joint_lowpass_alpha
        self.soft_real_time = soft_real_time
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_request_params = {
            ArmController.TASK_POS: (RequestType.MOVEL, {"target_pose": np.zeros((6,), dtype=self.dtype)}),
            ArmController.JOINT_POS: (RequestType.MOVEJ, {"target_jointq": np.zeros((self.num_joints,), dtype=self.dtype)}),
            ArmController.TASK_VEL: (RequestType.SPEEDL, {"target_twist": np.zeros((6,), dtype=self.dtype)}),
            ArmController.JOINT_VEL: (RequestType.SPEEDJ, {"target_jointqd": np.zeros((self.num_joints,), dtype=self.dtype)}),
        }[self.robot_controller][1]
        example_request_params = {
            **example_request_params,
            "target_time": time.now(),
        }

        dummy_data = bytes(1000)  # dummy data, just needs to be larger than 784
        example_robot_state = XArmSocket.bytes_to_state(dummy_data)
        example_robot_state = {k: np.array(v, dtype=self.dtype) for k, v in example_robot_state.items()}

        self.example_request = {
            "type": next(iter(RequestType)).value,
            **example_request_params,
        }
        self.example_data = {
            **example_robot_state,
            "timestamp": time.now(),
        }
        self.worker = self.pub
        self.run = self.req
        super().__post_init__()

        self.arm = XArmAPI(self.robot_ip, is_radian=True, report_type="real", do_not_open=True)

    def pub(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setblocking(True)
        sock.settimeout(1)
        try:
            sock.connect((self.robot_ip, XArmSocket.PORT))

            buffer = sock.recv(4)
            while len(buffer) < 4:
                buffer += sock.recv(4 - len(buffer))
            size = XArmSocket.bytes_to_u32(buffer[:4])

            # Main loop
            rate = time.Rate(XArmSocket.FREQ)
            not_pub_ready = True
            while not self.exit_event.is_set():
                buffer += sock.recv(size - len(buffer))
                if len(buffer) < size:
                    continue
                data = buffer[:size]
                buffer = buffer[size:]

                robot_state = XArmSocket.bytes_to_state(data)
                robot_state = {k: np.array(v, dtype=self.dtype) for k, v in robot_state.items()}
                robot_state["TargetTCPPose"][:3] *= 0.001  # convert mm to m
                robot_state["ActualTCPPose"][:3] *= 0.001  # convert mm to m
                robot_state["TargetTCPSpeed"][:3] *= 0.001  # convert mm/s to m/s
                robot_state["ActualTCPSpeed"][:3] *= 0.001  # convert mm/s to m/s

                # Store current state in ring buffer
                data = {
                    **robot_state,
                    "timestamp": time.now(),
                }
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            sock.close()

    def req(self):
        try:
            # enable soft real-time
            if self.soft_real_time:
                os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(20))

            arm = self.arm
            # https://help.ufactory.cc/en/articles/3954394-guide-to-run-ufactory-xarm-at-the-maximum-speed
            # arm.set_tcp_jerk(7000)
            # arm.set_tcp_maxacc(...)
            # arm.set_joint_jerk(...)
            # arm.set_joint_maxacc(...)
            # arm.set_linear_spd_limit_factor(1.2)
            # arm.set_collision_sensitivity(0)
            # arm.save_conf()
            arm.connect()
            arm.clean_error()
            arm.clean_warn()
            arm.motion_enable(True)
            if arm.has_err_warn:
                _, err_warn = arm.get_err_warn_code()
                if err_warn[0] != 0:
                    raise RuntimeError("Check whether e-stop button is pressed.")
            arm.set_mode(0)
            arm.set_state(0)

            # set parameters
            if self.tcp_offset_pose is not None:
                code = arm.set_tcp_offset(self.tcp_offset_pose)
                assert code == 0
            if self.payload_mass is not None:
                assert self.payload_cog is not None
                code = arm.set_tcp_load(self.payload_mass, self.payload_cog)
                assert code == 0

            # init pose
            if self.joints_init is not None:
                code = arm.set_servo_angle(angle=self.joints_init, speed=self.joints_init_speed, mvacc=1.4, wait=True)
                assert code == 0
            # arm.reset(wait=True)
            # arm.move_gohome(wait=True)

            if self.robot_controller == ArmController.TASK_POS:
                arm.set_mode(1)  # 1: servo motion mode
            elif self.robot_controller == ArmController.JOINT_POS:
                arm.set_mode(1)  # 1: servo motion mode
            else:
                raise ValueError(self.robot_controller)
            arm.set_state(0)
            time.sleep(0.1)

            if self.robot_controller == ArmController.TASK_POS:
                code, curr_pose = arm.get_position_aa()
                assert code == 0
                curr_pose = np.array(curr_pose, dtype=self.dtype)
                curr_pose[:3] *= 0.001  # convert mm to m
                # pose interpolation
                curr_t = time.now()
                last_waypoint_time = curr_t
                pose_interp = PoseTrajectoryInterpolator(times=[curr_t], poses=[curr_pose])
            elif self.robot_controller == ArmController.JOINT_POS:
                code, curr_jointq = arm.get_servo_angle()
                assert code == 0
                curr_jointq = np.array(curr_jointq[: self.num_joints], dtype=self.dtype)
                # joint filtering/smoothing
                target_jointq = curr_jointq
                lowpass_filter = LowPassFilter(alpha=self.joint_lowpass_alpha, initial=curr_jointq)
            else:
                raise ValueError(self.robot_controller)

            # Main loop
            dt = 1.0 / self.freq
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            while not self.exit_event.is_set():
                t_now = time.now()
                # send command to robot
                if self.robot_controller == ArmController.TASK_POS:
                    pose_command = pose_interp(t_now)
                    pose_command[:3] *= 1000.0  # convert m to mm
                    code = arm.set_servo_cartesian_aa(pose_command.tolist(), speed=100, mvacc=200)  # mode=1
                elif self.robot_controller == ArmController.JOINT_POS:
                    jointq_command = lowpass_filter(target_jointq)
                    code = arm.set_servo_angle_j(jointq_command.tolist(), is_radian=True)
                else:
                    raise ValueError(self.robot_controller)
                # if not (code == 0 and arm.error_code == 0 and arm.connected):
                #     raise RuntimeError

                # Fetch request from queue
                try:
                    req = self.request_queue.get()
                    if isinstance(req, dict):
                        req = Request(RequestType(req.pop("type")), req)
                except queue.Empty:
                    req = None
                if req:
                    if req.type == RequestType.MOVEL:
                        target_pose = np.array(req.params.get("target_pose"), dtype=self.dtype)
                        target_time = float(req.params.get("target_time"))
                        curr_time = t_now + dt
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=target_pose,
                            time=target_time,
                            max_pos_speed=self.max_pos_speed,
                            max_rot_speed=self.max_rot_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time,
                        )
                        last_waypoint_time = target_time
                    elif req.type == RequestType.MOVEJ:
                        target_jointq = np.array(req.params.get("target_jointq"), dtype=self.dtype)
                    elif req.type == RequestType.SPEEDL:
                        raise NotImplementedError
                    elif req.type == RequestType.SPEEDJ:
                        raise NotImplementedError
                    else:
                        raise ValueError(req.type)
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            # decelerate
            _, angles = arm.get_servo_angle()
            zero_angle_delta = [0.0] * len(angles)
            arm.set_servo_angle(angle=zero_angle_delta, wait=True, relative=True, timeout=0.5)

            # terminate
            arm.set_mode(0)
            arm.set_state(0)
            # arm.reset(wait=True)
            # arm.move_gohome(wait=True)
            arm.disconnect()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def moveL(self, target_pose, target_time):
        target_pose = np.array(target_pose, dtype=self.dtype)
        assert target_pose.shape == (6,)
        assert target_time > time.now()
        req = {
            "type": RequestType.MOVEL.value,
            "target_pose": target_pose,
            "target_time": target_time,
        }
        self.request_queue.put(req)

    def moveJ(self, target_jointq, target_time):
        target_jointq = np.array(target_jointq, dtype=self.dtype)
        assert target_jointq.shape == (self.num_joints,)
        assert target_time > time.now()
        req = {
            "type": RequestType.MOVEJ.value,
            "target_jointq": target_jointq,
            "target_time": target_time,
        }
        self.request_queue.put(req)

    def speedL(self, target_twist, target_time):
        target_twist = np.array(target_twist, dtype=self.dtype)
        assert target_twist.shape == (6,)
        assert target_time > time.now()
        req = {
            "type": RequestType.SPEEDL.value,
            "target_twist": target_twist,
            "target_time": target_time,
        }
        self.request_queue.put(req)

    def speedJ(self, target_jointqd, target_time):
        target_jointqd = np.array(target_jointqd, dtype=self.dtype)
        assert target_jointqd.shape == (self.num_joints,)
        assert target_time > time.now()
        req = {
            "type": RequestType.SPEEDJ.value,
            "target_jointqd": target_jointqd,
            "target_time": target_time,
        }
        self.request_queue.put(req)


class XArmSocket:
    # https://docs.supportarticle.ufactory.cc/support_articles/developer/firmware/how-to-get-the-real-time-data-via-tcp-30000-port.html
    # Frequency: 250HZ (200HZ with FT sensor)
    P30000 = {
        "ActualTCPPose": [473, 496],  # mm & rad, [x,y,z,rx,ry,rz]
        "ActualTCPSpeed": [497, 520],  # mm/s & rad/s
        "ActualQ": [117, 144],  # rad
        "ActualQd": [449, 472],  # rad/s
        "TargetTCPPose": [425, 448],  # mm & rad, [x,y,z,rx,ry,rz]
        "TargetTCPSpeed": [449, 472],  # mm/s & rad/s
        "TargetQ": [33, 60],  # rad
        "TargetQd": [61, 88],  # rad/s
    }
    PORT = 30000
    FREQ = 250

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
    def bytes_to_state(data):
        state = {}
        for key, (start, end) in XArmSocket.P30000.items():
            state[key] = XArmSocket.bytes_to_fp32_list(data[start - 1 : end])
        return state


def XArmServer(mw, *args, **kwargs):
    return ServerFactory(mw, XArm, *args, **kwargs)


def XArmClient(mw, *args, **kwargs):
    return ClientFactory(mw, XArm, *args, **kwargs)
