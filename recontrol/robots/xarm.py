import os
import queue
import socket
import struct
import threading as th
from enum import Enum, auto

import numpy as np
import scipy.spatial.transform as st

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..pose_trajectory_interpolator import PoseTrajectoryInterpolator
from ..request import Request

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    XArmAPI = None


class ArmModel(Enum):
    # XARM5 = "xarm5"
    # XARM6 = "xarm6"
    XARM7 = "xarm7"
    XARM850 = "xarm850"
    LITE6 = "lite6"


class ArmController(Enum):
    # JOINT_POS = "joint_pos"
    # JOINT_VEL = "joint_vel"
    TASK_POS = "task_pos"


class ArmRequestType(Enum):
    SCHEDULE_WAYPOINT = auto()


class XArm:
    __api__ = [
        "get_state",
        "get_all_state",
        "schedule_waypoint",
    ]

    def __init__(
        self,
        robot_ip: str = "192.168.1.111",
        robot_model: str = "xarm7",
        robot_controller: str = "task_pos",
        robot_gripper: bool = True,
        max_pos_speed=0.25,  # 25% of max speed, 1 m/s
        max_rot_speed=0.785,  # 25% of max speed, 180 deg/s
        tcp_offset_pose=None,
        payload_mass=None,
        payload_cog=None,
        joints_init=None,
        joints_init_speed=1.05,
        soft_real_time=False,
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
            soft_real_time: enables round-robin scheduling and real-time priority
        """
        assert 0 < freq <= 250
        assert 0 < max_pos_speed
        assert 0 < max_rot_speed
        if max_buffer_size is None:
            max_buffer_size = int(freq * 5)
        if tcp_offset_pose is not None:
            tcp_offset_pose = np.array(tcp_offset_pose)
            assert tcp_offset_pose.shape == (6,)
        if payload_mass is not None:
            assert 0 <= payload_mass <= 5
        if payload_cog is not None:
            payload_cog = np.array(payload_cog)
            assert payload_cog.shape == (3,)
            assert payload_mass is not None
        if joints_init is not None:
            joints_init = np.array(joints_init)
            assert joints_init.shape == (6,)
        self.robot_ip = robot_ip
        self.robot_model = ArmModel(robot_model)
        self.robot_controller = ArmController(robot_controller)
        self.robot_gripper = robot_gripper
        self.max_pos_speed = max_pos_speed
        self.max_rot_speed = max_rot_speed
        self.tcp_offset_pose = tcp_offset_pose
        self.payload_mass = payload_mass
        self.payload_cog = payload_cog
        self.joints_init = joints_init
        self.joints_init_speed = joints_init_speed
        self.soft_real_time = soft_real_time
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_request = {
            "type": ArmRequestType.SCHEDULE_WAYPOINT.value,
            "target_pose": np.zeros((6,), dtype=np.float32),
            "target_time": time.now(),
        }

        dummy_data = bytes(1000)  # dummy data, just needs to be larger than 784
        example_robot_state = XArmSocket.bytes_to_state(dummy_data)
        example_robot_state = {k: np.array(v) for k, v in example_robot_state.items()}
        self.example_data = {
            **example_robot_state,
            "timestamp": time.now(),
        }
        super().__post_init__()

        self.arm_c = XArmAPI(self.robot_ip, is_radian=True, report_type="real", do_not_open=True)
        self.arm_r = XArmReceiveInterface(self.robot_ip, self.ring_buffer, self.timeout)

    def run(self):
        try:
            # enable soft real-time
            if self.soft_real_time:
                os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(20))

            arm = self.arm_c

            # https://help.ufactory.cc/en/articles/3954394-guide-to-run-ufactory-xarm-at-the-maximum-speed
            # arm.set_tcp_jerk(7000)
            # arm.set_tcp_maxacc(...)
            # arm.set_joint_jerk(...)
            # arm.set_joint_maxacc(...)
            # arm.save_conf()

            arm.connect()
            arm.clean_error()
            arm.clean_warn()
            arm.motion_enable(True)

            if self.robot_gripper:
                # arm.set_gripper_enable(True)
                # arm.set_collision_tool_model(1)
                arm.set_gripper_speed(5000)  # [0, 5000]

            arm.set_mode(0)
            code = arm.set_state(0)
            assert code == 0, "Check whether e-stop button is pressed."
            # arm.reset(wait=True)
            # arm.move_gohome(wait=True)

            # set parameters
            if self.tcp_offset_pose is not None:
                arm.set_tcp_offset(self.tcp_offset_pose)
            if self.payload_mass is not None:
                if self.payload_cog is not None:
                    code = arm.set_tcp_load(self.payload_mass, self.payload_cog)
                else:
                    code = arm.set_tcp_load(self.payload_mass)
                assert code == 0

            # init pose
            if self.joints_init is not None:
                code = arm.set_servo_angle(angle=self.joints_init, speed=self.joints_init_speed, mvacc=1.4, wait=True)
                assert code == 0

            # 1: servo motion mode, 7: cartesian online trajectory planning mode
            arm.set_mode(1)
            # arm.set_linear_spd_limit_factor(1.2)
            # arm.set_collision_sensitivity(0)
            arm.set_state(0)
            time.sleep(0.1)

            code, curr_pose = arm.get_position_aa()
            assert code == 0
            curr_pose = np.array(curr_pose)
            curr_pose[:3] *= 0.001  # convert mm to m
            # pose interpolation
            curr_t = time.now()
            last_waypoint_time = curr_t
            pose_interp = PoseTrajectoryInterpolator(times=[curr_t], poses=[curr_pose])

            # Main loop
            dt = 1.0 / self.freq
            self.ready_event.set()
            rate = time.Rate(self.freq)
            while not self.exit_event.is_set():
                t_now = time.now()
                # send command to robot
                pose_command = pose_interp(t_now)
                pose_command[:3] *= 1000.0  # convert m to mm
                arm.set_servo_cartesian_aa(pose_command.tolist(), speed=100, mvacc=200)  # mode=1
                # arm.set_position_aa(*pose_command.tolist(), speed=60, wait=False)  # mode=7

                # fetch request from queue with timeout
                try:
                    req = self.input_queue.get()
                    if isinstance(req, dict):
                        req = Request(req.pop("type"), req)
                except queue.Empty:
                    req = None
                if req:
                    if req.type == ArmRequestType.SCHEDULE_WAYPOINT.value:
                        target_pose = np.array(req.params.get("target_pose"))
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
                    else:
                        raise RuntimeError

                # print(1 / (time.now() - rate.start_time))  # max actual frequency
                rate.precise_sleep()
        # except Exception as e:
        #     import traceback
        #     print(e, traceback.format_exc())
        except KeyboardInterrupt:
            pass
        finally:
            # decelerate
            _, angles = arm.get_servo_angle()
            zero_angle_delta = [0.0] * len(angles)
            arm.set_servo_angle(angle=zero_angle_delta, wait=True, relative=True, timeout=0.5)

            if self.robot_gripper and self.robot_model == ArmModel.LITE6:
                arm.stop_lite6_gripper()

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

    def schedule_waypoint(self, pose, target_time):
        assert target_time > time.now()
        pose = np.array(pose)
        assert pose.shape == (6,)
        req = {
            "type": ArmRequestType.SCHEDULE_WAYPOINT.value,
            "target_pose": pose,
            "target_time": target_time,
        }
        self.input_queue.put(req)


class XArmSocket:
    # https://docs.supportarticle.ufactory.cc/support_articles/developer/firmware/how-to-get-the-real-time-data-via-tcp-30000-port.html
    # Frequency: 250HZ (200HZ with FT sensor)
    P30000 = {
        "TargetQ": [33, 60],  # rad
        "TargetQd": [61, 88],  # rad/s
        "TargetTCPPose": [425, 448],  # mm & rad, [x,y,z,rx,ry,rz]
        "TargetTCPSpeed": [449, 472],  # mm/s & rad/s
        "ActualQ": [117, 144],  # rad
        "ActualQd": [449, 472],  # rad/s
        "ActualTCPPose": [473, 496],  # mm & rad, [x,y,z,rx,ry,rz]
        "ActualTCPSpeed": [497, 520],  # mm/s & rad/s
    }
    PORT = 30000
    FREQ = 250

    def bytes_to_fp32(bytes_data, is_big_endian=False):
        return struct.unpack(">f" if is_big_endian else "<f", bytes_data)[0]

    def bytes_to_fp32_list(bytes_data, n=0, is_big_endian=False):
        ret = []
        count = n if n > 0 else len(bytes_data) // 4
        for i in range(count):
            ret.append(XArmSocket.bytes_to_fp32(bytes_data[i * 4 : i * 4 + 4], is_big_endian))
        return ret

    def bytes_to_u8(data):
        data_u8_0 = int.from_bytes(data[:], byteorder="little")
        return data_u8_0

    def bytes_to_u16(data):
        data_u16 = data[0] << 8 | data[1]
        return data_u16

    def bytes_to_u32(data):
        data_u32 = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]
        return data_u32

    def bytes_to_u64(data):
        if len(data) != 8:
            raise ValueError("Input data must be exactly 8 bytes long")
        u64 = struct.unpack(">Q", data)[0]
        return u64

    def pose_aa_to_rpy(pose_aa):
        rpy = st.Rotation.from_rotvec(pose_aa[3:]).as_euler("xyz")
        pose = [*pose_aa[:3], *list(rpy)]
        return pose

    def bytes_to_state(data):
        state = {}
        for key, (start, end) in XArmSocket.P30000.items():
            state[key] = XArmSocket.bytes_to_fp32_list(data[start - 1 : end])
        # state["TargetTCPPoseRPY"] = XArmSocket.pose_aa_to_rpy(state["TargetTCPPose"])
        # state["ActualTCPPoseRPY"] = XArmSocket.pose_aa_to_rpy(state["ActualTCPPose"])
        return state


class XArmReceiveInterface(th.Thread):
    def __init__(self, robot_ip, ring_buffer, timeout, daemon=True):
        super().__init__(daemon=daemon)
        self.robot_ip = robot_ip
        self.ring_buffer = ring_buffer
        self.timeout = timeout
        self.__post_init__()
        self.start()

    def __post_init__(self):
        self.ready_event = th.Event()
        self.exit_event = th.Event()

    def start(self):
        super().start()
        self.ready_event.wait(self.timeout)
        assert self.is_alive()

    def stop(self):
        self.exit_event.set()
        self.join(self.timeout)

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setblocking(True)
        sock.settimeout(1)
        sock.connect((self.robot_ip, XArmSocket.PORT))

        buffer = sock.recv(4)
        while len(buffer) < 4:
            buffer += sock.recv(4 - len(buffer))
        size = XArmSocket.bytes_to_u32(buffer[:4])

        # Main loop
        it = 0
        rate = time.Rate(XArmSocket.FREQ)
        while not self.exit_event.is_set():
            buffer += sock.recv(size - len(buffer))
            if len(buffer) < size:
                continue
            data = buffer[:size]
            buffer = buffer[size:]
            state = XArmSocket.bytes_to_state(data)
            self._put(state)
            rate.precise_sleep()
            if it == 0:
                self.ready_event.set()
            it += 1
        sock.close()

    def _put(self, robot_state):
        robot_state = {k: np.array(v) for k, v in robot_state.items()}
        robot_state["TargetTCPPose"][:3] *= 0.001  # convert mm to m
        robot_state["ActualTCPPose"][:3] *= 0.001  # convert mm to m
        robot_state["TargetTCPSpeed"][:3] *= 0.001  # convert mm/s to m/s
        robot_state["ActualTCPSpeed"][:3] *= 0.001  # convert mm/s to m/s
        data = {
            **robot_state,
            "timestamp": time.now(),
        }
        self.ring_buffer.put(data)


def XArmServer(mw, *args, **kwargs):
    return ServerFactory(mw, XArm, *args, **kwargs)


def XArmClient(mw, *args, **kwargs):
    return ClientFactory(mw, XArm, *args, **kwargs)
