import os
import queue
import socket
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..filters import LowPassFilter
from ..interpolators import PoseTrajectoryInterpolator, TrajectoryInterpolator
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request

try:
    from xarm.wrapper import XArmAPI

    from .utils.xarm_driver import XArmSocket
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        XArmAPI = None  # type: ignore
        XArmSocket = None  # type: ignore


class RobotModel(Enum):
    XARM6 = auto()
    XARM7 = auto()
    XARM850 = auto()
    LITE6 = auto()


RobotInfo = {
    RobotModel.XARM6: {"num_joints": 6},
    RobotModel.XARM7: {"num_joints": 7},
    RobotModel.XARM850: {"num_joints": 6},
    RobotModel.LITE6: {"num_joints": 6},
}


class RobotController(Enum):
    TASK_POS = auto()
    JOINT_POS = auto()
    TASK_VEL = auto()
    JOINT_VEL = auto()
    TASK_IMPEDANCE = auto()


class RequestType(Enum):
    MOVEL = auto()
    MOVEJ = auto()
    SPEEDL = auto()
    SPEEDJ = auto()


class XarmArm(Node):
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
        max_pos_speed: float | None = 0.25,  # 25% of max speed, 1 m/s
        max_rot_speed: float | None = 0.785,  # 25% of max speed, 180 deg/s
        max_motor_speed: float | None = 3.1415,  # 100% of max speed, 180 deg/s
        tcp_offset_pose=None,
        payload_mass=None,
        payload_cog=None,
        joints_init=None,
        joints_init_speed=1.05,
        joints_lowpass_alpha=0.1,
        impedance_params: dict | None = None,
        soft_real_time=False,
        dtype=np.float32,
        *,
        freq: int = 250,
        max_buffer_size: int | None = None,
        **kwargs,
    ):
        """
        Args:
            max_pos_speed: m/s
            max_rot_speed: rad/s
            max_motor_speed: rad/s
            tcp_offset_pose: 6d pose
            payload_mass: float
            payload_cog: 3d position, center of gravity
            joint_init:
            joint_init_speed:
            joints_lowpass_alpha:
            soft_real_time: enables round-robin scheduling and real-time priority
            dtype:
        """
        assert 0 < freq <= 250
        assert 0 < max_pos_speed
        assert 0 < max_rot_speed
        assert 0 < max_motor_speed
        robot_model = RobotModel[robot_model.upper()]
        robot_controller = RobotController[robot_controller.upper()]
        num_joints = RobotInfo[robot_model]["num_joints"]
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
        self.max_motor_speed = max_motor_speed
        self.tcp_offset_pose = tcp_offset_pose
        self.payload_mass = payload_mass
        self.payload_cog = payload_cog
        self.joints_init = joints_init
        self.joints_init_speed = joints_init_speed
        self.joints_lowpass_alpha = joints_lowpass_alpha
        _impedance_defaults = {
            "K_pos": 800.0,  #  x/y/z linear stiffness coefficient, range: 0 ~ 2000 (N/m)
            "K_ori": 8.0,  #  Rx/Ry/Rz rotational stiffness coefficient, range: 0 ~ 20 (Nm/rad)
            # for M and J, smaller value means less effort to drive the arm, but may also be less stable, please be careful.
            "M": 0.04,  #  x/y/z equivalent mass; range: 0.02 ~ 1 kg
            "J_scale": 0.01,  #  Rx/Ry/Rz equivalent moment of inertia, range: 1e-4 ~ 0.01 (Kg*m^2)
            "c_axis": [1, 1, 1, 1, 1, 1],  # compliant axes (translation, rotation)
            "ref_frame": 0,  # 0 : base , 1 : tool
            "ft_sensor_zero": False,
        }
        self.impedance_params = {**_impedance_defaults, **(impedance_params or {})}
        self.soft_real_time = soft_real_time
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_request_params = {
            "target_eef_pose": np.zeros((6,), dtype=self.dtype),
            "target_joint_q": np.zeros((self.num_joints,), dtype=self.dtype),
            "target_eef_twist": np.zeros((6,), dtype=self.dtype),
            "target_joint_qd": np.zeros((self.num_joints,), dtype=self.dtype),
        }
        request_params_keys = {
            RobotController.TASK_POS: (RequestType.MOVEL, ("target_eef_pose",)),
            RobotController.JOINT_POS: (RequestType.MOVEJ, ("target_joint_q",)),
            RobotController.TASK_VEL: (RequestType.SPEEDL, ("target_eef_twist",)),
            RobotController.JOINT_VEL: (RequestType.SPEEDJ, ("target_joint_qd",)),
            RobotController.TASK_IMPEDANCE: (RequestType.MOVEL, ("target_eef_pose",)),
        }[self.robot_controller][1]
        example_request_params = {k: example_request_params[k] for k in request_params_keys}
        example_request_params["target_time"] = time.now()

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
            freq = XArmSocket.FREQ
            if self.robot_controller == RobotController.TASK_IMPEDANCE:
                freq = XArmSocket.FREQ_FT
            rate = time.Rate(freq)
            not_pub_ready = True
            while not self.exit_event.is_set():
                buffer += sock.recv(size - len(buffer))
                if len(buffer) < size:
                    continue
                data = buffer[:size]
                buffer = buffer[size:]

                robot_state = XArmSocket.bytes_to_state(data)
                robot_state = {k: np.array(v, dtype=self.dtype) for k, v in robot_state.items()}
                robot_state["eef_pose"][:3] *= 0.001  # convert mm to m
                robot_state["eef_twist"][:3] *= 0.001  # convert mm/s to m/s
                robot_state["target_eef_pose"][:3] *= 0.001  # convert mm to m
                robot_state["target_eef_twist"][:3] *= 0.001  # convert mm/s to m/s

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
        # enable soft real-time
        if self.soft_real_time:
            os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(20))

        arm = XArmAPI(self.robot_ip, is_radian=True, report_type="real", do_not_open=True)
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

        # https://help.ufactory.cc/en/articles/3954394-guide-to-run-ufactory-xarm-at-the-maximum-speed
        # arm.set_tcp_jerk(7000)
        # arm.set_tcp_maxacc(...)
        # arm.set_joint_jerk(28647, is_radian=True)
        # arm.set_joint_maxacc(10000, is_radian=True)
        # arm.set_linear_spd_limit_factor(1.75)
        # arm.set_collision_sensitivity(0)
        # arm.set_teach_sensitivity(5)
        # arm.save_conf()

        try:
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

            if self.robot_controller == RobotController.TASK_IMPEDANCE:
                K_pos, K_ori = self.impedance_params["K_pos"], self.impedance_params["K_ori"]
                M = self.impedance_params["M"]
                J = M * self.impedance_params["J_scale"]
                ref_frame, c_axis = self.impedance_params["ref_frame"], self.impedance_params["c_axis"]

                arm.set_ft_sensor_admittance_parameters(
                    ref_frame,
                    c_axis,
                    M=[M, M, M, J, J, J],
                    K=[K_pos, K_pos, K_pos, K_ori, K_ori, K_ori],
                    B=[0] * 6,  # B(damping) is reserved, give zeros
                )
                arm.set_ft_sensor_enable(1)

                # will overwrite previous sensor zero and payload configuration
                if self.impedance_params["ft_sensor_zero"]:
                    arm.set_ft_sensor_zero()  # remove this if zero_offset and payload already identified & compensated!
                time.sleep(0.2)  # wait for writing zero operation to take effect, do not remove
                arm.set_ft_sensor_mode(1)  # activate admittance mode

            if self.robot_controller in (RobotController.TASK_POS, RobotController.TASK_IMPEDANCE):
                arm.set_mode(1)  # 1: servo motion mode
            elif self.robot_controller == RobotController.JOINT_POS:
                arm.set_mode(1)  # 1: servo motion mode
            elif self.robot_controller == RobotController.JOINT_VEL:
                arm.set_mode(4)  # 4: velocity control mode
            else:
                raise ValueError(self.robot_controller)
            arm.set_state(0)
            time.sleep(0.1)

            if self.robot_controller in (RobotController.TASK_POS, RobotController.TASK_IMPEDANCE):
                code, curr_pose = arm.get_position_aa()
                assert code == 0
                curr_pose = np.array(curr_pose, dtype=self.dtype)
                curr_pose[:3] *= 0.001  # convert mm to m
                if self.max_pos_speed is not None and self.max_rot_speed is not None:
                    # pose interpolation
                    curr_time = time.now()
                    last_waypoint_time = curr_time
                    pose_interp = PoseTrajectoryInterpolator(times=[curr_time], poses=[curr_pose])
                else:
                    target_pose = np.copy(curr_pose)
                    pose_interp = None
            elif self.robot_controller == RobotController.JOINT_POS:
                code, curr_joint_q = arm.get_servo_angle()
                assert code == 0
                curr_joint_q = np.array(curr_joint_q[: self.num_joints], dtype=self.dtype)
                if self.max_motor_speed is not None:
                    # joint interpolation
                    curr_time = time.now()
                    last_waypoint_time = curr_time
                    joint_interp = TrajectoryInterpolator(times=[curr_time], values=[curr_joint_q])
                    # joint filtering/smoothing
                    lowpass_filter = LowPassFilter(alpha=self.joints_lowpass_alpha, initial=curr_joint_q)
                else:
                    target_joint_q = np.copy(curr_joint_q)
                    joint_interp = None
                    lowpass_filter = None
            elif self.robot_controller == RobotController.JOINT_VEL:
                target_joint_qd = np.zeros(self.num_joints, dtype=self.dtype)
            else:
                raise ValueError(self.robot_controller)

            # Main loop
            dt = 1.0 / self.freq
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            while not self.exit_event.is_set():
                t_now = time.now()
                # send command to robot
                if self.robot_controller in (RobotController.TASK_POS, RobotController.TASK_IMPEDANCE):
                    if pose_interp is not None:
                        pose_command = pose_interp(t_now)
                    else:
                        pose_command = np.copy(target_pose)
                    pose_command[:3] *= 1000.0  # convert m to mm
                    code = arm.set_servo_cartesian_aa(pose_command.tolist(), speed=100, mvacc=200)
                elif self.robot_controller == RobotController.JOINT_POS:
                    if joint_interp is not None:
                        joint_command = joint_interp(t_now)
                        joint_command = lowpass_filter(joint_command)
                    else:
                        joint_command = np.copy(target_joint_q)
                    code = arm.set_servo_angle_j(joint_command.tolist(), speed=3.14, mvacc=20, is_radian=True)
                elif self.robot_controller == RobotController.JOINT_VEL:
                    joint_command = np.copy(target_joint_qd)
                    code = arm.vc_set_joint_velocity(joint_command.tolist(), is_radian=True, is_sync=True, duration=0)
                else:
                    raise ValueError(self.robot_controller)
                if not (code == 0 and arm.error_code == 0 and arm.connected):
                    _ = arm.get_err_warn_code(show=True, lang="en")
                    raise RuntimeError(f"code: {code}, error_code: {arm.error_code}, connected: {arm.connected}")

                # Fetch requests from queue
                try:
                    reqs = self.request_queue.get_all()
                    if isinstance(reqs, dict):
                        reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
                except queue.Empty:
                    reqs = []
                for r in reqs:
                    req = Request(RequestType(r.pop("type")), r)
                    if req.type == RequestType.MOVEL:
                        target_pose = np.array(req.params.get("target_eef_pose"), dtype=self.dtype)
                        target_time = float(req.params.get("target_time"))
                        if pose_interp is not None:
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
                        target_joint_q = np.array(req.params.get("target_joint_q"), dtype=self.dtype)
                        target_time = float(req.params.get("target_time"))
                        if joint_interp is not None:
                            curr_time = t_now + dt
                            joint_interp = joint_interp.schedule_waypoint(
                                value=target_joint_q,
                                time=target_time,
                                max_speed=self.max_motor_speed,
                                curr_time=curr_time,
                                last_waypoint_time=last_waypoint_time,
                            )
                            last_waypoint_time = target_time
                    elif req.type == RequestType.SPEEDL:
                        raise NotImplementedError
                    elif req.type == RequestType.SPEEDJ:
                        target_joint_qd = np.array(req.params.get("target_joint_qd"), dtype=self.dtype)
                        target_time = float(req.params.get("target_time"))
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
            if self.robot_controller == RobotController.TASK_IMPEDANCE:
                arm.set_ft_sensor_mode(0)
                arm.set_ft_sensor_enable(0)
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

    def moveL(self, target_eef_pose, target_time):
        target_eef_pose = np.array(target_eef_pose, dtype=self.dtype)
        assert target_eef_pose.shape == (6,)
        assert target_time > time.now()
        req = {
            "type": RequestType.MOVEL.value,
            "target_eef_pose": target_eef_pose,
            "target_time": target_time,
        }
        self.request_queue.put(req)

    def moveJ(self, target_joint_q, target_time):
        target_joint_q = np.array(target_joint_q, dtype=self.dtype)
        assert target_joint_q.shape == (self.num_joints,)
        assert target_time > time.now()
        req = {
            "type": RequestType.MOVEJ.value,
            "target_joint_q": target_joint_q,
            "target_time": target_time,
        }
        self.request_queue.put(req)

    def speedL(self, target_eef_twist, target_time):
        target_eef_twist = np.array(target_eef_twist, dtype=self.dtype)
        assert target_eef_twist.shape == (6,)
        assert target_time > time.now()
        req = {
            "type": RequestType.SPEEDL.value,
            "target_eef_twist": target_eef_twist,
            "target_time": target_time,
        }
        self.request_queue.put(req)

    def speedJ(self, target_joint_qd, target_time):
        target_joint_qd = np.array(target_joint_qd, dtype=self.dtype)
        assert target_joint_qd.shape == (self.num_joints,)
        assert target_time > time.now()
        req = {
            "type": RequestType.SPEEDJ.value,
            "target_joint_qd": target_joint_qd,
            "target_time": target_time,
        }
        self.request_queue.put(req)


def XarmArmServer(mw, *args, **kwargs):
    return ServerFactory(mw, XarmArm, *args, **kwargs)


def XarmArmClient(mw, *args, **kwargs):
    return ClientFactory(mw, XarmArm, *args, **kwargs)
