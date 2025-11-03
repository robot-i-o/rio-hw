import os
import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..filters import LowPassFilter
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..pose_trajectory_interpolator import PoseTrajectoryInterpolator
from ..request import Request

try:
    from .utils.franka_driver import FrankaDriver
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        FrankaDriver = None  # type: ignore


class ArmModel(Enum):
    FR3 = auto()
    PANDA = auto()


class ArmController(Enum):
    TASK_POS = auto()
    JOINT_POS = auto()
    TASK_VEL = auto()
    JOINT_VEL = auto()
    TASK_IMPEDANCE = auto()
    JOINT_IMPEDANCE = auto()
    TASK_OSC = auto()


ArmInfo = {
    ArmModel.FR3: {"num_joints": 7},
    ArmModel.PANDA: {"num_joints": 7},
}


class RequestType(Enum):
    MOVEL = auto()
    MOVEJ = auto()
    SPEEDL = auto()
    SPEEDJ = auto()
    IMPEDANCEL = auto()
    IMPEDANCEJ = auto()
    OSCL = auto()


class Franka(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "moveL",
        "moveJ",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        robot_ip: str = "192.168.1.111",
        robot_model: str = "fr3",
        robot_controller: str = "task_pos",
        max_pos_speed=0.25,  # 12.5% of max speed, 2m/s
        max_rot_speed=0.657,  # 12.5% of max speed, 301 deg/s
        tcp_offset_pose=None,
        payload_mass=None,
        payload_cog=None,
        joints_init=None,
        joints_init_speed=1.05,
        joint_lowpass_alpha=0.1,
        soft_real_time=False,
        driver: str = "panda_py",
        robot_port: int = 50051,
        dtype: np.dtype = np.float32,
        *,
        freq: int = 500,
        max_buffer_size: int | None = None,
        **kwargs,
    ):
        assert 0 < freq <= 1000
        assert 0 < max_pos_speed
        assert 0 < max_rot_speed
        robot_model = ArmModel[robot_model.upper()]
        robot_controller = ArmController[robot_controller.upper()]
        num_joints = ArmInfo[robot_model]["num_joints"]
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
        self.driver = driver
        self.robot_port = robot_port
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

        example_robot_state = {
            "actual_tcp_pose": np.zeros((6,), dtype=self.dtype),
            "actual_tcp_speed": np.zeros((6,), dtype=self.dtype),
            "actual_jointq": np.zeros((self.num_joints,), dtype=self.dtype),
            "actual_jointqd": np.zeros((self.num_joints,), dtype=self.dtype),
            "target_tcp_pose": np.zeros((6,), dtype=self.dtype),
            "target_tcp_speed": np.zeros((6,), dtype=self.dtype),
            "target_jointq": np.zeros((self.num_joints,), dtype=self.dtype),
            "target_jointqd": np.zeros((self.num_joints,), dtype=self.dtype),
        }

        self.example_request = {
            "type": next(iter(RequestType)).value,
            **example_request_params,
        }
        self.example_data = {
            **example_robot_state,
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pubreq
        super().__post_init__()

    def pubreq(self):
        # enable soft real-time
        if self.soft_real_time:
            os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(20))

        arm = FrankaDriver(
            self.driver,
            robot_ip=self.robot_ip,
            robot_model=self.robot_model.name.lower(),
            robot_port=self.robot_port,
        )
        arm.start()

        try:
            # set parameters
            if self.tcp_offset_pose is not None:
                arm.set_tcp_offset(self.tcp_offset_pose)
            if self.payload_mass is not None:
                arm.set_tcp_load(self.payload_mass, self.payload_cog)

            # init pose
            if self.joints_init is not None:
                arm.moveJ(self.joints_init, wait=True)

            if self.robot_controller == ArmController.TASK_POS:
                curr_pose = arm.state()["actual_tcp_pose"]
                # pose interpolation
                curr_t = time.now()
                last_waypoint_time = curr_t
                pose_interp = PoseTrajectoryInterpolator(times=[curr_t], poses=[curr_pose])
            elif self.robot_controller == ArmController.JOINT_POS:
                curr_jointq = arm.state()["actual_jointq"]
                # joint filtering/smoothing
                target_jointq = curr_jointq
                lowpass_filter = LowPassFilter(alpha=self.joint_lowpass_alpha, initial=curr_jointq)
            else:
                raise ValueError(self.robot_controller)

            # Main loop
            dt = 1.0 / self.freq
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                t_now = time.now()
                if self.robot_controller == ArmController.TASK_POS:
                    pose_command = pose_interp(t_now)
                    arm.moveL(pose_command)
                elif self.robot_controller == ArmController.JOINT_POS:
                    joint_command = lowpass_filter(target_jointq)
                    arm.moveJ(joint_command)
                else:
                    raise ValueError(self.robot_controller)
                robot_state = arm.state()

                # Store current state in ring buffer
                data = {
                    **robot_state,
                    "timestamp": time.now(),
                }
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False

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
                    elif req.type == RequestType.MOVEJ:
                        target_jointq = np.array(req.params.get("target_jointq"))
                    else:
                        raise ValueError(req.type)
                rate.sleep()  # not using precise_sleep() spinning for higher frequency control
        except KeyboardInterrupt:
            pass
        finally:
            arm.stop()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def moveL(self, target_pose, target_time):
        target_pose = np.array(target_pose)
        assert target_pose.shape == (6,)
        min_target_time = time.now() + 0.01
        if target_time < min_target_time:
            target_time = min_target_time
        req = {
            "type": RequestType.MOVEL.value,
            "target_pose": target_pose,
            "target_time": target_time,
        }
        self.request_queue.put(req)

    def moveJ(self, target_joints, target_time):
        target_joints = np.array(target_joints)
        assert target_joints.shape == (self.num_joints,)
        assert target_time > time.now()
        req = {
            "type": RequestType.MOVEJ.value,
            "target_jointq": target_joints,
            "target_time": target_time,
        }
        self.request_queue.put(req)


def FrankaServer(mw, *args, **kwargs):
    return ServerFactory(mw, Franka, *args, **kwargs)


def FrankaClient(mw, *args, **kwargs):
    return ClientFactory(mw, Franka, *args, **kwargs)
