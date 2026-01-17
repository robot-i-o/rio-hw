"""Kinova Gen3 arm controller using BaseCyclic API."""

import os
import queue
import time as builtin_time
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..filters import LowPassFilter
from ..interpolators import TrajectoryInterpolator
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request

try:
    from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
    from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Session_pb2
    from kortex_api.RouterClient import RouterClient
    from kortex_api.SessionManager import SessionManager
    from kortex_api.TCPTransport import TCPTransport
except (ImportError, AttributeError) as e:
    if TYPE_CHECKING:
        raise e
    else:
        BaseClient = None  # type: ignore
        BaseCyclicClient = None  # type: ignore
        Base_pb2 = None  # type: ignore
        BaseCyclic_pb2 = None  # type: ignore
        Session_pb2 = None  # type: ignore
        RouterClient = None  # type: ignore
        SessionManager = None  # type: ignore
        TCPTransport = None  # type: ignore


class RobotModel(Enum):
    GEN3_6DOF = auto()
    GEN3_7DOF = auto()
    GEN3_LITE = auto()


RobotInfo = {
    RobotModel.GEN3_6DOF: {"num_joints": 6},
    RobotModel.GEN3_7DOF: {"num_joints": 7},
    RobotModel.GEN3_LITE: {"num_joints": 6},
}


class RobotController(Enum):
    JOINT_POS = auto()


class RequestType(Enum):
    MOVEJ = auto()
    MOVEL = auto()


class KinovaArm(Node):
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
        robot_ip: str = "192.168.1.10",
        robot_model: str = "gen3_7dof",
        username: str = "admin",
        password: str = "admin",
        max_motor_speed: float = 3.1415,  # rad/s
        joints_init=None,
        joints_lowpass_alpha: float = 0.1,
        soft_real_time: bool = False,
        dtype: np.dtype = np.float32,
        *,
        freq: int = 125,
        max_buffer_size: int | None = None,
        **kwargs,
    ):
        """
        Args:
            robot_ip: IP address of the robot.
            robot_model: "gen3_6dof", "gen3_7dof", or "gen3_lite".
            username: Kinova web interface username.
            password: Kinova web interface password.
            max_motor_speed: Maximum joint speed in rad/s.
            joints_init: Initial joint positions in radians.
            joints_lowpass_alpha: Low-pass filter alpha for joint smoothing.
            soft_real_time: Enable round-robin scheduling and real-time priority.
            dtype: Data type for numpy arrays.
            freq: Control frequency (125-500Hz).
            max_buffer_size: Ring buffer size.
        """
        assert 125 <= freq <= 500
        assert 0 < max_motor_speed
        robot_model_enum = RobotModel[robot_model.upper()]
        num_joints = RobotInfo[robot_model_enum]["num_joints"]
        if max_buffer_size is None:
            max_buffer_size = int(freq * 5)
        if joints_init is not None:
            joints_init = np.array(joints_init, dtype=dtype)
            assert joints_init.shape == (num_joints,)
        self.robot_ip = robot_ip
        self.robot_model = robot_model_enum
        self.username = username
        self.password = password
        self.num_joints = num_joints
        self.max_motor_speed = max_motor_speed
        self.joints_init = joints_init
        self.joints_lowpass_alpha = joints_lowpass_alpha
        self.soft_real_time = soft_real_time
        self.dtype = dtype
        self._base = None
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_data = {
            "joint_q": np.zeros((self.num_joints,), dtype=self.dtype),
            "joint_qd": np.zeros((self.num_joints,), dtype=self.dtype),
            "joint_torques": np.zeros((self.num_joints,), dtype=self.dtype),
            "tcp_pose": np.zeros((6,), dtype=self.dtype),
            "timestamp": time.now(),
        }
        self.example_request = {
            "type": next(iter(RequestType)).value,
            "target_joint_q": np.zeros((self.num_joints,), dtype=self.dtype),
            "target_time": time.now(),
        }
        self.worker = None
        self.run = self.pubreq
        super().__post_init__()

    def pubreq(self):
        # enable soft real-time
        if self.soft_real_time:
            os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(20))

        # Connect to Kinova
        transport = TCPTransport()
        router = RouterClient(transport, lambda kException: None)
        transport.connect(self.robot_ip, 10000)

        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = self.username
        session_info.password = self.password
        session_info.session_inactivity_timeout = 60000
        session_info.connection_inactivity_timeout = 2000

        session_manager = SessionManager(router)
        session_manager.CreateSession(session_info)

        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Set low-level servoing mode (required for BaseCyclic API)
        servoing_mode = Base_pb2.ServoingModeInformation()
        servoing_mode.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
        base.SetServoingMode(servoing_mode)

        # Get initial feedback
        feedback = base_cyclic.RefreshFeedback()

        # Initialize BaseCyclic command from current position
        command = BaseCyclic_pb2.Command()
        command.frame_id = feedback.frame_id
        for i in range(self.num_joints):
            actuator_command = command.actuators.add()
            actuator_command.position = feedback.actuators[i].position
            actuator_command.velocity = 0.0
            actuator_command.torque_joint = 0.0
            actuator_command.command_id = feedback.actuators[i].command_id

        # Current joint positions in radians
        curr_joint_q = np.array(
            [np.deg2rad(feedback.actuators[i].position) for i in range(self.num_joints)],
            dtype=self.dtype,
        )

        # Set up interpolator and filter
        curr_time = time.now()
        last_waypoint_time = curr_time
        joint_interp = TrajectoryInterpolator(times=[curr_time], values=[curr_joint_q])
        lowpass_filter = LowPassFilter(alpha=self.joints_lowpass_alpha, initial=curr_joint_q)

        try:
            dt = 1.0 / self.freq
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                t_now = time.now()

                # Interpolate + filter joint command
                joint_command = joint_interp(t_now)
                joint_command = lowpass_filter(joint_command)

                # Convert to degrees and send to robot
                joint_command_deg = np.rad2deg(joint_command)
                command.frame_id += 1
                for i in range(self.num_joints):
                    command.actuators[i].position = float(joint_command_deg[i])
                    command.actuators[i].command_id += 1

                feedback = base_cyclic.Refresh(command)

                # Extract state from feedback
                joint_q = np.array(
                    [np.deg2rad(feedback.actuators[i].position) for i in range(self.num_joints)],
                    dtype=self.dtype,
                )
                joint_qd = np.array(
                    [np.deg2rad(feedback.actuators[i].velocity) for i in range(self.num_joints)],
                    dtype=self.dtype,
                )
                joint_torques = np.array(
                    [feedback.actuators[i].torque for i in range(self.num_joints)],
                    dtype=self.dtype,
                )
                tcp_pose = np.array(
                    [
                        feedback.base.tool_pose_x,
                        feedback.base.tool_pose_y,
                        feedback.base.tool_pose_z,
                        np.deg2rad(feedback.base.tool_pose_theta_x),
                        np.deg2rad(feedback.base.tool_pose_theta_y),
                        np.deg2rad(feedback.base.tool_pose_theta_z),
                    ],
                    dtype=self.dtype,
                )

                # Store current state in ring buffer
                data = {
                    "joint_q": joint_q,
                    "joint_qd": joint_qd,
                    "joint_torques": joint_torques,
                    "tcp_pose": tcp_pose,
                    "timestamp": builtin_time.time(),
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
                    if req.type == RequestType.MOVEJ:
                        target_joint_q = np.array(req.params.get("target_joint_q"), dtype=self.dtype)
                        target_time = float(req.params.get("target_time"))
                        curr_time = t_now + dt
                        joint_interp = joint_interp.schedule_waypoint(
                            value=target_joint_q,
                            time=target_time,
                            max_speed=self.max_motor_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time,
                        )
                        last_waypoint_time = target_time
                    elif req.type == RequestType.MOVEL:
                        target_eef_pose = np.array(req.params.get("target_eef_pose"), dtype=self.dtype)
                        target_time = float(req.params.get("target_time"))
                        ik_result = self._eval_ik(base, target_eef_pose)
                        if ik_result is not None:
                            curr_time = t_now + dt
                            joint_interp = joint_interp.schedule_waypoint(
                                value=ik_result,
                                time=target_time,
                                max_speed=self.max_motor_speed,
                                curr_time=curr_time,
                                last_waypoint_time=last_waypoint_time,
                            )
                            last_waypoint_time = target_time
                    else:
                        raise ValueError(req.type)
                rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            # Restore single-level servoing mode
            try:
                servoing_mode = Base_pb2.ServoingModeInformation()
                servoing_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
                base.SetServoingMode(servoing_mode)
            except Exception:
                pass
            try:
                session_manager.CloseSession()
                transport.disconnect()
            except Exception:
                pass

    def _eval_ik(self, base, target_eef_pose: np.ndarray) -> np.ndarray | None:
        try:
            ik_input = Base_pb2.IKData()
            ik_input.cartesian_pose.x = float(target_eef_pose[0])
            ik_input.cartesian_pose.y = float(target_eef_pose[1])
            ik_input.cartesian_pose.z = float(target_eef_pose[2])
            ik_input.cartesian_pose.theta_x = float(np.rad2deg(target_eef_pose[3]))
            ik_input.cartesian_pose.theta_y = float(np.rad2deg(target_eef_pose[4]))
            ik_input.cartesian_pose.theta_z = float(np.rad2deg(target_eef_pose[5]))

            ik_result = base.ComputeInverseKinematics(ik_input)
            if ik_result and len(ik_result.joint_angles) == self.num_joints:
                joint_q = np.array(
                    [np.deg2rad(ik_result.joint_angles[i].value) for i in range(self.num_joints)],
                    dtype=self.dtype,
                )
                return joint_q
        except Exception:
            pass
        return None

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

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


def KinovaArmServer(mw, *args, **kwargs):
    return ServerFactory(mw, KinovaArm, *args, **kwargs)


def KinovaArmClient(mw, *args, **kwargs):
    return ClientFactory(mw, KinovaArm, *args, **kwargs)
