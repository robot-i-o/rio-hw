import os
import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..interpolators import PoseTrajectoryInterpolator
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request

try:
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        RTDEControlInterface = None
        RTDEReceiveInterface = None


class RobotModel(Enum):
    UR5E = auto()


RobotInfo = {
    RobotModel.UR5E: {"num_joints": 6},
}


class RobotController(Enum):
    TASK_POS = auto()
    JOINT_POS = auto()
    TASK_VEL = auto()
    JOINT_VEL = auto()


class RequestType(Enum):
    MOVEL = auto()


class UrArm(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "moveL",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        robot_ip: str = "192.168.1.111",
        robot_model: str = "ur5e",
        robot_controller: str = "task_pos",
        lookahead_time=0.1,
        gain=300,
        max_pos_speed=0.25,  # 5% of max speed
        max_rot_speed=0.16,  # 5% of max speed
        tcp_offset_pose=None,
        payload_mass=None,
        payload_cog=None,
        joints_init=None,
        joints_init_speed=1.05,
        soft_real_time=False,
        dtype=np.float32,
        *,
        freq: int = 125,
        max_buffer_size: int | None = None,
        **kwargs,
    ):
        """
        Args:
            lookahead_time: [0.03, 0.2]s smoothens the trajectory with this lookahead time
            gain: [100, 2000] proportional gain for following target position
            max_pos_speed: m/s
            max_rot_speed: rad/s
            tcp_offset_pose: 6d pose
            payload_mass: float
            payload_cog: 3d position, center of gravity
            joint_init:
            joint_init_speed: rad/s
            soft_real_time: enables round-robin scheduling and real-time priority
            dtype:
        """
        assert 0 < freq <= 500
        assert 0.03 <= lookahead_time <= 0.2
        assert 100 <= gain <= 2000
        assert 0 < max_pos_speed
        assert 0 < max_rot_speed
        robot_model = RobotModel[robot_model.upper()]
        robot_controller = RobotController[robot_controller.upper()]
        num_joints = RobotInfo[robot_model]["num_joints"]
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
        self.lookahead_time = lookahead_time
        self.gain = gain
        self.max_pos_speed = max_pos_speed
        self.max_rot_speed = max_rot_speed
        self.tcp_offset_pose = tcp_offset_pose
        self.payload_mass = payload_mass
        self.payload_cog = payload_cog
        self.joints_init = joints_init
        self.joints_init_speed = joints_init_speed
        self.soft_real_time = soft_real_time
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_request_params = {
            "target_pose": np.zeros((6,), dtype=self.dtype),
        }
        request_params_keys = {
            RobotController.TASK_POS: (RequestType.MOVEL, ("target_pose",)),
        }[self.robot_controller][1]
        example_request_params = {k: example_request_params[k] for k in request_params_keys}
        example_request_params["target_time"] = time.now()

        rtde_r = RTDEReceiveInterface(hostname=self.robot_ip)
        receive_fn_map = {
            "actual_tcp_pose": "ActualTCPPose",
            "actual_tcp_speed": "ActualTCPSpeed",
            "actual_jointq": "ActualQ",
            "actual_jointqd": "ActualQd",
            "target_tcp_pose": "TargetTCPPose",
            "target_tcp_speed": "TargetTCPSpeed",
            "target_jointq": "TargetQ",
            "target_jointqd": "TargetQd",
        }
        example_robot_state = {}
        for k, v in receive_fn_map.items():
            example_robot_state[k] = np.array(getattr(rtde_r, f"get{v}")(), dtype=self.dtype)
        self.receive_fn_map = receive_fn_map

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

        rtde_c = RTDEControlInterface(hostname=self.robot_ip)
        rtde_r = RTDEReceiveInterface(hostname=self.robot_ip)

        try:
            # set parameters
            if self.tcp_offset_pose is not None:
                self.rtde_c.setTcp(self.tcp_offset_pose)
            if self.payload_mass is not None:
                if self.payload_cog is not None:
                    assert self.rtde_c.setPayload(self.payload_mass, self.payload_cog)
                else:
                    assert self.rtde_c.setPayload(self.payload_mass)

            # init pose
            if self.joints_init is not None:
                assert rtde_c.moveJ(self.joints_init, self.joints_init_speed, 1.4)

            if self.robot_controller == RobotController.TASK_POS:
                curr_pose = rtde_r.getActualTCPPose()
                # pose interpolation
                curr_t = time.now()
                last_waypoint_time = curr_t
                pose_interp = PoseTrajectoryInterpolator(times=[curr_t], poses=[curr_pose])
            else:
                raise ValueError(self.robot_controller)

            # Main loop
            dt = 1.0 / self.freq
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                t_now = time.now()
                # send command to robot
                if self.robot_controller == RobotController.TASK_POS:
                    pose_command = pose_interp(t_now)
                    vel, acc = 0.5, 0.5  # dummy, not used by ur5
                    assert rtde_c.servoL(pose_command, vel, acc, dt, self.lookahead_time, self.gain)
                else:
                    raise ValueError(self.robot_controller)
                robot_state = {}
                for k, v in self.receive_fn_map.items():
                    robot_state[k] = np.array(getattr(self.rtde_r, f"get{v}")(), dtype=self.dtype)

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
                    else:
                        raise ValueError(req.type)
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            # decelerate
            rtde_c.servoStop()
            # terminate
            rtde_c.stopScript()
            rtde_c.disconnect()
            rtde_r.disconnect()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def moveL(self, pose, target_time):
        pose = np.array(pose, dtype=self.dtype)
        assert target_time > time.now()
        assert pose.shape == (6,)
        req = {
            "type": RequestType.MOVEL.value,
            "target_pose": pose,
            "target_time": target_time,
        }
        self.request_queue.put(req)


def UrArmServer(mw, *args, **kwargs):
    return ServerFactory(mw, UrArm, *args, **kwargs)


def UrArmClient(mw, *args, **kwargs):
    return ClientFactory(mw, UrArm, *args, **kwargs)
