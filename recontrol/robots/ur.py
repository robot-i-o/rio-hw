import os
import queue
from enum import Enum, auto

import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..pose_trajectory_interpolator import PoseTrajectoryInterpolator
from ..request import Request

try:
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
except ImportError:
    RTDEControlInterface = None
    RTDEReceiveInterface = None


class ArmModel(Enum):
    UR5 = "ur5e"


class ArmController(Enum):
    # JOINT_POS = "joint_pos"
    # JOINT_VEL = "joint_vel"
    TASK_POS = "task_pos"


class ArmRequestType(Enum):
    SCHEDULE_WAYPOINT = auto()


class UR:
    __api__ = [
        "get_state",
        "get_all_state",
        "schedule_waypoint",
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
        dtype=np.float64,
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
        rtde_r = RTDEReceiveInterface(hostname=self.robot_ip)
        receive_keys = [
            "ActualTCPPose",
            "ActualTCPSpeed",
            "ActualQ",
            "ActualQd",
            "TargetTCPPose",
            "TargetTCPSpeed",
            "TargetQ",
            "TargetQd",
        ]
        example_robot_state = {}
        for key in receive_keys:
            example_robot_state[key] = np.array(getattr(rtde_r, f"get{key}")(), dtype=self.dtype)
        self.receive_keys = receive_keys

        self.example_request = {
            "type": next(iter(ArmRequestType)).value,
            "target_pose": np.zeros((6,), dtype=self.dtype),
            "target_time": time.now(),
        }
        self.example_data = {
            **example_robot_state,
            "timestamp": time.now(),
        }
        self.worker = self.pub
        self.run = self.req
        super().__post_init__()

        self.rtde_c = RTDEControlInterface(hostname=self.robot_ip)
        self.rtde_r = rtde_r

    def pub(self):
        try:
            # Main loop
            rate = time.Rate(self.freq)
            self.pub_ready_event.set()
            while not self.exit_event.is_set():
                robot_state = {}
                for key in self.receive_keys:
                    robot_state[key] = np.array(getattr(self.rtde_r, "get" + key)(), dtype=self.dtype)

                # Store current state in ring buffer
                data = {
                    **robot_state,
                    "timestamp": time.now(),
                }
                self.ring_buffer.put(data)
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            pass

    def req(self):
        try:
            # enable soft real-time
            if self.soft_real_time:
                os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(20))

            rtde_c = self.rtde_c
            rtde_r = self.rtde_r

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

            curr_pose = rtde_r.getActualTCPPose()
            # pose interpolation
            curr_t = time.now()
            last_waypoint_time = curr_t
            pose_interp = PoseTrajectoryInterpolator(times=[curr_t], poses=[curr_pose])

            # Main loop
            dt = 1.0 / self.freq
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            while not self.exit_event.is_set():
                t_now = time.now()
                # send command to robot
                pose_command = pose_interp(t_now)
                vel = 0.5
                acc = 0.5
                assert rtde_c.servoL(
                    pose_command,
                    vel,
                    acc,  # dummy, not used by ur5
                    dt,
                    self.lookahead_time,
                    self.gain,
                )

                # Fetch request from queue
                try:
                    req = self.request_queue.get()
                    if isinstance(req, dict):
                        req = Request(req.pop("type"), req)
                except queue.Empty:
                    req = None
                if req:
                    if req.type == ArmRequestType.SCHEDULE_WAYPOINT.value:
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
                        raise RuntimeError
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

    def schedule_waypoint(self, pose, target_time):
        pose = np.array(pose, dtype=self.dtype)
        assert target_time > time.now()
        assert pose.shape == (6,)
        req = {
            "type": ArmRequestType.SCHEDULE_WAYPOINT.value,
            "target_pose": pose,
            "target_time": target_time,
        }
        self.request_queue.put(req)


def URServer(mw, *args, **kwargs):
    return ServerFactory(mw, UR, *args, **kwargs)


def URClient(mw, *args, **kwargs):
    return ClientFactory(mw, UR, *args, **kwargs)
