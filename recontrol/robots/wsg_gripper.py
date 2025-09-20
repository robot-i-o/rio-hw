import queue
from enum import Enum, auto

import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..pose_trajectory_interpolator import PoseTrajectoryInterpolator
from ..request import Request
from .utils.wsg_binary_driver import WSGBinaryDriver


class GripperModel(Enum):
    WSG50 = "wsg50"


class GripperController(Enum):
    TASK_POS = "task_pos"


class RequestType(Enum):
    MOVEL = auto()


class WsgGripper:
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
        robot_port: int = 1000,
        robot_model: str = "wsg50",
        robot_controller: str = "task_pos",
        move_max_speed: float = 200.0,
        use_meters: bool = True,
        home_to_open: bool = True,
        dtype=np.float64,
        *,
        freq: int = 30,
        max_buffer_size: int | None = None,
        max_queue_size: int = 1024,
        **kwargs,
    ):
        if max_buffer_size is None:
            max_buffer_size = int(freq * 10)
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot_model = GripperModel(robot_model)
        self.robot_controller = GripperController(robot_controller)
        self.move_max_speed = move_max_speed
        self.use_meters = use_meters
        self.scale = 1000.0 if self.use_meters else 1.0
        self.home_to_open = home_to_open
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, max_queue_size=max_queue_size, **kwargs)

    def __post_init__(self):
        example_request_params = {
            GripperController.TASK_POS: (RequestType.MOVEL, {"target_pose": np.zeros((1,), dtype=self.dtype)}),
        }[self.robot_controller][1]
        example_request_params = {
            **example_request_params,
            "target_time": time.now(),
        }

        example_robot_state = {
            "gripper_state": 0,
            "gripper_position": 0.0,
            "gripper_velocity": 0.0,
            "gripper_force": 0.0,
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

        self.gripper = WSGBinaryDriver(hostname=self.robot_ip, port=self.robot_port)
        self.gripper.start()

    def pubreq(self):
        try:
            wsg = self.gripper

            # home gripper to initialize
            wsg.ack_fault()
            wsg.homing(positive_direction=self.home_to_open, wait=True)

            if self.robot_controller == GripperController.TASK_POS:
                curr_info = wsg.script_query()
                curr_pos = curr_info["position"]
                # pose interpolation
                curr_t = time.now()
                last_waypoint_time = curr_t
                pose_interp = PoseTrajectoryInterpolator(times=[curr_t], poses=[[curr_pos, 0, 0, 0, 0, 0]])
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
                if self.robot_controller == GripperController.TASK_POS:
                    target_pos = pose_interp(t_now)[0]
                    target_vel = (target_pos - pose_interp(t_now - dt)[0]) / dt
                    info = wsg.script_position_pd(position=target_pos, velocity=target_vel)
                else:
                    raise ValueError(self.robot_controller)

                # get state from robot
                robot_state = {
                    "gripper_state": info["state"],
                    "gripper_position": info["position"] / self.scale,
                    "gripper_velocity": info["velocity"] / self.scale,
                    "gripper_force": info["force_motor"],
                    # "gripper_measure_timestamp": info['measure_timestamp'],
                }

                # Store current state in ring buffer
                data = {
                    **robot_state,
                    "timestamp": time.now(),
                }
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False

                # Fetch request from queue
                try:
                    req = self.request_queue.get()
                    if isinstance(req, dict):
                        req = Request(RequestType(req.pop("type")), req)
                except queue.Empty:
                    req = None
                if req:
                    if req.type == RequestType.MOVEL:
                        target_pos = req.params["target_pose"][0]
                        target_pos = target_pos * self.scale
                        target_time = float(req.params["target_time"])
                        curr_time = t_now
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=[target_pos, 0, 0, 0, 0, 0],
                            time=target_time,
                            max_pos_speed=self.move_max_speed,
                            max_rot_speed=self.move_max_speed,
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
            wsg.stop()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def moveL(self, target_pose, target_time):
        target_pose = np.array(target_pose, dtype=self.dtype)
        assert target_pose.shape == (1,)
        assert target_time > time.now()
        req = {
            "type": RequestType.MOVEL.value,
            "target_pose": target_pose,
            "target_time": target_time,
        }
        self.request_queue.put(req)


def WsgGripperServer(mw, *args, **kwargs):
    return ServerFactory(mw, WsgGripper, *args, **kwargs)


def WsgGripperClient(mw, *args, **kwargs):
    return ClientFactory(mw, WsgGripper, *args, **kwargs)
