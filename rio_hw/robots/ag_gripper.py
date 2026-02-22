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
    from .utils.ag_driver import AgGripperDriver
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        AgGripperDriver = None  # type: ignore


class RobotController(Enum):
    TASK_POS = auto()


class RequestType(Enum):
    MOVEG = auto()


class AgGripper(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "moveG",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        robot_ip: str = "192.168.0.15",
        tcp_port: int = 54321,
        member_id: int = 1,
        timeout: float = 5.0,
        robot_controller: str = "task_pos",
        move_max_speed: float | None = 3.0,
        force: int = 50,
        calibrate: bool = True,
        dtype=np.float32,
        *,
        freq: int = 30,
        max_buffer_size: int | None = None,
        max_queue_size: int = 1,
        **kwargs,
    ):
        robot_controller = RobotController[robot_controller.upper()]
        if max_buffer_size is None:
            max_buffer_size = int(freq * 10)
        self.robot_ip = robot_ip
        self.tcp_port = tcp_port
        self.member_id = member_id
        self.timeout = timeout
        self.robot_controller = robot_controller
        self.move_max_speed = move_max_speed
        self.force = force
        self.calibrate = calibrate
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, max_queue_size=max_queue_size, **kwargs)

    def __post_init__(self):
        example_request_params = {
            "target_pos": np.zeros((1,), dtype=self.dtype),
        }
        request_params_keys = {
            RobotController.TASK_POS: (RequestType.MOVEG, ("target_pos",)),
        }[self.robot_controller][1]
        example_request_params = {k: example_request_params[k] for k in request_params_keys}
        example_request_params["target_time"] = time.now()

        example_robot_state = {
            "gripper_position": 0.0,
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
        return super().__post_init__()

    def pubreq(self):
        gripper = AgGripperDriver(self.robot_ip, self.tcp_port, self.member_id, self.timeout)
        gripper.connect()
        gripper.setup(force=self.force, do_calibration=self.calibrate)

        try:
            if self.robot_controller == RobotController.TASK_POS:
                # Get current position (normalized 0-1 from per-mille 0-1000)
                curr_pos = gripper.state / 1000.0
                if self.move_max_speed is not None:
                    # pose interpolation
                    curr_time = time.now()
                    last_waypoint_time = curr_time
                    pose_interp = PoseTrajectoryInterpolator(times=[curr_time], poses=[[curr_pos, 0, 0, 0, 0, 0]])
                else:
                    target_pos = np.copy(curr_pos)
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
                    if pose_interp is not None:
                        target_pos = pose_interp(t_now)[0]
                        # Convert normalized (0-1) to per-mille (0-1000)
                        target_permil = int(np.clip(target_pos, 0.0, 1.0) * 1000)
                    else:
                        raise NotImplementedError
                    gripper.set_position(target_permil)
                else:
                    raise ValueError(self.robot_controller)

                # get state from robot
                robot_state = {
                    "gripper_position": gripper.state / 1000.0,  # Normalize to 0-1
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

                # Fetch requests from queue
                try:
                    reqs = self.request_queue.get_all()
                    if isinstance(reqs, dict):
                        reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
                except queue.Empty:
                    reqs = []
                for r in reqs:
                    req = Request(RequestType(r.pop("type")), r)
                    if req.type == RequestType.MOVEG:
                        target_pos = np.array(req.params["target_pos"], dtype=self.dtype)[0]
                        target_time = float(req.params["target_time"])
                        if pose_interp is not None:
                            curr_time = t_now + dt
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
            gripper.close_conn()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def moveG(self, target_pos, target_time):
        target_pos = np.array(target_pos, dtype=self.dtype)
        assert target_pos.shape == (1,)
        assert target_time > time.now()
        req = {
            "type": RequestType.MOVEG.value,
            "target_pos": target_pos,
            "target_time": target_time,
        }
        self.request_queue.put(req)


def AgGripperServer(mw, *args, **kwargs):
    return ServerFactory(mw, AgGripper, *args, **kwargs)


def AgGripperClient(mw, *args, **kwargs):
    return ClientFactory(mw, AgGripper, *args, **kwargs)
