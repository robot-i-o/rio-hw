import queue
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
    from .utils.leap_hand_driver import LeapHandV1Driver
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        LeapHandV1Driver = None  # type: ignore


class RobotModel(Enum):
    LEAPV1 = auto()
    LEAPV1LITE = auto()


RobotInfo = {
    RobotModel.LEAPV1: {"num_joints": 16, "motor": "XC330_M288_T"},
    RobotModel.LEAPV1LITE: {"num_joints": 16, "motor": "XL330_M288_T"},
}


class RobotController(Enum):
    JOINT_POS = auto()


class RequestType(Enum):
    MOVEJ = auto()


class LeapHand(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "method",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        robot_port: str = "/dev/ttyUSB0",
        robot_model: str = "leapv1",
        robot_controller: str = "joint_pos",
        robot_driver_kwargs: dict | None = None,
        max_motor_speed=8.63,
        joints_init=None,
        joints_init_speed=1.05,
        joints_lowpass_alpha=0.1,
        dtype=np.float32,
        *,
        freq: int = 60,
        max_buffer_size: int | None = None,
        max_queue_size: int = 128,
        **kwargs,
    ):
        assert 0 < freq <= 83
        assert 0 < max_motor_speed
        robot_model = RobotModel[robot_model.upper()]
        robot_controller = RobotController[robot_controller.upper()]
        robot_driver_kwargs = robot_driver_kwargs or {}
        num_joints = RobotInfo[robot_model]["num_joints"]
        motor = RobotInfo[robot_model]["motor"]
        if max_buffer_size is None:
            max_buffer_size = int(freq * 10)
        if joints_init is not None:
            joints_init = np.array(joints_init, dtype=dtype)
            assert joints_init.shape == (num_joints,)
        self.robot_port = robot_port
        self.robot_model = robot_model
        self.robot_controller = robot_controller
        self.robot_driver_kwargs = robot_driver_kwargs
        self.num_joints = num_joints
        self.motor = motor
        self.max_motor_speed = max_motor_speed
        self.joints_init = joints_init
        self.joints_init_speed = joints_init_speed
        self.joints_lowpass_alpha = joints_lowpass_alpha
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, max_queue_size=max_queue_size, **kwargs)

    def __post_init__(self):
        example_request_params = {
            "target_joint_q": np.zeros((self.num_joints,), dtype=self.dtype),
        }
        request_params_keys = {
            RobotController.GUIDE: (None, ()),
            RobotController.JOINT_POS: (RequestType.MOVEJ, ("target_joint_q",)),
        }[self.robot_controller][1]
        example_request_params = {k: example_request_params[k] for k in request_params_keys}
        example_request_params["target_time"] = time.now()

        example_robot_state = {
            "joint_q": np.zeros((self.num_joints,), dtype=self.dtype),
            "joint_qd": np.zeros((self.num_joints,), dtype=self.dtype),
            "joint_current": np.zeros((self.num_joints,), dtype=self.dtype),
            "joint_torque": np.zeros((self.num_joints,), dtype=self.dtype),
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
        hand = LeapHandV1Driver(port=self.robot_port, motor=self.motor, **self.robot_driver_kwargs)
        hand.open()

        try:
            # init pose
            if self.joints_init is not None:
                hand.moveJ(self.joints_init, wait=True)

            if self.robot_controller == RobotController.JOINT_POS:
                curr_joint_q = hand.state()["joint_q"]
                # joint interpolation
                curr_t = time.now()
                last_waypoint_time = curr_t
                joint_interp = TrajectoryInterpolator(times=[curr_t], values=[curr_joint_q])
                # joint filtering/smoothing
                lowpass_filter = LowPassFilter(alpha=self.joints_lowpass_alpha, initial=curr_joint_q)
            elif self.robot_controller == RobotController.GUIDE:
                pass
            else:
                raise ValueError(self.robot_controller)

            # Main loop
            dt = 1.0 / self.freq
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                t_now = time.now()
                if self.robot_controller == RobotController.JOINT_POS:
                    joint_command = joint_interp(t_now)
                    joint_command = lowpass_filter(joint_command)
                    hand.moveJ(joint_command.tolist())
                elif self.robot_controller == RobotController.GUIDE:
                    pass
                else:
                    raise ValueError(self.robot_controller)
                robot_state = hand.state()

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
                    else:
                        raise RuntimeError(req.type)
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            hand.close()

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


def LeapHandServer(mw, *args, **kwargs):
    return ServerFactory(mw, LeapHand, *args, **kwargs)


def LeapHandClient(mw, *args, **kwargs):
    return ClientFactory(mw, LeapHand, *args, **kwargs)
