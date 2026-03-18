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
    from .utils.waveshare_piper_driver import WavesharePiperDriver
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        WavesharePiperDriver = None  # type: ignore


class RequestType(Enum):
    MOVEG = auto()


class WavesharePiperGripper(Node):
    """Waveshare PiperX gripper node over USB-CAN serial adapter."""

    __api__ = [
        "get_state",
        "get_all_state",
        "moveG",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 2_000_000,
        max_angle: int = 76_101,
        default_effort: int = 2000,
        max_gripper_speed: float | None = 3.0,
        dtype=np.float32,
        *,
        freq: int = 100,
        max_buffer_size: int | None = None,
        max_queue_size: int = 128,
        **kwargs,
    ):
        if max_buffer_size is None:
            max_buffer_size = int(freq * 10)
        self.port = port
        self.baudrate = baudrate
        self.max_angle = max_angle
        self.default_effort = default_effort
        self.max_gripper_speed = max_gripper_speed
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, max_queue_size=max_queue_size, **kwargs)

    def __post_init__(self):
        example_request_params = {
            "target_pos": np.zeros((1,), dtype=self.dtype),
            "target_time": time.now(),
        }

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
        super().__post_init__()

    def pubreq(self):
        gripper = WavesharePiperDriver(
            port=self.port,
            baudrate=self.baudrate,
            max_angle=self.max_angle,
            default_effort=self.default_effort,
        )
        gripper.start()

        try:
            curr_pos = gripper.state()["gripper_position"]
            if self.max_gripper_speed is not None:
                curr_time = time.now()
                last_waypoint_time = curr_time
                pose_interp = PoseTrajectoryInterpolator(times=[curr_time], poses=[[curr_pos, 0, 0, 0, 0, 0]])
            else:
                target_pos = np.copy(curr_pos)
                pose_interp = None

            # Main loop
            dt = 1.0 / self.freq
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                t_now = time.now()

                # Send command
                if self.max_gripper_speed is not None:
                    pos_command = pose_interp(t_now)[0]
                else:
                    pos_command = np.copy(target_pos)
                gripper.moveG(pos_command)

                # Read state
                robot_state = gripper.state()

                # Store in ring buffer
                data = {
                    **robot_state,
                    "timestamp": time.now(),
                }
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False

                # Process requests
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
                        if self.max_gripper_speed is not None:
                            curr_time = t_now + dt
                            pose_interp = pose_interp.schedule_waypoint(
                                pose=[target_pos, 0, 0, 0, 0, 0],
                                time=target_time,
                                max_pos_speed=self.max_gripper_speed,
                                max_rot_speed=self.max_gripper_speed,
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
            gripper.stop()

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


def WavesharePiperGripperServer(mw, *args, **kwargs):
    return ServerFactory(mw, WavesharePiperGripper, *args, **kwargs)


def WavesharePiperGripperClient(mw, *args, **kwargs):
    return ClientFactory(mw, WavesharePiperGripper, *args, **kwargs)
