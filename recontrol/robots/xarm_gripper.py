import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..pose_trajectory_interpolator import PoseTrajectoryInterpolator
from ..request import Request

try:
    from xarm.wrapper import XArmAPI
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        XArmAPI = None  # type: ignore


class GripperModel(Enum):
    LITE6 = auto()
    G1 = auto()
    G2 = auto()
    ROBOTIQ_2F85 = auto()
    ROBOTIQ_2F140 = auto()


class GripperController(Enum):
    TASK_POS = auto()


class RequestType(Enum):
    MOVEL = auto()


class XarmGripper(Node):
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
        robot_model: str = "g1",
        robot_controller: str = "task_pos",
        move_max_speed: float = 3.0,
        home_to_open: bool = True,
        dtype=np.float32,
        *,
        freq: int = 30,
        max_buffer_size: int | None = None,
        max_queue_size: int = 128,
        **kwargs,
    ):
        robot_model = GripperModel[robot_model.upper()]
        robot_controller = GripperController[robot_controller.upper()]
        if max_buffer_size is None:
            max_buffer_size = int(freq * 10)
        self.robot_ip = robot_ip
        self.robot_model = robot_model
        self.robot_controller = robot_controller
        self.home_to_open = home_to_open
        self.move_max_speed = move_max_speed
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, max_queue_size=max_queue_size, **kwargs)

    def __post_init__(self):
        example_request_params = {
            GripperController.TASK_POS: (RequestType.MOVEL, {"target_pos": np.zeros((1,), dtype=self.dtype)}),
        }[self.robot_controller][1]
        example_request_params = {
            **example_request_params,
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

        self.gripper = XarmGripperDriver(self.robot_ip, self.robot_model, self.home_to_open)

    def pubreq(self):
        try:
            gripper = self.gripper
            gripper.start()

            if self.robot_controller == GripperController.TASK_POS:
                curr_pos = self.gripper.state()["gripper_position"]
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
                    gripper.moveL(target_pos)
                else:
                    raise ValueError(self.robot_controller)

                # get state from robot
                robot_state = gripper.state()

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
                        target_pos = np.array(req.params["target_pos"], dtype=self.dtype)[0]
                        target_time = float(req.params["target_time"])
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
            gripper.stop()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def moveL(self, target_pos, target_time):
        target_pos = np.array(target_pos, dtype=self.dtype)
        assert target_pos.shape == (1,)
        assert target_time > time.now()
        req = {
            "type": RequestType.MOVEL.value,
            "target_pos": target_pos,
            "target_time": target_time,
        }
        self.request_queue.put(req)


class XarmGripperDriver:
    def __init__(self, robot_ip, robot_model, home_to_open: bool = True):
        self.robot_ip = robot_ip
        self.robot_model = robot_model
        self.home_to_open = home_to_open

    def start(self):
        arm = XArmAPI(self.robot_ip, is_radian=True, do_not_open=True)

        arm.connect()
        arm.clean_error()
        arm.clean_warn()
        arm.motion_enable(True)
        if arm.has_err_warn:
            _, err_warn = arm.get_err_warn_code()
            if err_warn[0] != 0:
                raise RuntimeError("Check whether e-stop button is pressed.")

        self.gripper = arm
        if self.robot_model == GripperModel.LITE6:
            self.gripper.set_mode(0)
            self.gripper.set_state(0)
        elif self.robot_model in (GripperModel.G1, GripperModel.G2):
            self.gripper.set_gripper_mode(0)
            self.gripper.set_gripper_enable(True)
            # self.gripper.set_collision_tool_model(1)
        elif self.robot_model in (GripperModel.ROBOTIQ_2F85, GripperModel.ROBOTIQ_2F140):
            self.gripper.set_mode(0)
            self.gripper.set_state(0)
            self.gripper.robotiq_reset()
            self.gripper.robotiq_set_activate()
        else:
            raise ValueError(self.robot_model)
        time.sleep(0.1)

        if self.home_to_open:
            self.moveL(1.0, wait=True)  # open

    def stop(self):
        if self.robot_model == GripperModel.LITE6:
            self.gripper.stop_lite6_gripper()
        self.gripper.disconnect()

    def state(self):
        # get state from robot
        if self.robot_model == GripperModel.LITE6:
            pos = self._lite6_gripper_pos
            robot_state = {
                "gripper_position": pos,
            }
        elif self.robot_model == GripperModel.G1:
            _, pos = self.gripper.get_gripper_position()
            # [-10, 850] -> [0, 1]
            pos = (pos + 10) / 860
            robot_state = {
                "gripper_position": pos,
            }
        elif self.robot_model == GripperModel.G2:
            _, pos = self.gripper.get_gripper_g2_position()
            # [0, 84] -> [0, 1]
            pos = pos / 84
            robot_state = {
                "gripper_position": pos,
            }
        elif self.robot_model in (GripperModel.ROBOTIQ_2F85, GripperModel.ROBOTIQ_2F140):
            _, result = self.gripper.robotiq_get_status()
            pos = result[6]  # [0, 255]
            # [255, 0] -> [0, 1]
            pos = 1 - pos / 255  # 0 is open and 255 is closed
            robot_state = {
                "gripper_position": pos,
            }
        else:
            raise ValueError(self.robot_model)
        return robot_state

    def moveL(self, target_pos, wait=False):
        if self.robot_model == GripperModel.LITE6:
            assert self.gripper.mode == 0
            if target_pos > 0.5:
                self.gripper.open_lite6_gripper(sync=wait)
                self._lite6_gripper_pos = 1.0
            else:
                self.gripper.close_lite6_gripper(sync=wait)
                self._lite6_gripper_pos = 0.0
        elif self.robot_model == GripperModel.G1:
            # [0, 1] -> [-10, 850]
            pos = target_pos * 860 - 10
            self.gripper.set_gripper_position(pos, speed=5000, wait=wait)  # speed: [0, 5000]
        elif self.robot_model == GripperModel.G2:
            # [0, 1] -> [0, 84]
            pos = target_pos * 84
            self.gripper.set_gripper_g2_position(pos, speed=225, force=50, wait=wait)  # speed: [15, 225], force: [1, 100]
        elif self.robot_model in (GripperModel.ROBOTIQ_2F85, GripperModel.ROBOTIQ_2F140):
            # [0, 1] -> [255, 0]
            pos = 255 - target_pos * 255  # 0 is open and 255 is closed
            self.gripper.robotiq_set_position(pos, speed=255, force=255, wait=wait)  # speed: [0, 255], force: [0, 255]
        else:
            raise ValueError(self.robot_model)


def XarmGripperServer(mw, *args, **kwargs):
    return ServerFactory(mw, XarmGripper, *args, **kwargs)


def XarmGripperClient(mw, *args, **kwargs):
    return ClientFactory(mw, XarmGripper, *args, **kwargs)
