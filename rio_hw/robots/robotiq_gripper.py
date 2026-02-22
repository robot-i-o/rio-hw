import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..interpolators import PoseTrajectoryInterpolator
from ..middleware import ClientFactory, ServerFactory
from ..request import Request

try:
    from .utils.robotiq_driver import RobotiqDriver
    from .utils.robotiq_tcp_driver import RobotiqTcpDriver
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        RobotiqDriver = None
        RobotiqTcpDriver = None


GripperInfo = {
    "robotiq_2f85": {"range": (0, 85)},
    "robotiq_2f140": {"range": (0, 140)},
}


class GripperController(Enum):
    TASK_POS = "task_pos"


class RequestType(Enum):
    MOVEG = auto()


class MODBUSMode(Enum):
    SERIAL = auto()
    TCPIP = auto()


class RobotiqGripper:
    """
    Robot agnostic interface for Robotiq Gripper, controlled via serial port.
    """

    __api__ = [
        "get_state",
        "get_all_state",
        "moveG",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        port: str = "auto",
        model: str = "robotiq_2f140",
        control_mode: str = "bits",
        calibrate: bool = False,
        robot_controller: str = "task_pos",
        max_gripper_speed: float | None = 3.0,
        modbus_mode: str = "serial",
        dtype=np.float64,
        *,
        freq: int = 50,
        max_buffer_size: int | None = None,
        max_queue_size: int = 1,
        **kwargs,
    ):
        """
        Args:
            port (str, optional): Serial port for the gripper. Defaults to "auto".
            model (str, optional): Gripper model. Defaults to "robotiq_2f140".
            control_mode (str, optional): Control mode for the gripper. Defaults to "bits".
            calibrate (bool, optional): Whether to calibrate the gripper. Defaults to False.
            robot_controller (str, optional): Robot controller type. Defaults to "task_pos".
            dtype (_type_, optional): Data type for the gripper. Defaults to np.float64.
            freq (int, optional): Defaults to 30.
            max_buffer_size (int | None, optional): Defaults to None.
            max_queue_size (int, optional): Defaults to 1024.
        """
        if max_buffer_size is None:
            max_buffer_size = int(freq * 10)
        self.port = port
        self.model = model
        self.mm_range = GripperInfo[model]["range"]
        self.control_mode = control_mode
        self.calibrate = calibrate
        self.robot_controller = GripperController(robot_controller)
        self.max_gripper_speed = max_gripper_speed
        self.modbus_mode = MODBUSMode[modbus_mode.upper()]
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, max_queue_size=max_queue_size, **kwargs)

    def __post_init__(self):
        example_request_params = {
            GripperController.TASK_POS: (RequestType.MOVEG, {"target_pose": np.zeros((1,), dtype=self.dtype)}),
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

    def pubreq(self):
        try:
            if self.modbus_mode == MODBUSMode.SERIAL:
                gripper = RobotiqDriver(
                    port=self.port,
                    model=self.model,
                    control_mode=self.control_mode,
                    calibrate=self.calibrate,
                    freq=self.freq,
                )
            elif self.modbus_mode == MODBUSMode.TCPIP:
                gripper = RobotiqTcpDriver(
                    robot_ip=self.port,
                    freq=self.freq,
                )
            else:
                raise ValueError(self.modbus_mode)
            gripper.start()

            if self.robot_controller == GripperController.TASK_POS:
                curr_pos = gripper.state()["gripper_position"]
                if self.max_gripper_speed is not None:
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
                if self.robot_controller == GripperController.TASK_POS:
                    if pose_interp is not None:
                        pos_command = pose_interp(t_now)[0]
                    else:
                        pos_command = np.copy(target_pos)
                    gripper.moveG(pos_command)
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
                    if req.type == RequestType.MOVEG:
                        target_pos = np.array(req.params["target_pose"], dtype=self.dtype)[0]
                        target_time = float(req.params["target_time"])
                        if pose_interp is not None:
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

    def moveG(self, target_pose, target_time):
        target_pose = np.array(target_pose, dtype=self.dtype)
        assert target_pose.shape == (1,)
        assert target_time > time.now()
        req = {
            "type": RequestType.MOVEG.value,
            "target_pose": target_pose,
            "target_time": target_time,
        }
        self.request_queue.put(req)


def RobotiqGripperServer(mw, *args, **kwargs):
    return ServerFactory(mw, RobotiqGripper, *args, **kwargs)


def RobotiqGripperClient(mw, *args, **kwargs):
    return ClientFactory(mw, RobotiqGripper, *args, **kwargs)
