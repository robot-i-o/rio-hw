import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request

try:
    import unitree_interface
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        unitree_interface = None  # type: ignore

# fmt: off
DEFAULT_KP = (
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0.,
    0., 0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0., 0.,
)

DEFAULT_KD = (
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0.,
    0., 0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0., 0.,
)
# fmt: on


class RobotModel(Enum):
    G1_29 = auto()


RobotInfo = {
    RobotModel.G1_29: {"num_joints": 29},
}


class RobotController(Enum):
    JOINT_POS = auto()


class RequestType(Enum):
    MOVEJ = auto()


class UnitreeG1(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "moveJ",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        robot_iface: str = "eth0",
        robot_model: str = "g1_29",
        robot_controller: str = "joint_pos",
        motor_kp: tuple[float, ...] = DEFAULT_KP,
        motor_kd: tuple[float, ...] = DEFAULT_KD,
        kp_scale: float = 1.0,
        kd_scale: float = 1.0,
        *,
        freq: int = 500,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        robot_model = RobotModel[robot_model.upper()]
        robot_controller = RobotController[robot_controller.upper()]
        num_joints = RobotInfo[robot_model]["num_joints"]
        self.robot_iface = robot_iface
        self.robot_model = robot_model
        self.robot_controller = robot_controller
        self.num_joints = num_joints
        self.motor_kp = np.array(motor_kp)
        self.motor_kd = np.array(motor_kd)
        self.kp_scale = kp_scale
        self.kd_scale = kd_scale
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        print(f"G1 motor_kp: {self.motor_kp}")
        print(f"G1 motor_kd: {self.motor_kd}")

        example_request_params = {
            "target_joint_q": np.zeros((self.num_joints,), dtype=np.float32),
        }
        request_params_keys = {
            RobotController.JOINT_POS: (RequestType.MOVEJ, ("target_joint_q",)),
        }[self.robot_controller][1]
        example_request_params = {k: example_request_params[k] for k in request_params_keys}
        example_request_params["target_time"] = time.now()

        example_robot_state = {
            # Motor data
            "joint_q": np.zeros(shape=(self.num_joints,), dtype=np.float32),
            "joint_qd": np.zeros(shape=(self.num_joints,), dtype=np.float32),
            "joint_torque": np.zeros(shape=(self.num_joints,), dtype=np.float32),
            "joint_temperature": np.zeros(shape=(self.num_joints,), dtype=np.float32),
            "joint_voltage": np.zeros(shape=(self.num_joints,), dtype=np.float32),
            # IMU data
            "imu_quat": np.zeros(shape=(4,), dtype=np.float32),
            "imu_gyro": np.zeros(shape=(3,), dtype=np.float32),
            "imu_accel": np.zeros(shape=(3,), dtype=np.float32),
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
        robot = unitree_interface.create_robot(
            self.robot_iface,
            unitree_interface.RobotType.G1,
            unitree_interface.MessageType.HG,
        )
        robot.set_control_mode(unitree_interface.ControlMode.PR)
        target_joint_q = None

        try:
            # Main loop
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                if target_joint_q is not None:
                    cmd = robot.create_zero_command()
                    cmd.q_target = list(target_joint_q)
                    cmd.kp = list(self.kp_scale * self.motor_kp)
                    cmd.kd = list(self.kd_scale * self.motor_kd)
                    robot.write_low_command(cmd)

                state = robot.read_low_state()
                if state is not None:
                    robot_state = {
                        "joint_q": np.array(state.motor.q),
                        "joint_qd": np.array(state.motor.dq),
                        "joint_torque": np.array(state.motor.tau_est),
                        "joint_temperature": np.array(state.motor.temperature),
                        "joint_voltage": np.array(state.motor.voltage),
                        "imu_quat": np.array(state.imu.quat)[[1, 2, 3, 0]],  # (w, x, y, z) -> (x, y, z, w)
                        "imu_gyro": np.array(state.imu.omega),
                        "imu_accel": np.array(state.imu.accel),
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
                    if req.type == RequestType.MOVEJ:
                        target_joint_q = req.params.get("target_joint_q")
                    else:
                        raise ValueError(req.type)
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            pass

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def moveJ(self, target_joint_q, target_time):
        target_joint_q = np.array(target_joint_q, dtype=np.float32)
        assert target_joint_q.shape == (self.num_joints,)
        req = {
            "type": RequestType.MOVEJ.value,
            "target_joint_q": target_joint_q,
            "target_time": target_time,
        }
        self.request_queue.put(req)


def UnitreeG1Server(mw, *args, **kwargs):
    return ServerFactory(mw, UnitreeG1, *args, **kwargs)


def UnitreeG1Client(mw, *args, **kwargs):
    return ClientFactory(mw, UnitreeG1, *args, **kwargs)
