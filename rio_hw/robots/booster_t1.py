import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request

try:
    from booster_robotics_sdk_python import (
        B1LocoClient,
        B1LowCmdPublisher,
        B1LowStateSubscriber,
        ChannelFactory,
        LowCmd,
        LowCmdType,
        MotorCmd,
        RobotMode,
    )
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        B1LocoClient = None  # type: ignore
        B1LowCmdPublisher = None  # type: ignore
        B1LowStateSubscriber = None  # type: ignore
        ChannelFactory = None  # type: ignore
        LowCmd = None  # type: ignore
        LowCmdType = None  # type: ignore
        MotorCmd = None  # type: ignore
        RobotMode = None  # type: ignore

# fmt: off
DEFAULT_KP = (
    0., 0.,
    0., 0., 0., 0.,
    0., 0., 0., 0.,
    0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
)

DEFAULT_KD = (
    0., 0.,
    0., 0., 0., 0.,
    0., 0., 0., 0.,
    0.,
    0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0.,
)
# fmt: on


class RobotModel(Enum):
    T1_23 = auto()


RobotInfo = {
    RobotModel.T1_23: {"num_joints": 23},
}


class RequestType(Enum):
    MOVEJ = auto()


class BoosterT1(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "moveJ",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        motor_kp: tuple[float, ...] = DEFAULT_KP,
        motor_kd: tuple[float, ...] = DEFAULT_KD,
        motor_type: str = "serial",
        robot_model: str = "t1_23",
        network_interface: str | None = None,
        domain_id: int = 0,
        kp_scale: float = 1.0,
        kd_scale: float = 1.0,
        *,
        freq: int = 500,
        max_buffer_size: int | None = 500,
        max_queue_size: int = 500,
        **kwargs,
    ):
        robot_model = RobotModel[robot_model.upper()]
        self.num_joints = RobotInfo[robot_model]["num_joints"]

        self.motor_type = LowCmdType[motor_type.upper()]
        self.motor_kp = np.array(motor_kp)
        self.motor_kd = np.array(motor_kd)
        self.default_motor_angles = np.zeros(self.num_joints)
        self.network_interface = network_interface
        self.kp_scale = kp_scale
        self.kd_scale = kd_scale
        self.domain_id = domain_id

        super().__init__(freq=freq, max_buffer_size=max_buffer_size, max_queue_size=max_queue_size, **kwargs)

    def __post_init__(self):
        self.example_request = {
            "type": next(iter(RequestType)).value,
            "target_joint_q": np.zeros(shape=(self.num_joints,), dtype=np.float32),  # target position
            "target_time": 0.0,
        }

        example_robot_state = {
            "joint_q": np.zeros((self.num_joints,), dtype=np.float32),
            "joint_dq": np.zeros((self.num_joints,), dtype=np.float32),
            "rpy": np.zeros((3,), dtype=np.float32),
            "gyro": np.zeros((3,), dtype=np.float32),
        }

        self.example_data = {
            **example_robot_state,
            "timestamp": time.now(),
        }

        self.worker = None
        self.run = self.pubreq

        super().__post_init__()

    def _low_state_handler(self, msg):
        self.latest_msg = msg

    def _process_msg(self, msg):
        if self.motor_type == LowCmdType.SERIAL:
            motor_state = getattr(msg, "motor_state_serial", None)
        elif self.motor_type == LowCmdType.PARALLEL:
            motor_state = getattr(msg, "motor_state_parallel", None)
        else:
            raise ValueError(self.motor_type)

        if motor_state is None:
            return None

        joint_pos = np.zeros(self.num_joints)
        joint_vel = np.zeros(self.num_joints)
        for j_id in range(self.num_joints):
            joint_pos[j_id] = float(motor_state[j_id].q)
            joint_vel[j_id] = float(motor_state[j_id].dq)

        return {
            "joint_q": joint_pos,
            "joint_dq": joint_vel,
            "rpy": msg.imu_state.rpy,
            "gyro": msg.imu_state.gyro,
        }

    def pubreq(self):
        ChannelFactory.Instance().Init(self.domain_id)

        client = B1LocoClient()
        client.Init()

        lowcmd_publisher = B1LowCmdPublisher()
        lowcmd_publisher.InitChannel()

        lowstate_subscriber = B1LowStateSubscriber(self._low_state_handler)
        lowstate_subscriber.InitChannel()

        # Allow SDK to initialize before changing mode
        time.sleep(1.0)

        client.ChangeMode(RobotMode.kCustom)

        rate = time.Rate(self.freq)
        self.req_ready_event.set()

        motor_cmds = [MotorCmd() for _ in range(self.num_joints)]

        low_cmd = LowCmd()
        low_cmd.cmd_type = self.motor_type
        low_cmd.motor_cmd = motor_cmds

        target_q = None
        not_pub_ready = True

        self.latest_msg = None

        while not self.exit_event.is_set():
            # publish robot state
            if self.latest_msg is not None:
                robot_state = self._process_msg(self.latest_msg)
                if robot_state is not None:
                    data = {
                        **robot_state,
                        "timestamp": time.now(),
                    }
                    self.ring_buffer.put(data)
                    if not_pub_ready:
                        self.pub_ready_event.set()
                        not_pub_ready = False

            # process robot commands
            try:
                reqs = self.request_queue.get_all()
                if isinstance(reqs, dict):
                    reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
            except queue.Empty:
                reqs = []

            for r in reqs:
                req = Request(RequestType(r.pop("type")), r)
                if req.type == RequestType.MOVEJ:
                    target_q = np.array(req.params.get("target_joint_q")).copy()
                else:
                    raise ValueError(req.type)

            if target_q is not None:
                for i in range(self.num_joints):
                    motor_cmds[i].q = float(target_q[i])
                    motor_cmds[i].dq = 0.0
                    motor_cmds[i].tau = 0.0
                    motor_cmds[i].kp = float(self.motor_kp[i] * self.kp_scale)
                    motor_cmds[i].kd = float(self.motor_kd[i] * self.kd_scale)
                    motor_cmds[i].weight = 0.0

            lowcmd_publisher.Write(low_cmd)

            rate.sleep()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def moveJ(self, target_joint_q, target_time):
        target_joint_q = np.array(target_joint_q)
        assert target_joint_q.shape == (self.num_joints,)

        req = {
            "type": RequestType.MOVEJ.value,
            "target_joint_q": target_joint_q,
            "target_time": target_time,
        }
        self.request_queue.put(req)


def BoosterT1Server(mw, *args, **kwargs):
    """Factory function to create BoosterT1 server."""
    return ServerFactory(mw, BoosterT1, *args, **kwargs)


def BoosterT1Client(mw, *args, **kwargs):
    """Factory function to create BoosterT1 client."""
    return ClientFactory(mw, BoosterT1, *args, **kwargs)
