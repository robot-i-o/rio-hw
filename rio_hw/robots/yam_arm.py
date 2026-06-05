import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request

try:
    from i2rt.robots.get_robot import get_yam_robot
    from i2rt.robots.utils import GripperType
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        get_yam_robot = None  # type: ignore
        GripperType = None  # type: ignore


NUM_JOINTS = 6
STABILIZE_TIME = 3.0


class RequestType(Enum):
    MOVEJ = auto()
    MOVEG = auto()


class YamArm(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "moveJ",
        "moveG",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        channel: str = "can0",
        gripper_type: str = "linear_4310",
        zero_gravity_mode: bool = False,
        dtype=np.float32,
        *,
        freq: int = 50,
        max_buffer_size: int | None = None,
        **kwargs,
    ):
        assert 0 < freq <= 250
        if max_buffer_size is None:
            max_buffer_size = int(freq * 5)
        self.channel = channel
        self.gripper_type = gripper_type.upper()
        self.zero_gravity_mode = zero_gravity_mode
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_request = {
            "type": RequestType.MOVEJ.value,
            "target_joint_q": np.zeros((NUM_JOINTS,), dtype=self.dtype),
            "target_time": time.now(),
        }
        self.example_data = {
            "joint_q": np.zeros((NUM_JOINTS,), dtype=self.dtype),
            "gripper_position": np.float32(0.0),
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pubreq
        super().__post_init__()

    def pubreq(self):
        arm = get_yam_robot(
            channel=self.channel,
            zero_gravity_mode=self.zero_gravity_mode,
            gripper_type=GripperType[self.gripper_type],
        )
        time.sleep(STABILIZE_TIME)

        n_dofs = len(arm.get_joint_pos())
        has_gripper = n_dofs > NUM_JOINTS
        has_encoder = hasattr(arm, "motor_chain") and arm.motor_chain.same_bus_device_driver is not None
        target = np.array(arm.get_joint_pos(), dtype=self.dtype)

        try:
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                joint_pos = arm.get_joint_pos()
                gripper = np.float32(joint_pos[NUM_JOINTS]) if has_gripper else np.float32(0.0)
                if has_encoder:
                    states = arm.motor_chain.same_bus_device_states
                    if states is not None and len(states) > 0:
                        gripper = np.float32(1 - states[0].position)
                self.ring_buffer.put(
                    {
                        "joint_q": np.array(joint_pos[:NUM_JOINTS], dtype=self.dtype),
                        "gripper_position": gripper,
                        "timestamp": time.now(),
                    }
                )
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False

                try:
                    reqs = self.request_queue.get_all()
                    if isinstance(reqs, dict):
                        reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
                except queue.Empty:
                    reqs = []
                for r in reqs:
                    req = Request(RequestType(r.pop("type")), r)
                    if req.type == RequestType.MOVEJ:
                        target[:NUM_JOINTS] = np.array(req.params["target_joint_q"], dtype=self.dtype)
                    elif req.type == RequestType.MOVEG:
                        if has_gripper:
                            target[NUM_JOINTS] = float(req.params["target_pos"][0])
                    else:
                        raise ValueError(f"Unknown request type: {req.type}")

                if not self.zero_gravity_mode:
                    arm.command_joint_pos(target[:n_dofs])

                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            try:
                arm.close()
            except Exception:
                pass

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def moveJ(self, target_joint_q, target_time):
        target_joint_q = np.array(target_joint_q, dtype=self.dtype)
        assert target_joint_q.shape == (NUM_JOINTS,)
        assert target_time > time.now()
        self.request_queue.put(
            {
                "type": RequestType.MOVEJ.value,
                "target_joint_q": target_joint_q,
                "target_time": target_time,
            }
        )

    def moveG(self, target_pos, target_time):
        target_pos = np.array(target_pos, dtype=self.dtype)
        assert target_pos.shape == (1,)
        assert target_time > time.now()
        self.request_queue.put(
            {
                "type": RequestType.MOVEG.value,
                "target_pos": target_pos,
                "target_time": target_time,
            }
        )


def YamArmServer(mw, *args, **kwargs):
    return ServerFactory(mw, YamArm, *args, **kwargs)


def YamArmClient(mw, *args, **kwargs):
    return ClientFactory(mw, YamArm, *args, **kwargs)
