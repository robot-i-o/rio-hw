import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory

try:
    from gello.robots.dynamixel import DynamixelRobot
except ImportError:
    DynamixelRobot = None


class GelloInterface:
    __api__ = [
        "get_state",
        "get_all_state",
    ]
    __pub__ = True
    __req__ = False

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        joint_ids: tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7, 8),
        joint_offsets: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        joint_signs: tuple[int | float, ...] = (1, 1, 1, 1, 1, 1, 1, 1),
        gripper_config: tuple[int, float, float] = (9, 0.0, 0.0),
        start_joints: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        *,
        freq: int = 200,
        max_buffer_size: int = 30,
        dtype=np.float32,
        **kwargs,
    ):
        self.port = port
        self.baudrate = baudrate
        self.joint_ids = joint_ids
        self.joint_offsets = joint_offsets
        self.joint_signs = joint_signs
        self.gripper_config = gripper_config if gripper_config[0] != 0 else None
        self.start_joints = start_joints
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_request = None
        self.example_data = {
            "jointq": np.zeros(len(self.joint_ids), dtype=self.dtype),
            "gripper_position": 0.0,
        }
        self.worker = None
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        try:
            robot = DynamixelRobot(
                joint_ids=list(self.joint_ids),
                joint_offsets=list(self.joint_offsets),
                joint_signs=list(self.joint_signs),
                real=True,
                port=self.port,
                baudrate=self.baudrate,
                gripper_config=self.gripper_config,
                start_joints=np.array(list(self.start_joints)) if self.start_joints else None,
            )

            # Main loop
            rate = time.Rate(self.freq)
            not_pub_ready = True
            while not self.exit_event.is_set():
                # get_joint_state returns arm joints + gripper
                joint_state = robot.get_joint_state()

                # Split into joint positions and gripper position
                if self.gripper_config is not None:
                    jointq = joint_state[:-1].astype(self.dtype)
                    gripper_position = float(joint_state[-1])
                else:
                    jointq = joint_state.astype(self.dtype)
                    gripper_position = 0.0

                # Store current state in ring buffer
                data = {
                    "jointq": jointq,
                    "gripper_position": gripper_position,
                }
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False
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


def GelloServer(mw, *args, **kwargs):
    return ServerFactory(mw, GelloInterface, *args, **kwargs)


def GelloClient(mw, *args, **kwargs):
    return ClientFactory(mw, GelloInterface, *args, **kwargs)
