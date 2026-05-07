import os  # noqa: I001
import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np
from loguru import logger
from scipy.spatial.transform import Rotation

from .. import time
from ..filters import LowPassFilter
from ..interpolators import PoseTrajectoryInterpolator, TrajectoryInterpolator
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request

try:
    from pyAgxArm import AgxArmFactory, create_agx_arm_config
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        AgxArmFactory = None  # type: ignore
        create_agx_arm_config = None  # type: ignore


class RobotModel(Enum):
    PIPER = auto()
    PIPER_H = auto()
    PIPER_L = auto()
    PIPER_X = auto()
    NERO = auto()


RobotInfo = {
    RobotModel.PIPER: {"num_joints": 6},
    RobotModel.PIPER_H: {"num_joints": 6},
    RobotModel.PIPER_L: {"num_joints": 6},
    RobotModel.PIPER_X: {"num_joints": 6},
    RobotModel.NERO: {"num_joints": 7},
}

# Map enum name → SDK robot string (e.g. PIPER_H → "piper_h")
_MODEL_TO_SDK = {
    m: m.name.lower()
    .replace("_", "-")
    .replace("piper-h", "piper_h")
    .replace("piper-l", "piper_l")
    .replace("piper-x", "piper_x")
    for m in RobotModel
}
_MODEL_TO_SDK = {
    RobotModel.PIPER: "piper",
    RobotModel.PIPER_H: "piper_h",
    RobotModel.PIPER_L: "piper_l",
    RobotModel.PIPER_X: "piper_x",
    RobotModel.NERO: "nero",
}


class RobotController(Enum):
    JOINT_POS = auto()
    TASK_POS = auto()


class RequestType(Enum):
    MOVEJ = auto()
    MOVEL = auto()
    MOVE_GRIPPER = auto()


_JOINTS_INIT_TIMEOUT = 10.0  # seconds
_JOINTS_INIT_TOLERANCE = 0.05  # radians


class PiperArm(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "moveL",
        "moveJ",
        "move_gripper",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        robot_model: str = "piper",
        can_channel: str = "can0",
        can_interface: str = "socketcan",
        can_bitrate: int = 1_000_000,
        robot_controller: str = "joint_pos",
        max_pos_speed: float = 0.25,  # m/s
        max_rot_speed: float = 0.785,  # rad/s
        max_motor_speed: float = 1.0,  # rad/s
        speed_percent: int = 30,
        joints_init: list | None = None,
        joints_lowpass_alpha: float = 0.1,
        soft_real_time: bool = False,
        with_gripper: bool = True,
        gripper_max_range: float = 0.07,  # meters
        dtype=np.float32,
        *,
        freq: int = 50,
        max_buffer_size: int | None = None,
        **kwargs,
    ):
        assert 0 < freq <= 500
        assert 0 < max_pos_speed
        assert 0 < max_rot_speed
        assert 0 < max_motor_speed
        assert 0 < speed_percent <= 100
        assert 0 < gripper_max_range

        robot_model_key = robot_model.upper().replace("-", "_")
        robot_model_enum = RobotModel[robot_model_key]
        robot_controller = RobotController[robot_controller.upper()]
        num_joints = RobotInfo[robot_model_enum]["num_joints"]

        if max_buffer_size is None:
            max_buffer_size = int(freq * 5)

        if joints_init is not None:
            joints_init = np.array(joints_init, dtype=dtype)
            assert joints_init.shape == (num_joints,)

        self.robot_model = robot_model_enum
        self.can_channel = can_channel
        self.can_interface = can_interface
        self.can_bitrate = can_bitrate
        self.robot_controller = robot_controller
        self.num_joints = num_joints
        self.max_pos_speed = max_pos_speed
        self.max_rot_speed = max_rot_speed
        self.max_motor_speed = max_motor_speed
        self.speed_percent = speed_percent
        self.joints_init = joints_init
        self.joints_lowpass_alpha = joints_lowpass_alpha
        self.soft_real_time = soft_real_time
        self.with_gripper = with_gripper
        self.gripper_max_range = gripper_max_range
        self.dtype = dtype

        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_request_params = {
            RobotController.JOINT_POS: {
                "target_joint_q": np.zeros((self.num_joints,), dtype=self.dtype),
                "target_time": time.now(),
            },
            RobotController.TASK_POS: {
                "target_tcp_pose": np.zeros((6,), dtype=self.dtype),
                "target_time": time.now(),
            },
        }[self.robot_controller]

        self.example_request = {
            "type": next(iter(RequestType)).value,
            **example_request_params,
        }

        example_data = {
            "joint_q": np.zeros((self.num_joints,), dtype=self.dtype),
            "tcp_pose": np.zeros((6,), dtype=self.dtype),
            "timestamp": time.now(),
        }
        if self.with_gripper:
            example_data["gripper_position"] = np.float32(0.0)
        self.example_data = example_data

        self.worker = None
        self.run = self.pubreq
        super().__post_init__()

    def pubreq(self):
        if self.soft_real_time:
            os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(20))

        # Phase A: Initialize hardware
        cfg = create_agx_arm_config(
            robot=_MODEL_TO_SDK[self.robot_model],
            comm="can",
            channel=self.can_channel,
            interface=self.can_interface,
            bitrate=self.can_bitrate,
        )
        robot = AgxArmFactory.create_arm(cfg)

        end_effector = None
        if self.with_gripper:
            end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)

        robot.connect()

        try:
            # Phase B: Wait for SDK data + enable
            logger.info("Waiting for Piper arm SDK data...")
            while robot.get_joint_angles() is None:
                time.sleep(0.02)

            robot.enable()
            robot.set_speed_percent(self.speed_percent)

            # Phase C: Move to initial pose if specified
            if self.joints_init is not None:
                logger.info("Moving Piper arm to joints_init...")
                robot.move_j(self.joints_init.tolist())
                deadline = time.now() + _JOINTS_INIT_TIMEOUT
                while time.now() < deadline:
                    ja = robot.get_joint_angles()
                    if ja is not None:
                        curr = np.array(ja.msg, dtype=self.dtype)
                        if np.max(np.abs(curr - self.joints_init)) < _JOINTS_INIT_TOLERANCE:
                            break
                    time.sleep(0.05)
                else:
                    logger.warning("joints_init timeout — continuing anyway")

            # Phase D: Build interpolator from current state
            curr_t = time.now()
            last_waypoint_time = curr_t

            if self.robot_controller == RobotController.JOINT_POS:
                ja = robot.get_joint_angles()
                curr_joint_q = np.array(ja.msg, dtype=self.dtype)
                joint_interp = TrajectoryInterpolator(times=[curr_t], values=[curr_joint_q])
                lowpass_filter = LowPassFilter(alpha=self.joints_lowpass_alpha, initial=curr_joint_q)

            elif self.robot_controller == RobotController.TASK_POS:
                fp = robot.get_flange_pose()
                curr_pose_rpy = np.array(fp.msg, dtype=self.dtype)
                # Convert RPY → axis-angle for PoseTrajectoryInterpolator
                curr_pose = curr_pose_rpy.copy()
                curr_pose[3:] = Rotation.from_euler("xyz", curr_pose_rpy[3:]).as_rotvec()
                pose_interp = PoseTrajectoryInterpolator(times=[curr_t], poses=[curr_pose])

            else:
                raise ValueError(f"Unsupported controller: {self.robot_controller}")

            # Phase E: Ready signals and main control loop
            dt = 1.0 / self.freq
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True

            while not self.exit_event.is_set():
                t_now = time.now()

                # (a) Send command to robot
                if self.robot_controller == RobotController.JOINT_POS:
                    cmd = joint_interp(t_now)
                    cmd = lowpass_filter(cmd)
                    robot.move_js(cmd.tolist())

                elif self.robot_controller == RobotController.TASK_POS:
                    cmd = pose_interp(t_now)
                    # Convert axis-angle → RPY for SDK
                    rpy = Rotation.from_rotvec(cmd[3:]).as_euler("xyz")
                    robot.move_l(cmd[:3].tolist() + rpy.tolist())

                # (b) Read state → ring buffer
                ja = robot.get_joint_angles()
                fp = robot.get_flange_pose()
                joint_q = np.array(ja.msg, dtype=self.dtype) if ja is not None else np.zeros(self.num_joints, dtype=self.dtype)
                tcp_pose = np.array(fp.msg, dtype=self.dtype) if fp is not None else np.zeros(6, dtype=self.dtype)

                data = {
                    "joint_q": joint_q,
                    "tcp_pose": tcp_pose,
                    "timestamp": time.now(),
                }
                if self.with_gripper and end_effector is not None:
                    gs = end_effector.get_gripper_status()
                    width = gs.msg.width if gs is not None else 0.0
                    data["gripper_position"] = np.float32(np.clip(width / self.gripper_max_range, 0.0, 1.0))
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False

                # (c) Drain request queue
                try:
                    reqs = self.request_queue.get_all()
                    if isinstance(reqs, dict):
                        reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
                except queue.Empty:
                    reqs = []

                for r in reqs:
                    req = Request(RequestType(r.pop("type")), r)

                    if req.type == RequestType.MOVEJ:
                        target_joint_q = np.array(req.params["target_joint_q"], dtype=self.dtype)
                        target_time = float(req.params["target_time"])
                        curr_time = t_now + dt
                        if target_time < curr_time:
                            continue
                        joint_interp = joint_interp.schedule_waypoint(
                            value=target_joint_q,
                            time=target_time,
                            max_speed=self.max_motor_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time,
                        )
                        last_waypoint_time = target_time

                    elif req.type == RequestType.MOVEL:
                        target_tcp_pose = np.array(req.params["target_tcp_pose"], dtype=self.dtype)
                        target_time = float(req.params["target_time"])
                        curr_time = t_now + dt
                        if target_time < curr_time:
                            continue
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=target_tcp_pose,
                            time=target_time,
                            max_pos_speed=self.max_pos_speed,
                            max_rot_speed=self.max_rot_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time,
                        )
                        last_waypoint_time = target_time

                    elif req.type == RequestType.MOVE_GRIPPER:
                        if end_effector is not None:
                            normalized = float(req.params["gripper_position"])
                            meters = np.clip(normalized * self.gripper_max_range, 0.0, self.gripper_max_range)
                            end_effector.move_gripper(meters)
                        else:
                            logger.warning("MOVE_GRIPPER requested but gripper not initialized")

                    else:
                        raise ValueError(f"Unknown request type: {req.type}")

                rate.sleep()

        except KeyboardInterrupt:
            pass
        finally:
            try:
                robot.disable()
            except Exception:
                pass
            try:
                robot.disconnect()
            except Exception:
                pass

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def moveJ(self, target_joint_q, target_time):
        assert self.robot_controller == RobotController.JOINT_POS, "moveJ requires robot_controller='joint_pos'"
        target_joint_q = np.array(target_joint_q, dtype=self.dtype)
        assert target_joint_q.shape == (self.num_joints,)
        assert target_time > time.now()
        self.request_queue.put(
            {
                "type": RequestType.MOVEJ.value,
                "target_joint_q": target_joint_q,
                "target_time": target_time,
            }
        )

    def moveL(self, target_tcp_pose, target_time):
        assert self.robot_controller == RobotController.TASK_POS, "moveL requires robot_controller='task_pos'"
        target_tcp_pose = np.array(target_tcp_pose, dtype=self.dtype)
        assert target_tcp_pose.shape == (6,)
        assert target_time > time.now()
        self.request_queue.put(
            {
                "type": RequestType.MOVEL.value,
                "target_tcp_pose": target_tcp_pose,
                "target_time": target_time,
            }
        )

    def move_gripper(self, gripper_position: float):
        """Move gripper. gripper_position: normalized [0=closed, 1=open]."""
        if not self.with_gripper:
            raise RuntimeError("Gripper not initialized; set with_gripper=True")
        gripper_position = float(gripper_position)
        assert 0.0 <= gripper_position <= 1.0
        self.request_queue.put(
            {
                "type": RequestType.MOVE_GRIPPER.value,
                "gripper_position": gripper_position,
            }
        )

    def speedJ(self, target_joint_qd, target_time):
        raise NotImplementedError("speedJ not implemented for PiperArm")

    def speedL(self, target_tcp_twist, target_time):
        raise NotImplementedError("speedL not implemented for PiperArm")


def PiperArmServer(mw, *args, **kwargs):
    return ServerFactory(mw, PiperArm, *args, **kwargs)


def PiperArmClient(mw, *args, **kwargs):
    return ClientFactory(mw, PiperArm, *args, **kwargs)
