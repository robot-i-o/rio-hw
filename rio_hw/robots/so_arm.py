import os  # noqa: I001
import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..filters import LowPassFilter
from ..interpolators import PoseTrajectoryInterpolator, TrajectoryInterpolator
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request
from loguru import logger

try:
    from pysoarm.driver import SOArmDriver
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        SOArmDriver = None  # type: ignore


NUM_JOINTS = 5


class RobotController(Enum):
    TASK_POS = auto()
    JOINT_POS = auto()
    TASK_VEL = auto()
    JOINT_VEL = auto()


class RequestType(Enum):
    MOVEL = auto()
    MOVEJ = auto()
    SPEEDL = auto()
    SPEEDJ = auto()
    MOVE_GRIPPER = auto()


class SoArm(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "moveL",
        "moveJ",
        "speedL",
        "speedJ",
        "move_gripper",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 1_000_000,
        model: str = "so101",
        robot_controller: str = "task_pos",
        max_pos_speed: float = 0.25,  # m/s
        max_rot_speed: float = 0.785,  # rad/s
        max_motor_speed: float = 3.1415,  # rad/s
        arm_id: str | None = None,
        calibration_file: str | None = None,
        joint_units: str = "radians",
        urdf_path: str | None = None,
        target_frame_name: str = "gripper_frame_link",
        joints_init: list | None = None,
        joints_init_speed: float = 500.0,
        joints_lowpass_alpha: float = 0.1,
        soft_real_time: bool = False,
        motors_enabled: bool = True,
        integrated_gripper: bool = True,
        use_waypoint_interpolation: bool = False,
        dtype=np.float32,
        *,
        freq: int = 50,
        max_buffer_size: int | None = None,
        **kwargs,
    ):
        assert 0 < freq <= 250
        assert 0 < max_pos_speed
        assert 0 < max_rot_speed
        assert 0 < max_motor_speed

        robot_controller = RobotController[robot_controller.upper()]

        if robot_controller in (RobotController.TASK_POS, RobotController.TASK_VEL):
            if urdf_path is None:
                raise ValueError(
                    f"robot_controller={robot_controller.name!r} requires a URDF for kinematics; pass urdf_path=<path>"
                )

        self.integrated_gripper = integrated_gripper

        if max_buffer_size is None:
            max_buffer_size = int(freq * 5)
        if joints_init is not None:
            joints_init = np.array(joints_init, dtype=dtype)
            assert joints_init.shape == (NUM_JOINTS,)

        self.port = port
        self.baudrate = baudrate
        self.model = model
        self.robot_controller = robot_controller
        self.max_pos_speed = max_pos_speed
        self.max_rot_speed = max_rot_speed
        self.max_motor_speed = max_motor_speed
        self.arm_id = arm_id
        self.calibration_file = calibration_file
        self.joint_units = joint_units
        self.urdf_path = urdf_path
        self.target_frame_name = target_frame_name
        self.joints_init = joints_init
        self.joints_init_speed = joints_init_speed
        self.joints_lowpass_alpha = joints_lowpass_alpha
        self.soft_real_time = soft_real_time
        self.motors_enabled = motors_enabled
        self.use_waypoint_interpolation = use_waypoint_interpolation

        if not self.use_waypoint_interpolation:
            logger.warning("Waypoint interpolation is disabled")
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def _make_arm(self) -> "SOArmDriver":
        ee_control = self.robot_controller in (RobotController.TASK_POS, RobotController.TASK_VEL)
        return SOArmDriver(
            port=self.port,
            baudrate=self.baudrate,
            calibration_file=self.calibration_file,
            urdf_path=self.urdf_path if ee_control else None,
            model=self.model,
            joint_units=self.joint_units,
            target_frame_name=self.target_frame_name,
            arm_id=self.arm_id,
            motors_enabled=self.motors_enabled,
        )

    def __post_init__(self):
        example_request_params = {
            "target_tcp_pose": np.zeros((6,), dtype=self.dtype),
            "target_joint_q": np.zeros((NUM_JOINTS + 1,), dtype=self.dtype),
            "target_tcp_twist": np.zeros((6,), dtype=self.dtype),
            "target_joint_qd": np.zeros((NUM_JOINTS + 1,), dtype=self.dtype),
            "gripper_position": np.float32(0.0),
        }
        request_params_keys = {
            RobotController.TASK_POS: (RequestType.MOVEL, ("target_tcp_pose",)),
            RobotController.JOINT_POS: (RequestType.MOVEJ, ("target_joint_q",)),
            RobotController.TASK_VEL: (RequestType.SPEEDL, ("target_tcp_twist",)),
            RobotController.JOINT_VEL: (RequestType.SPEEDJ, ("target_joint_qd",)),
        }[self.robot_controller][1]
        example_request_params = {k: example_request_params[k] for k in request_params_keys}
        example_request_params["target_time"] = time.now()

        self.example_request = {
            "type": next(iter(RequestType)).value,
            **example_request_params,
        }
        self.example_data = {
            "joint_q": np.zeros((NUM_JOINTS,), dtype=self.dtype),
            "tcp_pose": np.zeros((6,), dtype=self.dtype),
            "gripper_position": np.float32(0.0),
            "timestamp": time.now(),
        }

        self.worker = self.pub
        self.run = self.req
        super().__post_init__()

    def pub(self):
        """Publisher thread that reads robot state at high frequency."""
        arm = self._make_arm()

        try:
            arm.connect()
            while not arm._connected:
                time.sleep(0.1)
            # Main loop
            rate = time.Rate(self.freq)
            not_pub_ready = True
            while not self.exit_event.is_set():
                # Read robot state
                robot_state = arm.get_state()

                # Store current state in ring buffer
                ee_pose = robot_state["end_effector_pose"]
                data = {
                    "joint_q": np.array(robot_state["joint_positions"], dtype=self.dtype),
                    "tcp_pose": np.array(ee_pose, dtype=self.dtype) if ee_pose is not None else np.zeros(6, dtype=self.dtype),
                    "gripper_position": np.float32(robot_state["gripper_position"]),
                    "timestamp": time.now(),
                }
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            arm.disconnect()

    def req(self):
        """Request handler thread that sends commands to the robot."""
        # enable soft real-time
        if self.soft_real_time:
            os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(20))

        arm = self._make_arm()

        try:
            arm.connect()

            # Move to initial position if specified
            if self.joints_init is not None:
                arm.moveJ(self.joints_init.tolist(), speed=self.joints_init_speed)
                time.sleep(0.5)

            if self.robot_controller == RobotController.TASK_POS:
                curr_pose = arm.get_end_effector_pose()
                curr_pose = np.array(curr_pose, dtype=self.dtype)
                # pose interpolation
                curr_t = time.now()
                last_waypoint_time = curr_t
                pose_interp = PoseTrajectoryInterpolator(times=[curr_t], poses=[curr_pose])

            elif self.robot_controller == RobotController.JOINT_POS:
                curr_joint_q = arm.get_joint_positions()
                if self.integrated_gripper:
                    curr_gripper_pos = arm.get_gripper_position()
                    curr_joint_q = curr_joint_q.tolist()
                    curr_joint_q.append(curr_gripper_pos)
                    curr_joint_q = np.array(curr_joint_q, dtype=self.dtype)
                if curr_joint_q is None:
                    raise RuntimeError("Failed to get current joint positions from SO-ARM")

                curr_joint_q = np.array(curr_joint_q, dtype=self.dtype)
                joint_command = np.concatenate([curr_joint_q.copy(), np.array([0.0], dtype=self.dtype)])

                if self.use_waypoint_interpolation:
                    curr_t = time.now()
                    last_waypoint_time = curr_t
                    joint_interp = TrajectoryInterpolator(times=[curr_t], values=[curr_joint_q])
                    # joint filtering/smoothing
                    lowpass_filter = LowPassFilter(alpha=self.joints_lowpass_alpha, initial=curr_joint_q)
            else:
                raise ValueError(f"Controller mode {self.robot_controller} not implemented")

            # Main loop
            dt = 1.0 / self.freq
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            while not self.exit_event.is_set():
                # Fetch requests from queue
                try:
                    reqs = self.request_queue.get_all()
                    if isinstance(reqs, dict):
                        reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
                except queue.Empty:
                    reqs = []

                t_now = time.now()

                for r in reqs:
                    req = Request(RequestType(r.pop("type")), r)
                    if req.type == RequestType.MOVEL:
                        target_pose = np.array(req.params.get("target_tcp_pose"), dtype=self.dtype)
                        target_time = float(req.params.get("target_time"))
                        curr_time = t_now + dt
                        if target_time < curr_time:
                            # logger.warning(f"Target time for MOVEL is in the past. time diff {target_time - curr_time}")
                            continue
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=target_pose,
                            time=target_time,
                            max_pos_speed=self.max_pos_speed,
                            max_rot_speed=self.max_rot_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time,
                        )
                        last_waypoint_time = target_time

                    elif req.type == RequestType.MOVEJ:
                        target_joint_q = np.array(req.params.get("target_joint_q"), dtype=self.dtype)
                        target_time = float(req.params.get("target_time"))
                        curr_time = t_now + dt
                        if target_time < curr_time:
                            # logger.warning(f"Target time for MOVEJ is in the past. time diff {target_time - curr_time}")
                            continue
                        joint_command = target_joint_q
                        if self.use_waypoint_interpolation:
                            joint_interp = joint_interp.schedule_waypoint(
                                value=target_joint_q,
                                time=target_time,
                                max_speed=self.max_motor_speed,
                                curr_time=curr_time,
                                last_waypoint_time=last_waypoint_time,
                            )
                            last_waypoint_time = target_time

                    elif req.type == RequestType.SPEEDL:
                        raise NotImplementedError("SPEEDL not yet implemented for SO-ARM")

                    elif req.type == RequestType.SPEEDJ:
                        raise NotImplementedError("SPEEDJ not yet implemented for SO-ARM")

                    elif req.type == RequestType.MOVE_GRIPPER:
                        gripper_pos = float(req.params.get("gripper_position"))
                        gripper_speed = float(req.params.get("speed", 1000.0))
                        arm.move_gripper(gripper_pos, speed=gripper_speed)

                    else:
                        raise ValueError(f"Unknown request type: {req.type}")

                # Send command to robot
                if self.robot_controller == RobotController.TASK_POS:
                    pose_command = pose_interp(t_now)
                    pose_cmd = pose_command.tolist()[:NUM_JOINTS]
                    gripper_cmd = pose_command.tolist()[NUM_JOINTS]
                    arm.moveL(position=pose_cmd, gripper=gripper_cmd, speed=self.joints_init_speed)

                elif self.robot_controller == RobotController.JOINT_POS:
                    if self.use_waypoint_interpolation:
                        joint_command = joint_interp(t_now)
                        joint_command = lowpass_filter(joint_command)

                    cmd_q = joint_command.tolist()[:NUM_JOINTS]
                    gripper_cmd = joint_command.tolist()[NUM_JOINTS]
                    arm.moveJ(positions=cmd_q, gripper=gripper_cmd)
                else:
                    raise ValueError(self.robot_controller)

                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            arm.disconnect()

    def get_state(self, k=None, out=None):
        """Get the most recent robot state or last k states."""
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        """Get all states from the ring buffer."""
        return self.ring_buffer.get_all()

    def moveL(self, target_tcp_pose, target_time):
        """Move end effector to target pose in Cartesian space."""
        target_tcp_pose = np.array(target_tcp_pose, dtype=self.dtype)
        assert target_tcp_pose.shape == (6,)
        assert target_time > time.now()
        req = {
            "type": RequestType.MOVEL.value,
            "target_tcp_pose": target_tcp_pose,
            "target_time": target_time,
        }
        self.request_queue.put(req)

    def moveJ(self, target_joint_q, target_time):
        """Move to target joint positions."""
        target_joint_q = np.array(target_joint_q, dtype=self.dtype)
        if target_time < time.now():
            logger.warning("Target time for MOVEJ is in the past, skipping command")
            return

        req = {
            "type": RequestType.MOVEJ.value,
            "target_joint_q": target_joint_q,
            "target_time": target_time,
        }
        self.request_queue.put(req)

    def speedL(self, target_tcp_twist, target_time):
        """Set end effector velocity in Cartesian space (not yet implemented)."""
        target_tcp_twist = np.array(target_tcp_twist, dtype=self.dtype)
        assert target_tcp_twist.shape == (6,)
        assert target_time > time.now()
        req = {
            "type": RequestType.SPEEDL.value,
            "target_tcp_twist": target_tcp_twist,
            "target_time": target_time,
        }
        self.request_queue.put(req)

    def speedJ(self, target_joint_qd, target_time):
        """Set joint velocities (not yet implemented)."""
        target_joint_qd = np.array(target_joint_qd, dtype=self.dtype)
        assert target_joint_qd.shape == (NUM_JOINTS,)
        assert target_time > time.now()
        req = {
            "type": RequestType.SPEEDJ.value,
            "target_joint_qd": target_joint_qd,
            "target_time": target_time,
        }
        self.request_queue.put(req)

    def move_gripper(self, gripper_position, speed=1000.0):
        """Move gripper to target position (0-1)."""
        gripper_position = float(gripper_position)
        assert 0.0 <= gripper_position <= 1.0
        req = {
            "type": RequestType.MOVE_GRIPPER.value,
            "gripper_position": gripper_position,
            "speed": speed,
        }
        self.request_queue.put(req)


def SoArmServer(mw, *args, **kwargs):
    """Create a server for the SO-ARM robot."""
    return ServerFactory(mw, SoArm, *args, **kwargs)


def SoArmClient(mw, *args, **kwargs):
    """Create a client for the SO-ARM robot."""
    return ClientFactory(mw, SoArm, *args, **kwargs)
