import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol

import numpy as np
import scipy.spatial.transform as st

try:
    import panda_py  # type: ignore
    import panda_py.constants  # type: ignore
    import panda_py.controllers  # type: ignore
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        panda_py = None  # type: ignore

try:
    import zerorpc  # type: ignore
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        zerorpc = None  # type: ignore

try:
    import pylibfranka  # type: ignore
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        pylibfranka = None  # type: ignore


@dataclass
class FrankaCfg:
    # gripper
    home_to_open: bool = True

    # pandapy
    robot_model: str = "fr3"
    username: str = "admin"
    password: str = "password1"

    # polymetis
    robot_port: int = 50051

    # pylibfranka


class Franka(Protocol):
    robot_ip: str
    cfg: FrankaCfg

    def __init__(
        self,
        robot_ip: str = "192.168.1.111",
        **kwargs,
    ):
        self.robot_ip = robot_ip
        self.cfg = FrankaCfg(**kwargs)

    def start(self): ...

    def stop(self): ...

    def state(self): ...

    def moveL(self, tcp_pose, wait=False): ...

    def moveJ(self, joint_q, wait=False): ...

    def impedanceL(self, tcp_pose, wait=False): ...

    def impedanceJ(self, joint_q, wait=False): ...


class FrankaPandapy:
    def __init__(
        self,
        robot_ip: str = "192.168.1.111",
        **kwargs,
    ):
        self.robot_ip = robot_ip
        self.cfg = FrankaCfg(**kwargs)

    def start(self):
        # desk = panda_py.Desk(self.robot_ip, self.cfg.username, self.cfg.password, platform=self.cfg.robot_model)
        # # desk.take_control(force=True)
        # desk.unlock()
        # desk.activate_fci()

        self.panda = panda_py.Panda(self.robot_ip, cutoff_frequency=500.0)
        self.panda.set_default_behavior()
        self.panda.get_robot().set_collision_behavior([100.0] * 7, [100.0] * 7, [100.0] * 6, [100.0] * 6)
        # ctrl is lazy initialized to make it easier to mix different modes
        self.ctrl: panda_py.controllers.TorqueController | None = None
        self.ctx = None

    def _start_ctrl(self, mode: str):
        if mode == "cartesian_impedance":
            # translation, rotation stiffness
            Kt, Kr = 600.0, 30.0
            ctrl = panda_py.controllers.CartesianImpedance(
                impedance=np.diag([Kt, Kt, Kt, Kr, Kr, Kr]),
                damping_ratio=1.0,
                nullspace_stiffness=1.0,
                filter_coeff=1.0,
            )
        elif mode == "joint_position":
            stiffness = [40, 30, 50, 50, 35, 25, 10]
            damping = [4, 6, 5, 5, 3, 2, 1]
            ctrl = panda_py.controllers.JointPosition(stiffness=stiffness, damping=damping, filter_coeff=1.0)
        elif mode == "integrated_velocity":
            ctrl = panda_py.controllers.IntegratedVelocity()
        else:
            raise ValueError(mode)
        self.ctrl = ctrl
        self.panda.start_controller(ctrl)
        self.ctx = self.panda.create_context(frequency=1000.0)
        self.ctx.__enter__()

    def stop(self):
        if self.ctx is not None:
            self.ctx.__exit__(None, None, None)
            self.panda.stop_controller()

    def state(self):
        if self.ctx is not None:
            ok = self.ctx.ok()  # throttles PandaContext loop and raises any control exceptions by libfranka
            if not ok:
                raise RuntimeError

        state = self.panda.get_state()
        self._state = state

        O_T_EE = np.array(state.O_T_EE.copy()).reshape(4, 4)
        O_T_EE_d = np.array(state.O_T_EE_d.copy()).reshape(4, 4)
        if getattr(state, "O_dP_EE", None) is not None:
            O_dP_EE = np.array(state.O_dP_EE.copy())
        else:
            O_dP_EE = np.zeros(6)
        O_dP_EE_d = np.array(state.O_dP_EE_d.copy())

        actual_tcp_p = O_T_EE[3, :3]
        actual_tcp_aa = st.Rotation.from_matrix(O_T_EE[:3, :3]).as_rotvec()
        target_tcp_p = O_T_EE_d[3, :3]
        target_tcp_aa = st.Rotation.from_matrix(O_T_EE_d[:3, :3]).as_rotvec()
        actual_tcp_pose = np.concatenate([actual_tcp_p, actual_tcp_aa])
        target_tcp_pose = np.concatenate([target_tcp_p, target_tcp_aa])

        actual_tcp_speed = O_dP_EE
        target_tcp_speed = O_dP_EE_d

        state = {
            "tcp_pose": actual_tcp_pose,
            "tcp_speed": actual_tcp_speed,
            "joint_q": np.array(state.q.copy()),
            "joint_qd": np.array(state.dq.copy()),
            "target_tcp_pose": target_tcp_pose,
            "target_tcp_speed": target_tcp_speed,
            "target_joint_q": np.array(state.q_d.copy()),
            "target_joint_qd": np.array(state.dq_d.copy()),
        }
        return state

    def moveL(self, tcp_pose, wait=False):
        return self.impedanceL(tcp_pose, wait=wait)

    def moveJ(self, joint_q, wait=False):
        if isinstance(joint_q, np.ndarray):
            joint_q = joint_q.tolist()
        if wait:
            self.panda.move_to_joint_position(joint_q)
        else:
            if self.ctrl is None:
                self._start_ctrl("joint_position")
            self.ctrl.set_control(joint_q)

    def impedanceL(self, tcp_pose, wait=False):
        if isinstance(tcp_pose, np.ndarray):
            tcp_pose = tcp_pose.tolist()
        p = tcp_pose[:3]
        q = st.Rotation.from_rotvec(tcp_pose[3:]).as_quat(scalar_first=False).tolist()
        if wait:
            self.panda.move_to_pose(p, q)
        else:
            if self.ctrl is None:
                self._start_ctrl("cartesian_impedance")
            self.ctrl.set_control(p, q)

    def impedanceJ(self, joint_q, wait=False):
        raise NotImplementedError


class FrankaPolymetis:
    def __init__(
        self,
        robot_ip: str = "192.168.1.111",
        robot_port: int = 50051,
        **kwargs,
    ):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.cfg = FrankaCfg(**kwargs)
        self.server_port = 4242
        self.client = None
        self._mode = None
        self.Kx = [750.0, 750.0, 750.0, 15.0, 15.0, 15.0]  # Stiffness
        self.Kxd = [37.0, 37.0, 37.0, 2.0, 2.0, 2.0]  # Damping
        self.Kq = [20.0, 30.0, 25.0, 25.0, 15.0, 10.0, 10.0]  # Joint stiffness
        self.Kqd = [1.0, 1.5, 1.0, 1.0, 0.5, 0.5, 0.5]  # Joint damping

    def start(self):
        if zerorpc is None:
            raise ImportError("zerorpc is not installed.")
        self.client = zerorpc.Client(heartbeat=20, timeout=300)
        self.client.connect(f"tcp://{self.robot_ip}:{self.server_port}")

    def stop(self):
        if self.client is not None:
            if self._mode is not None:
                self.client.terminate_current_policy()
            self.client.close()

    def state(self):
        ee_pose = np.array(self.client.get_ee_pose())  # [x, y, z, rx, ry, rz]
        joint_pos = np.array(self.client.get_joint_positions())
        joint_vel = np.array(self.client.get_joint_velocities())

        state = {
            "tcp_pose": ee_pose,
            "tcp_speed": np.zeros(6),  # Not available from server
            "joint_q": joint_pos,
            "joint_qd": joint_vel,
            "target_tcp_pose": ee_pose.copy(),  # Use current as target
            "target_tcp_speed": np.zeros(6),
            "target_joint_q": joint_pos.copy(),
            "target_joint_qd": joint_vel.copy(),
        }
        return state

    def moveL(self, pose, wait=False):
        if isinstance(pose, np.ndarray):
            pose = pose.tolist()

        if self._mode != "cartesian":
            self.client.start_cartesian_impedance(self.Kx, self.Kxd)
            self._mode = "cartesian"

        self.client.update_desired_ee_pose(pose)

    def moveJ(self, jointq, wait=False):
        if isinstance(jointq, np.ndarray):
            jointq = jointq.tolist()

        if self._mode != "joint":
            if self._mode is not None:
                self.client.terminate_current_policy()
            self.client.start_joint_impedance(self.Kq, self.Kqd)
            self._mode = "joint"

        self.client.update_desired_joint_positions(jointq)


class FrankaPylibfranka:
    pass


def FrankaDriver(driver: str, *args, **kwargs):
    if driver == "panda_py":
        return FrankaPandapy(*args, **kwargs)
    if driver == "polymetis":
        return FrankaPolymetis(*args, **kwargs)
    if driver == "pylibfranka":
        return FrankaPylibfranka(*args, **kwargs)
    raise ValueError(driver)


class FrankaGripperDriver:
    def __init__(self, driver: str, *, robot_ip: str = "192.168.1.111", **kwargs):
        assert driver in ("panda_py", "polymetis")
        self.driver = driver
        self.robot_ip = robot_ip
        self.cfg = FrankaCfg(**kwargs)
        self.server_port = 4243

    def start(self):
        if self.driver == "panda_py":
            self.gripper = panda_py.libfranka.Gripper(self.robot_ip)
            # NOTE: read_once() does not seem to work real-time, so just read once at start right now
            state = self.gripper.read_once()
            self._gripper_max_width = state.max_width
            self._gripper_width = state.width
            if self.cfg.home_to_open:
                self.gripper.homing()
                self._gripper_width = self._gripper_max_width

        elif self.driver == "polymetis":
            self.gripper = zerorpc.Client(heartbeat=20, timeout=300)
            self.gripper.connect(f"tcp://{self.robot_ip}:{self.server_port}")
            # Get initial state
            self._gripper_max_width = 0.08
            self._gripper_width = self.gripper.get_gripper_width()
            if self.cfg.home_to_open:
                self.gripper.open_gripper()
                self._gripper_width = self._gripper_max_width
        else:
            raise ValueError(self.driver)

    def stop(self):
        if self.driver == "panda_py":
            self.gripper.stop()
        elif self.driver == "polymetis":
            self.gripper.close()
        else:
            raise ValueError(self.driver)

    def state(self):
        # [0, max_width] -> [0, 1]
        pos = self._gripper_width / self._gripper_max_width
        robot_state = {
            "gripper_position": pos,
        }
        return robot_state

    def moveG(self, target_pos, wait=False):
        if self.driver == "panda_py":
            # [0, 1] -> [0, max_width]
            pos = target_pos * self._gripper_max_width
            self._gripper_width = pos
            self.gripper.move(pos, 1.0)
        elif self.driver == "polymetis":
            width = target_pos * self._gripper_max_width
            self._gripper_width = width
            self.gripper.goto_gripper(width, 0.1, 20.0)
        else:
            raise ValueError(self.driver)
        if wait:
            time.sleep(2.0)  # 2s should be enough time given max gripper speed/width
