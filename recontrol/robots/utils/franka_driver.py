import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol

import numpy as np
import scipy.spatial.transform as st

try:
    import deoxys  # type: ignore
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        deoxys = None  # type: ignore
try:
    import franky  # type: ignore
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        franky = None  # type: ignore
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
    import polymetis  # type: ignore
    import torch  # type: ignore
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        polymetis = None  # type: ignore
        torch = None  # type: ignore
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

    # deoxys

    # franky
    rel_dyn_factor: tuple[float, float, float] = (0.2, 0.1, 0.1)  # global vel/acc/jerk scaling

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


class FrankaDeoxys:
    pass


class FrankaFranky:
    pass


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
        self.panda.get_robot().set_collision_behavior(
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
        )
        # ctrl is lazy initialized to make it easier to mix different modes
        self.ctrl: panda_py.controllers.TorqueController | None = None
        self.ctx = None

    def _start_ctrl(self, mode: str):
        if mode == "cartesian_impedance":
            # translation, rotation stiffnss
            # Kt, Kr = 210.0, 50.0
            Kt, Kr = 800.0, 40.0
            ctrl = panda_py.controllers.CartesianImpedance(
                impedance=np.diag([Kt, Kt, Kt, Kr, Kr, Kr]),
                damping_ratio=1.0,
                nullspace_stiffness=0.5,
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
    pass


class FrankaPylibfranka:
    pass


def FrankaDriver(driver: str, *args, **kwargs):
    if driver == "deoxys":
        return FrankaDeoxys(*args, **kwargs)
    elif driver == "franky":
        return FrankaFranky(*args, **kwargs)
    elif driver == "panda_py":
        return FrankaPandapy(*args, **kwargs)
    elif driver == "polymetis":
        return FrankaPolymetis(*args, **kwargs)
    elif driver == "pylibfranka":
        return FrankaPylibfranka(*args, **kwargs)
    else:
        raise ValueError(driver)


class FrankaGripperDriver:
    def __init__(self, driver: str, *, robot_ip: str = "192.168.1.111", **kwargs):
        assert driver in ("panda_py",)
        self.robot_ip = robot_ip
        self.cfg = FrankaCfg(**kwargs)

    def start(self):
        self.gripper = panda_py.libfranka.Gripper(self.robot_ip)

        # NOTE: read_once() seem to work real-time, so just read once at start right now
        state = self.gripper.read_once()
        self._gripper_max_width = state.max_width
        self._gripper_width = state.width / self._gripper_max_width

        if self.cfg.home_to_open:
            self.gripper.homing()
            self._gripper_width = self._gripper_max_width

    def stop(self):
        self.gripper.stop()

    def state(self):
        # [0, max_width] -> [0, 1]
        pos = self._gripper_width / self._gripper_max_width
        robot_state = {
            "gripper_position": pos,
        }
        return robot_state

    def moveL(self, target_pos, wait=False):
        # [0, 1] -> [0, max_width]
        pos = target_pos / self._gripper_max_width
        self._gripper_width = pos
        self.gripper.move(pos, 1.0)
        # self.gripper.grasp(pos, 0.1, 50.0)  # pos, speed, force
        if wait:
            time.sleep(2.0)  # 2s should be enough time given max gripper speed/width
