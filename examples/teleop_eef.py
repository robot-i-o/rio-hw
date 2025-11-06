import multiprocessing as mp
from contextlib import nullcontext
from dataclasses import asdict, dataclass, field

import numpy as np
import scipy.spatial.transform as st
import tyro

from ._nodes import StationCfg, make_node


class Interface:
    @staticmethod
    def poll(_teleop, teleop, t_sample, t_last_mode_change, teleop_mode):
        poll_teleop_fn = getattr(Interface, f"poll_{_teleop.lower()}")
        return poll_teleop_fn(teleop, t_sample, t_last_mode_change, teleop_mode)

    @staticmethod
    def poll_gamepad(gp, t_sample, t_last_mode_change, teleop_mode):
        """
        Controls:
        - Left stick: XY translation
        - Right stick: XY rotation
        - LT/RT: Z translation/rotation
        - up_button/B: gripper open/close
        - Y button: change mode
        """
        gp_motion = gp.get_motion_state_transformed()

        gp_x = gp.is_button_pressed(2)
        gp_a = gp.is_button_pressed(0)
        gp_b = gp.is_button_pressed(3)

        delta_arm_pose = gp_motion
        gripper_pose = None
        if gp_x:
            if t_sample - t_last_mode_change > 1.0:  # 1 second delay between mode changes
                teleop_mode = (teleop_mode + 1) % 3  # 3 modes: 0, 1, 2
                t_last_mode_change = t_sample
        elif gp_a:
            gripper_pose = 1.0  # open
        elif gp_b:
            gripper_pose = 0.0  # close
        return delta_arm_pose, gripper_pose, t_last_mode_change, teleop_mode

    @staticmethod
    def poll_keyboard(kb, t_sample, t_last_mode_change, teleop_mode):
        """
        Controls:
        - WASD: XY translation
        - QE: Z translation
        - IJKL: XY rotation
        - UO: Z rotation
        - 1/2/3: mode
        - []: gripper open/close
        """
        alphanumeric_state = kb.get_state()["alphanumeric_state"]
        # special_state = kb.get_state()["special_state"]
        kb_motion = np.zeros((6,), dtype=np.float32)
        pos_gripper = None

        keys = []
        for key in alphanumeric_state:
            if key != 0:
                keys.append(chr(key))

        for key in keys:
            # translation
            if key == "w":
                kb_motion[0] = 1.0
            elif key == "s":
                kb_motion[0] = -1.0
            elif key == "a":
                kb_motion[1] = -1.0
            elif key == "d":
                kb_motion[1] = 1.0
            elif key == "q":
                kb_motion[2] = -1.0
            elif key == "e":
                kb_motion[2] = 1.0

            # rotation
            if key == "i":
                kb_motion[3] = 1.0
            elif key == "k":
                kb_motion[3] = -1.0
            elif key == "j":
                kb_motion[4] = 1.0
            elif key == "l":
                kb_motion[4] = -1.0
            elif key == "u":
                kb_motion[5] = 1.0
            elif key == "o":
                kb_motion[5] = -1.0

            # gripper
            if key == "[":
                pos_gripper = 0.0
            elif key == "]":
                pos_gripper = 1.0

            # teleop mode
            if key == "1":
                teleop_mode = 0
            elif key == "2":
                teleop_mode = 1
            elif key == "3":
                teleop_mode = 2

        delta_tcp_pose = kb_motion
        return delta_tcp_pose, pos_gripper, t_last_mode_change, teleop_mode

    @staticmethod
    def poll_spacemouse(sm, t_sample, t_last_mode_change, teleop_mode):
        sm_motion = sm.get_motion_state_transformed()
        sm_b0 = sm.is_button_pressed(0)
        sm_b1 = sm.is_button_pressed(1)
        delta_tcp_pose = sm_motion
        gripper_pos = None
        if sm_b0 and sm_b1:
            if t_sample - t_last_mode_change > 1.0:  # 1 second delay between mode changes
                teleop_mode = (teleop_mode + 1) % 3  # 3 modes: 0, 1, 2
                t_last_mode_change = t_sample
        elif sm_b0:
            gripper_pos = 0.0  # close
        elif sm_b1:
            gripper_pos = 1.0  # open
        return delta_tcp_pose, gripper_pos, t_last_mode_change, teleop_mode


class Robot:
    @staticmethod
    def move_arm(arm, freq, t_cmd_target, teleop_mode, delta_pose, target_pose, max_pos_speed, max_rot_speed):
        dpos = delta_pose[:3] * (max_pos_speed / freq)
        drot_xyz = delta_pose[3:] * (max_rot_speed / freq)
        if teleop_mode == 0:
            # 2D translation mode
            drot_xyz[:] = 0
            dpos[2] = 0
        elif teleop_mode == 1:
            # translation mode
            drot_xyz[:] = 0
        elif teleop_mode == 2:
            # rotation mode
            dpos[:] = 0
        else:
            raise RuntimeError(teleop_mode)
        drot = st.Rotation.from_euler("xyz", drot_xyz)
        target_pose[:3] += dpos
        target_pose[3:] = (drot * st.Rotation.from_rotvec(target_pose[3:])).as_rotvec()
        arm.moveL(target_pose.tolist(), t_cmd_target)
        return target_pose

    @staticmethod
    def move_gripper(gripper, freq, t_cmd_target, pos, target_pos):
        if pos is not None:
            target_pos = pos
        gripper.moveL([target_pos], t_cmd_target)
        return target_pos


def teleop_eef(args, teleop, arm, gripper, arm2, gripper2):
    from recontrol import time

    arm_target_pose = arm.get_state()["tcp_pose"] if arm else None
    arm2_target_pose = arm2.get_state()["tcp_pose"] if arm2 else None
    gripper_target_pos = gripper.get_state()["gripper_position"] if gripper else None
    gripper2_target_pos = gripper2.get_state()["gripper_position"] if gripper2 else None
    teleop_mode = 0
    t_last_mode_change = time.now()

    input("Press Enter to start")
    try:
        # Main loop
        freq = args.freq
        dt = 1.0 / freq
        command_latency = dt / 2
        t_start = time.now()
        it = 0
        while True:
            t_cycle_end = t_start + (it + 1) * dt
            t_sample = t_cycle_end - command_latency
            t_cmd_target = t_cycle_end + dt

            time.precise_wait(t_sample)
            # get teleop command
            polled = Interface.poll(args.teleop, teleop, t_sample, t_last_mode_change, teleop_mode)
            delta_tcp_pose, gripper_pos, t_last_mode_change, teleop_mode = polled

            # move robots
            if arm:
                _t_cmd_target = t_cmd_target + args.arm_latency
                max_pos_speed, max_rot_speed = args.arm_cfg.max_pos_speed, args.arm_cfg.max_rot_speed
                arm_target_pose = Robot.move_arm(
                    arm, freq, _t_cmd_target, teleop_mode, delta_tcp_pose, arm_target_pose, max_pos_speed, max_rot_speed
                )

            if arm2:
                _t_cmd_target = t_cmd_target + args.arm_latency
                max_pos_speed, max_rot_speed = args.arm2_cfg.max_pos_speed, args.arm2_cfg.max_rot_speed
                arm2_target_pose = Robot.move_arm(
                    arm2, freq, _t_cmd_target, teleop_mode, delta_tcp_pose, arm2_target_pose, max_pos_speed, max_rot_speed
                )

            if gripper:
                _t_cmd_target = t_cmd_target + args.gripper_latency
                gripper_target_pos = Robot.move_gripper(gripper, freq, _t_cmd_target, gripper_pos, gripper_target_pos)

            if gripper2:
                _t_cmd_target = t_cmd_target + args.gripper_latency
                gripper2_target_pos = Robot.move_gripper(gripper2, freq, _t_cmd_target, gripper_pos, gripper2_target_pos)

            # logging
            if it % freq == 0:
                print(
                    f"t: {t_cycle_end - t_start:.3f}s",
                    "|",
                    f"teleop_mode: {teleop_mode}",
                    "|",
                    f"delta_tcp_pose: {delta_tcp_pose}",
                    "|",
                    f"gripper_pos: {gripper_pos}",
                )

            time.precise_wait(t_cycle_end)
            it += 1
    except KeyboardInterrupt:
        pass


def main(args):
    teleop_server, teleop_client = make_node(args.mw, "interfaces", args.teleop, asdict(args.teleop_cfg))
    arm_server, arm_client = make_node(args.mw, "robots", args.arm, asdict(args.arm_cfg))
    if getattr(args, "gripper", None):
        gripper_server, gripper_client = make_node(args.mw, "robots", args.gripper, asdict(args.gripper_cfg))
    else:
        gripper_server, gripper_client = lambda: None, None
    if getattr(args, "arm2", None):
        arm2_server, arm2_client = make_node(args.mw, "robots", args.arm2, asdict(args.arm2_cfg))
    else:
        arm2_server, arm2_client = lambda: None, None
    if getattr(args, "gripper2", None):
        gripper2_server, gripper2_client = make_node(args.mw, "robots", args.gripper2, asdict(args.gripper2_cfg))
    else:
        gripper2_server, gripper2_client = lambda: None, None

    from recontrol.middleware import ServerManager

    with ServerManager(args.mw, [teleop_server, arm_server, gripper_server, arm2_server, gripper2_server]):
        with (
            teleop_client() as teleop,
            arm_client() if arm_client else nullcontext() as arm,
            gripper_client() if gripper_client else nullcontext() as gripper,
            arm2_client() if arm2_client else nullcontext() as arm2,
            gripper2_client() if gripper2_client else nullcontext() as gripper2,
        ):
            teleop_eef(args, teleop, arm, gripper, arm2, gripper2)


@dataclass
class Args(StationCfg):
    @dataclass
    class TeleopCfg:
        addr: str = "127.0.0.1:5000"

    teleop: str = "Spacemouse"  # Gamepad, Keyboard, Spacemouse
    teleop_cfg: TeleopCfg = field(default_factory=lambda: Args.TeleopCfg())

    arm_latency: float = 0.0
    gripper_latency: float = 0.1

    mw: str = "Shm"  # middleware
    mp_method: str | None = "fork"
    freq: int = 30


if __name__ == "__main__":
    args = tyro.cli(Args)
    print(args)
    mp.set_start_method(args.mp_method, force=True)
    main(args)
