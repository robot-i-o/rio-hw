import multiprocessing as mp
from contextlib import nullcontext
from dataclasses import asdict, dataclass, field

import scipy.spatial.transform as st
import tyro

from ._nodes import StationCfg, make_node


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


def move_gripper(gripper, freq, t_cmd_target, teleop_mode, target_pose):
    if target_pose is not None:
        gripper.moveL([target_pose], t_cmd_target)


def poll_spacemouse(sm, t_sample, t_last_mode_change, teleop_mode):
    sm_motion = sm.get_motion_state_transformed()
    sm_b0 = sm.is_button_pressed(0)
    sm_b1 = sm.is_button_pressed(1)
    if sm_b0 and sm_b1:
        if t_sample - t_last_mode_change > 1.0:  # 1 second delay between mode changes
            teleop_mode = (teleop_mode + 1) % 3  # 3 modes: 0, 1, 2
            t_last_mode_change = t_sample
    delta_arm_pose = sm_motion
    if sm_b0:
        gripper_pose = 0.0  # close
    elif sm_b1:
        gripper_pose = 1.0  # open
    else:
        gripper_pose = None
    return delta_arm_pose, gripper_pose, t_last_mode_change, teleop_mode


def teleop_eef(args, teleop, arm, gripper, arm2, gripper2):
    from recontrol import time

    target_pose = arm.get_state()["TargetTCPPose"] if arm else None
    target_pose2 = arm2.get_state()["TargetTCPPose"] if arm2 else None
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
            if args.teleop == "Spacemouse":
                polled = poll_spacemouse(teleop, t_sample, t_last_mode_change, teleop_mode)
            else:
                raise RuntimeError(args.teleop)
            delta_arm_pose, gripper_pose, t_last_mode_change, teleop_mode = polled

            if arm:
                max_pos_speed, max_rot_speed = args.arm_cfg.max_pos_speed, args.arm_cfg.max_rot_speed
                move_arm(arm, freq, t_cmd_target, teleop_mode, delta_arm_pose, target_pose, max_pos_speed, max_rot_speed)

            if arm2:
                max_pos_speed, max_rot_speed = args.arm2_cfg.max_pos_speed, args.arm2_cfg.max_rot_speed
                move_arm(arm, freq, t_cmd_target, teleop_mode, delta_arm_pose, target_pose2, max_pos_speed, max_rot_speed)

            if gripper:
                move_gripper(gripper, freq, t_cmd_target, teleop_mode, gripper_pose)

            if gripper2:
                move_gripper(gripper2, freq, t_cmd_target, teleop_mode, gripper_pose)

            # logging
            if it % freq == 0:
                print(
                    f"t: {t_cycle_end - t_start:.3f}s",
                    "|",
                    f"teleop_mode: {teleop_mode}",
                    "|",
                    f"delta_arm_pose: {delta_arm_pose}",
                    "|",
                    f"gripper_pose: {gripper_pose}",
                )

            time.precise_wait(t_cycle_end)
            it += 1
    except KeyboardInterrupt:
        pass


def main(args):
    teleop_server, teleop_client = make_node(args.mw, "interfaces", args.teleop, asdict(args.teleop_cfg))
    arm_server, arm_client = make_node(args.mw, "robots", args.arm, asdict(args.arm_cfg))
    gripper_server, gripper_client = make_node(args.mw, "robots", args.gripper, asdict(args.gripper_cfg))
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

    teleop: str = "Spacemouse"  # Gamepad, Iphone, Keyboard, Spacemouse
    teleop_cfg: TeleopCfg = field(default_factory=lambda: Args.TeleopCfg())

    mw: str = "Shm"  # middleware
    mp_method: str | None = "fork"
    freq: int = 30


if __name__ == "__main__":
    args = tyro.cli(Args)
    print(args)
    mp.set_start_method(args.mp_method, force=True)
    main(args)
