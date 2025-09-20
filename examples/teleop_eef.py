from contextlib import nullcontext
from dataclasses import asdict, dataclass

import scipy.spatial.transform as st
import tyro

from .nodes import StationCfg, make_node


def move_arm(freq, t_cmd_target, sm_motion, teleop_mode, arm, max_pos_speed, max_rot_speed, target_pose):
    dpos = sm_motion[:3] * (max_pos_speed / freq)
    drot_xyz = sm_motion[3:] * (max_rot_speed / freq)

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


def move_gripper(freq, t_cmd_target, sm_b0, sm_b1, gripper):
    if sm_b0:
        gripper.moveL([0.0], t_cmd_target)  # close
    elif sm_b1:
        gripper.moveL([1.0], t_cmd_target)  # open


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

    from recontrol import time
    from recontrol.middleware import ServerManager

    with ServerManager(args.mw, [teleop_server, arm_server, gripper_server, arm2_server, gripper2_server]):
        with (
            teleop_client() as teleop,
            arm_client() if arm_client else nullcontext() as arm,
            gripper_client() if gripper_client else nullcontext() as gripper,
            arm2_client() if arm2_client else nullcontext() as arm2,
            gripper2_client() if gripper2_client else nullcontext() as gripper2,
        ):
            freq = args.freq
            sm = teleop
            target_pose = arm.get_state()["TargetTCPPose"] if arm else None
            target_pose2 = arm2.get_state()["TargetTCPPose"] if arm2 else None
            teleop_mode = 0
            last_mode_change = time.now()

            input("Press Enter to start")
            try:
                # Main loop
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
                    sm_motion = sm.get_motion_state_transformed()
                    sm_b0 = sm.is_button_pressed(0)
                    sm_b1 = sm.is_button_pressed(1)
                    if sm_b0 and sm_b1:
                        t_mode = time.now()
                        if t_mode - last_mode_change > 1.0:  # 1 second delay between mode changes
                            teleop_mode = (teleop_mode + 1) % 3  # 3 modes: 0, 1, 2
                            last_mode_change = t_mode

                    if arm:
                        max_pos_speed, max_rot_speed = args.arm_cfg.max_pos_speed, args.arm_cfg.max_rot_speed
                        move_arm(freq, t_cmd_target, sm_motion, teleop_mode, arm, max_pos_speed, max_rot_speed, target_pose)

                    if arm2:
                        max_pos_speed, max_rot_speed = args.arm2_cfg.max_pos_speed, args.arm2_cfg.max_rot_speed
                        move_arm(freq, t_cmd_target, sm_motion, teleop_mode, arm2, max_pos_speed, max_rot_speed, target_pose2)

                    if gripper:
                        move_gripper(freq, t_cmd_target, sm_b0, sm_b1, gripper)

                    if gripper2:
                        move_gripper(freq, t_cmd_target, sm_b0, sm_b1, gripper2)

                    # logging
                    if it % freq == 0:
                        print(
                            f"t: {t_cycle_end - t_start:.3f}s",
                            "|",
                            f"teleop_mode: {teleop_mode}",
                            "|",
                            f"sm_motion: {sm_motion}",
                        )

                    time.precise_wait(t_cycle_end)
                    it += 1
            except KeyboardInterrupt:
                pass


@dataclass
class Args(StationCfg):
    teleop: str = "Spacemouse"  # Gamepad, Iphone, Keyboard, Spacemouse
    mw: str = "Shm"  # middleware
    freq: int = 30


if __name__ == "__main__":
    args = tyro.cli(Args)
    print(args)
    main(args)
