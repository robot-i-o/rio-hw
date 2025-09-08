from contextlib import nullcontext
from dataclasses import asdict, dataclass, field

import scipy.spatial.transform as st
import tyro


def main(args):
    if args.teleop == "Spacemouse":
        from recontrol.interfaces.spacemouse import SpacemouseClient, SpacemouseServer

        teleop_kwargs = {}
        teleop_server = lambda: SpacemouseServer(args.mw, **teleop_kwargs)
        teleop_client = SpacemouseClient(args.mw, **teleop_kwargs)
    else:
        raise ValueError

    if args.arm == "XArm":
        from recontrol.robots.xarm import XArmClient, XArmServer

        arm_kwargs = asdict(args.arm_cfg)
        arm_server = lambda: XArmServer(args.mw, **arm_kwargs)
        arm_client = XArmClient(args.mw, **arm_kwargs)
    else:
        arm_server = lambda: None
        arm_client = None

    from recontrol import time
    from recontrol.middleware import ServerManager

    with ServerManager(args.mw, [teleop_server, arm_server]):
        with teleop_client as teleop, arm_client if arm_client else nullcontext() as arm:
            freq = args.freq
            sm = teleop
            target_pose = arm.get_state()["TargetTCPPose"] if arm else None

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
                    t_command_target = t_cycle_end + dt

                    time.precise_wait(t_sample)
                    # get teleop command
                    sm_state = sm.get_motion_state_transformed()
                    print(sm_state)
                    if arm:
                        max_pos_speed, max_rot_speed = args.arm_cfg.max_pos_speed, args.arm_cfg.max_rot_speed
                        dpos = sm_state[:3] * (max_pos_speed / freq)
                        drot_xyz = sm_state[3:] * (max_rot_speed / freq)

                        if not sm.is_button_pressed(0):
                            # translation mode
                            drot_xyz[:] = 0
                        else:
                            dpos[:] = 0
                        if not sm.is_button_pressed(1):
                            # 2D translation mode
                            dpos[2] = 0

                        drot = st.Rotation.from_euler("xyz", drot_xyz)
                        target_pose[:3] += dpos
                        target_pose[3:] = (drot * st.Rotation.from_rotvec(target_pose[3:])).as_rotvec()
                        arm.schedule_waypoint(target_pose.tolist(), t_command_target)
                    time.precise_wait(t_cycle_end)
                    it += 1
            except KeyboardInterrupt:
                pass


class Cfg:
    @dataclass
    class TeleopCfg:
        addr: str = "127.0.0.1:5557"

    @dataclass
    class ArmCfg:
        robot_ip: str = "192.168.1.228"
        addr: str = "127.0.0.1:5559"
        max_pos_speed = 0.25
        max_rot_speed = 0.6


@dataclass
class Args:
    freq: int = 30
    mw: str = "Shm"  # middleware
    teleop: str = "Spacemouse"
    teleop_cfg: Cfg.TeleopCfg = field(default_factory=lambda: Cfg.TeleopCfg())
    arm: str | None = None
    arm_cfg: Cfg.ArmCfg = field(default_factory=lambda: Cfg.ArmCfg())


if __name__ == "__main__":
    args = tyro.cli(Args)
    print(args)
    main(args)
