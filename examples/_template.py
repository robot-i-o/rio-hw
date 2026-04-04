import multiprocessing as mp
from contextlib import nullcontext
from dataclasses import dataclass

import tyro

from ._real_env import RealEnv, StationCfg


def template(args, arm, gripper, arm2, gripper2):
    from rio_hw import time

    input("Press Enter to start")
    try:
        # Main loop
        freq = args.freq
        dt = 1.0 / freq
        command_latency = dt / 2 if args.command_latency is None else args.command_latency
        t_start = time.now()
        it = 0
        while True:
            t_cycle_end = t_start + (it + 1) * dt
            t_sample = t_cycle_end - command_latency
            t_cmd_target = t_cycle_end + dt

            time.precise_wait(t_sample)
            # get command
            ...

            # move robots
            if arm:
                _t_cmd_target = t_cmd_target + args.arm_latency
            if gripper:
                _t_cmd_target = t_cmd_target + args.gripper_latency
            if arm2:
                _t_cmd_target = t_cmd_target + args.arm_latency
            if gripper2:
                _t_cmd_target = t_cmd_target + args.gripper_latency

            # get state
            ...

            # logging
            if it % freq == 0:
                print(
                    f"t: {t_cycle_end - t_start:.3f}s",
                    "|",
                )

            time.precise_wait(t_cycle_end)
            it += 1
    except KeyboardInterrupt:
        pass


def main(args):
    servers, clients = RealEnv.make_nodes(args)

    from rio_hw.middleware import ServerManager

    with ServerManager(args.mw, list(servers.values())):
        with (
            clients["arm"]() if clients["arm"] else nullcontext() as arm,
            clients["gripper"]() if clients["gripper"] else nullcontext() as gripper,
            clients["arm2"]() if clients["arm2"] else nullcontext() as arm2,
            clients["gripper2"]() if clients["gripper2"] else nullcontext() as gripper2,
        ):
            template(args, arm, gripper, arm2, gripper2)


@dataclass
class Args(StationCfg):
    command_latency: float | None = 0.01
    arm_latency: float = 0.0
    gripper_latency: float = 0.0

    mw: str = "Shm"  # middleware
    mp_method: str | None = "fork"
    freq: int = 50


if __name__ == "__main__":
    args = tyro.cli(Args)
    print(args)
    mp.set_start_method(args.mp_method, force=True)
    main(args)
