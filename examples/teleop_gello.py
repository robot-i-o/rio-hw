import multiprocessing as mp
from contextlib import nullcontext
from dataclasses import asdict, dataclass, field

import numpy as np
import tyro
import yaml

from recontrol import time

from ._nodes import StationCfg, make_node


def move_arm(freq, t_cmd_target, gello_joints, arm, target_joints, max_joint_delta=0.02, deadband=0.0):
    command_joints = gello_joints[: len(target_joints)]
    delta = command_joints - target_joints

    if deadband > 0:
        delta = np.where(np.abs(delta) < deadband, 0, delta)

    delta = np.clip(delta, -max_joint_delta, max_joint_delta)
    target_joints[:] = target_joints + delta
    arm.moveJ(target_joints.tolist(), t_cmd_target)


def move_gripper(t_cmd_target, gello_gripper_pos, gripper, last_time, last_pos, max_delta=0.1, gripper_freq=30):
    dt = 1.0 / gripper_freq
    t_now = time.now()

    if t_now - last_time[0] >= dt:
        delta = np.clip(gello_gripper_pos - last_pos[0], -max_delta, max_delta)
        target_pos = last_pos[0] + delta

        gripper.moveL([target_pos], t_cmd_target)
        last_time[0] = t_now
        last_pos[0] = target_pos


def check_gello_alignment(gello_joints, target_joints, max_joint_delta=0.8):
    joint_delta = np.abs(gello_joints - target_joints)
    max_delta = joint_delta.max()

    # Show detailed joint-by-joint comparison
    print("\nJoint-by-joint alignment:")
    print("Joint | Gello (°) | Robot (°) | Delta (°)")
    print("------|-----------|-----------|----------")
    for i in range(len(gello_joints)):
        gello_deg = np.rad2deg(gello_joints[i])
        robot_deg = np.rad2deg(target_joints[i])
        delta_deg = np.rad2deg(joint_delta[i])
        status = "❌" if joint_delta[i] > max_joint_delta else "✅"
        print(f"  {i + 1}   | {gello_deg:8.1f}  | {robot_deg:8.1f}  | {delta_deg:6.1f} {status}")
    if max_delta > max_joint_delta:
        max_joint_idx = np.argmax(joint_delta)
        print(f"Joint {max_joint_idx + 1} has the largest delta: {np.rad2deg(max_delta):.1f}°")
        raise RuntimeError("Align Gello to match robot initial pose. ")
    print(f"\n✓ Alignment OK (max delta: {np.rad2deg(max_delta):.1f}°)")


def teleop_gello(args, teleop, teleop2, arm, gripper, arm2, gripper2):
    gello = teleop
    gello2 = teleop2
    target_jointq = arm.get_state()["ActualQ"] if arm else None
    target2_jointq = arm2.get_state()["ActualQ"] if arm2 else None

    print("Checking Gello alignment...")
    gello_jointq = gello.get_state()["joint_positions"]
    check_gello_alignment(gello_jointq, target_jointq)
    if arm2:
        gello2_jointq = gello2.get_state()["joint_positions"] if gello2 else gello_jointq
        check_gello_alignment(gello2_jointq, target2_jointq)

    input("Press Enter to start")
    try:
        # Main loop
        freq = args.freq
        dt = 1.0 / freq
        command_latency = dt / 2
        t_start = time.now()

        last_gripper_time = [0.0]
        last_gripper2_time = [0.0]
        last_gripper_pos = [0.0]
        last_gripper2_pos = [0.0]

        it = 0
        while True:
            t_cycle_end = t_start + (it + 1) * dt
            t_sample = t_cycle_end - command_latency
            t_cmd_target = t_cycle_end + dt

            time.precise_wait(t_sample)
            # get teleop command
            gello_state = gello.get_state()
            gello_jointq = gello_state["joint_positions"]
            gello_gripper_pos = gello_state["gripper_position"]
            if gello2:
                gello2_state = gello2.get_state()
                gello2_jointq = gello2_state["joint_positions"]
                gello2_gripper_pos = gello2_state["gripper_position"]

            if arm:
                move_arm(freq, t_cmd_target, gello_jointq, arm, target_jointq)

            if arm2:
                _gello_jointq = gello2_jointq if gello2 else gello_jointq
                move_arm(freq, t_cmd_target, _gello_jointq, arm2, target2_jointq)

            if gripper:
                move_gripper(time.now() + 0.1, gello_gripper_pos, gripper, last_gripper_time, last_gripper_pos)

            if gripper2:
                _gello_gripper_pos = gello2_gripper_pos if gello2 else gello_gripper_pos
                move_gripper(time.now() + 0.1, _gello_gripper_pos, gripper2, last_gripper2_time, last_gripper2_pos)

            # logging
            if it % freq == 0:
                print(
                    f"t: {t_cycle_end - t_start:.3f}s",
                    "|",
                    f"gello_jointq: {gello_jointq}",
                    "|",
                    f"gripper: {gello_gripper_pos:.2f}",
                )

            time.precise_wait(t_cycle_end)
            it += 1
    except KeyboardInterrupt:
        pass


def main(args):
    teleop_server, teleop_client = make_node(args.mw, "interfaces", args.teleop, asdict(args.teleop_cfg))
    teleop2_server, teleop2_client = make_node(args.mw, "interfaces", args.teleop2, asdict(args.teleop2_cfg))

    arm_cfg_dict = asdict(args.arm_cfg)
    arm_cfg_dict["robot_controller"] = "joint_pos"
    arm_cfg_dict["joints_init"] = args.teleop_cfg.start_joints[:-1]  # Exclude gripper position
    arm_server, arm_client = make_node(args.mw, "robots", args.arm, arm_cfg_dict)

    gripper_server, gripper_client = make_node(args.mw, "robots", args.gripper, asdict(args.gripper_cfg))

    if getattr(args, "arm2", None):
        arm2_cfg_dict = asdict(args.arm2_cfg)
        arm2_cfg_dict["robot_controller"] = "joint_pos"
        # Exclude gripper position
        start_joints = args.teleop2_cfg.start_joints if args.teleop2 else args.teleop_cfg.start_joints
        arm2_cfg_dict["joints_init"] = start_joints[:-1]
        arm2_server, arm2_client = make_node(args.mw, "robots", args.arm2, arm2_cfg_dict)
    else:
        arm2_server, arm2_client = lambda: None, None
    if getattr(args, "gripper2", None):
        gripper2_server, gripper2_client = make_node(args.mw, "robots", args.gripper2, asdict(args.gripper2_cfg))
    else:
        gripper2_server, gripper2_client = lambda: None, None

    from recontrol.middleware import ServerManager

    with ServerManager(args.mw, [teleop_server, teleop2_server, arm_server, gripper_server, arm2_server, gripper2_server]):
        with (
            teleop_client() as teleop,
            teleop2_client() if teleop2_client else nullcontext() as teleop2,
            arm_client() if arm_client else nullcontext() as arm,
            gripper_client() if gripper_client else nullcontext() as gripper,
            arm2_client() if arm2_client else nullcontext() as arm2,
            gripper2_client() if gripper2_client else nullcontext() as gripper2,
        ):
            teleop_gello(args, teleop, teleop2, arm, gripper, arm2, gripper2)


@dataclass
class GelloCfg:
    port: str = "/dev/ttyUSB0"
    baudrate: int = 57600
    joint_ids: tuple = (1, 2, 3, 4, 5, 6, 7, 8)
    joint_offsets: tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_signs: tuple = (1, 1, 1, 1, 1, 1, 1)
    gripper_config: tuple = (0, 0, 0)
    start_joints: tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass
class Args(StationCfg):
    teleop: str = "Gello"
    teleop_cfg: GelloCfg = field(default_factory=lambda: GelloCfg())
    teleop_cfg_yaml: str | None = None

    teleop2: str | None = None
    teleop2_cfg: GelloCfg = field(default_factory=lambda: GelloCfg())
    teleop2_cfg_yaml: str | None = None

    mw: str = "Shm"  # middleware
    mp_method: str | None = "fork"
    freq: int = 200

    def __post_init__(self):
        def load_gello_yaml(yaml_path):
            with open(yaml_path) as f:
                teleop_cfg = yaml.safe_load(f)
            teleop_kwargs = {
                "port": teleop_cfg["agent"]["port"],
                "joint_ids": tuple(teleop_cfg["agent"]["dynamixel_config"]["joint_ids"]),
                "joint_offsets": tuple(teleop_cfg["agent"]["dynamixel_config"]["joint_offsets"]),
                "joint_signs": tuple(teleop_cfg["agent"]["dynamixel_config"]["joint_signs"]),
                "gripper_config": tuple(teleop_cfg["agent"]["dynamixel_config"]["gripper_config"]),
                "start_joints": tuple(teleop_cfg["agent"]["start_joints"]),
            }
            return GelloCfg(**teleop_kwargs)

        if self.teleop_cfg_yaml:
            self.teleop_cfg = load_gello_yaml(self.teleop_cfg_yaml)

        if self.teleop2_cfg_yaml:
            self.teleop2_cfg = load_gello_yaml(self.teleop2_cfg_yaml)


if __name__ == "__main__":
    args = tyro.cli(Args)
    print(args)
    mp.set_start_method(args.mp_method)
    main(args)
