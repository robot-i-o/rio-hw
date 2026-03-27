import multiprocessing as mp
from contextlib import nullcontext
from dataclasses import asdict, dataclass, field

import numpy as np
import tyro
import yaml

from rio_hw import time

from ._real_env import RealEnv, StationCfg


class Robot:
    @staticmethod
    def move_arm(arm, freq, t_cmd_target, gello_joints, target_joints, max_joint_delta=0.02, deadband=0.0):
        command_joints = gello_joints[: len(target_joints)]
        delta = command_joints - target_joints

        if deadband > 0:
            delta = np.where(np.abs(delta) < deadband, 0, delta)

        delta = np.clip(delta, -max_joint_delta, max_joint_delta)
        target_joints[:] = target_joints + delta
        arm.moveJ(target_joints.tolist(), t_cmd_target)
        return target_joints

    @staticmethod
    def move_gripper(gripper, freq, t_cmd_target, pos, target_pos):
        if pos is not None:
            target_pos = pos
        gripper.moveG([target_pos], t_cmd_target)
        return target_pos


def check_alignment(lead_joints, target_joints, max_joint_delta=0.8):
    joint_delta = np.abs(lead_joints - target_joints)
    max_delta = joint_delta.max()

    # Show detailed joint-by-joint comparison
    print("\nJoint-by-joint alignment:")
    print("Joint | Leader (°) | Follower (°) | Delta (°)")
    print("------|-----------|-----------|----------")
    for i in range(len(lead_joints)):
        lead_deg = np.rad2deg(lead_joints[i])
        robot_deg = np.rad2deg(target_joints[i])
        delta_deg = np.rad2deg(joint_delta[i])
        status = "❌" if joint_delta[i] > max_joint_delta else "✅"
        print(f"  {i + 1}   | {lead_deg:8.1f}  | {robot_deg:8.1f}  | {delta_deg:6.1f} {status}")
    if max_delta > max_joint_delta:
        max_joint_idx = np.argmax(joint_delta)
        print(f"Joint {max_joint_idx + 1} has the largest delta: {np.rad2deg(max_delta):.1f}°")
        raise RuntimeError("Align leader to match follower robot initial pose")
    print(f"\n✓ Alignment OK (max delta: {np.rad2deg(max_delta):.1f}°)")


def teleop_leadfollow(args, teleop, teleop2, arm_lead, gripper_lead, arm2_lead, gripper2_lead, arm, gripper, arm2, gripper2):
    gello = teleop
    gello2 = teleop2

    arm_target_joint_q = arm.get_state()["joint_q"] if arm else None
    arm2_target_joint_q = arm2.get_state()["joint_q"] if arm2 else None
    gripper_target_pos = gripper.get_state()["gripper_position"] if gripper else None
    gripper2_target_pos = gripper2.get_state()["gripper_position"] if gripper2 else None

    print("Checking leader-follower alignment...")
    if arm:
        if gello:
            lead_joint_q = gello.get_state()["joint_q"]
        if arm_lead:
            lead_joint_q = arm_lead.get_state()["joint_q"]
        check_alignment(lead_joint_q, arm_target_joint_q)
    if arm2:
        if gello2:
            lead2_joint_q = gello2.get_state()["joint_q"]
        if arm2_lead:
            lead2_joint_q = arm2_lead.get_state()["joint_q"]
        check_alignment(lead2_joint_q, arm2_target_joint_q)

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
            if gello:
                lead_state = gello.get_state()
                lead_joint_q = lead_state["joint_q"]
                lead_gripper_pos = lead_state["gripper_position"]
            if arm_lead:
                lead_joint_q = arm_lead.get_state()["joint_q"]
            if gripper_lead:
                lead_gripper_pos = gripper_lead.get_state()["gripper_position"]

            if gello2:
                lead2_state = gello2.get_state()
                lead2_joint_q = lead2_state["joint_q"]
                lead2_gripper_pos = lead2_state["gripper_position"]
            if arm2_lead:
                lead2_joint_q = arm2_lead.get_state()["joint_q"]
            if gripper2_lead:
                lead2_gripper_pos = gripper2_lead.get_state()["gripper_position"]

            # move robots
            if arm:
                _t_cmd_target = t_cmd_target + args.arm_latency
                arm_target_joint_q = Robot.move_arm(arm, freq, _t_cmd_target, lead_joint_q, arm_target_joint_q)

            if arm2:
                _t_cmd_target = t_cmd_target + args.arm_latency
                arm2_target_joint_q = Robot.move_arm(arm2, freq, _t_cmd_target, lead2_joint_q, arm2_target_joint_q)

            if gripper:
                _t_cmd_target = t_cmd_target + args.gripper_latency
                gripper_target_pos = Robot.move_gripper(gripper, freq, _t_cmd_target, lead_gripper_pos, gripper_target_pos)

            if gripper2:
                _t_cmd_target = t_cmd_target + args.gripper_latency
                gripper2_target_pos = Robot.move_gripper(gripper2, freq, _t_cmd_target, lead2_gripper_pos, gripper2_target_pos)

            # logging
            if it % freq == 0:
                print(
                    f"t: {t_cycle_end - t_start:.3f}s",
                    "|",
                    f"lead_joint_q: {lead_joint_q}",
                    "|",
                    f"lead_gripper_pos: {lead_gripper_pos:.2f}",
                    "|",
                    f"lead2_joint_q: {lead2_joint_q}",
                    "|",
                    f"lead2_gripper_pos: {lead2_gripper_pos:.2f}",
                )

            time.precise_wait(t_cycle_end)
            it += 1
    except KeyboardInterrupt:
        pass


def main(args):
    kwargs = {}
    if hasattr(args, "arm"):
        arm_cfg = asdict(args.arm_cfg)
        arm_cfg["robot_controller"] = "joint_pos"
        start_joints = args.teleop_cfg.start_joints
        arm_cfg["joints_init"] = start_joints[:-1]  # Exclude gripper position
        kwargs["arm_cfg"] = arm_cfg
    if hasattr(args, "arm2"):
        arm2_cfg = asdict(args.arm2_cfg)
        arm2_cfg["robot_controller"] = "joint_pos"
        start_joints = args.teleop2_cfg.start_joints if args.teleop2 else args.teleop_cfg.start_joints
        arm2_cfg["joints_init"] = start_joints[:-1]  # Exclude gripper position
        kwargs["arm2_cfg"] = arm2_cfg
    if hasattr(args, "arm_lead") and args.arm_lead:
        arm_lead_cfg = asdict(args.arm_lead_cfg)
        arm_lead_cfg["robot_controller"] = "joint_pos"
        kwargs["arm_lead_cfg"] = arm_lead_cfg
    if hasattr(args, "arm2_lead") and args.arm2_lead:
        arm2_lead_cfg = asdict(args.arm2_lead_cfg)
        arm2_lead_cfg["robot_controller"] = "joint_pos"
        kwargs["arm2_lead_cfg"] = arm2_lead_cfg

    servers, clients = RealEnv.make_nodes(args, **kwargs)

    from rio_hw.middleware import ServerManager

    with ServerManager(args.mw, list(servers.values())):
        with (
            clients["teleop"]() as teleop,
            clients["teleop2"]() if clients["teleop2"] else nullcontext() as teleop2,
            clients["arm_lead"]() if clients.get("arm_lead") else nullcontext() as arm_lead,
            clients["gripper_lead"]() if clients.get("gripper_lead") else nullcontext() as gripper_lead,
            clients["arm2_lead"]() if clients.get("arm2_lead") else nullcontext() as arm2_lead,
            clients["gripper2_lead"]() if clients.get("gripper2_lead") else nullcontext() as gripper2_lead,
            clients["arm"]() if clients["arm"] else nullcontext() as arm,
            clients["gripper"]() if clients["gripper"] else nullcontext() as gripper,
            clients["arm2"]() if clients["arm2"] else nullcontext() as arm2,
            clients["gripper2"]() if clients["gripper2"] else nullcontext() as gripper2,
        ):
            # Follower grippers
            _gripper, _gripper2 = gripper, gripper2
            if getattr(args, "gripper", None) in ("arm",) and arm:
                _gripper = RealEnv.IntegratedGripper(arm)
            if getattr(args, "gripper2", None) in ("arm2",) and arm2:
                _gripper2 = RealEnv.IntegratedGripper(arm2)
            # Lead grippers
            _gripper_lead, _gripper2_lead = gripper_lead, gripper2_lead
            if getattr(args, "gripper_lead", None) in ("arm_lead",) and arm_lead:
                _gripper_lead = RealEnv.IntegratedGripper(arm_lead)
            if getattr(args, "gripper2_lead", None) in ("arm2_lead",) and arm2_lead:
                _gripper2_lead = RealEnv.IntegratedGripper(arm2_lead)

            teleop_leadfollow(
                args, teleop, teleop2, arm_lead, _gripper_lead, arm2_lead, _gripper2_lead, arm, _gripper, arm2, _gripper2
            )


@dataclass
class GelloCfg:
    port: str = "/dev/ttyUSB0"
    baudrate: int = 57600
    joint_ids: tuple = (1, 2, 3, 4, 5, 6, 7, 8)
    joint_offsets: tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
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

    arm_latency: float = 0.0
    gripper_latency: float = 0.1

    mw: str = "Shm"  # middleware
    mp_method: str | None = "fork"
    freq: int = 200

    def __post_init__(self):
        def load_gello_yaml(yaml_path):
            with open(yaml_path) as f:
                teleop_cfg = yaml.safe_load(f)
            teleop_kwargs = {
                "port": teleop_cfg["agent"]["port"],
                "baudrate": teleop_cfg["agent"].get("baudrate", 57600),
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
    mp.set_start_method(args.mp_method, force=True)
    main(args)
