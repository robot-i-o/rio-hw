from dataclasses import dataclass, field


@dataclass
class MobileXArm7Station:
    @dataclass
    class ArmCfg:
        robot_ip: str = "192.168.1.205"
        addr: str = "127.0.0.1:5110"
        robot_model: str = "xarm7"
        max_pos_speed: float = 0.25
        max_rot_speed: float = 0.6

    arm: str | None = "Xarm"
    arm_cfg: ArmCfg = field(default_factory=lambda: MobileXArm7Station.ArmCfg(robot_ip="192.168.1.205"))

    gripper: str | None = None
