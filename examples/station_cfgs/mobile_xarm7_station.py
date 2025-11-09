from dataclasses import dataclass, field


@dataclass
class MobileXarm7Station:
    @dataclass
    class ArmCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "192.168.1.205"
        robot_model: str = "xarm7"
        max_pos_speed: float = 0.25
        max_rot_speed: float = 0.6

    arm: str | None = "XarmArm"
    arm_cfg: ArmCfg = field(
        default_factory=lambda: MobileXarm7Station.ArmCfg(
            addr="127.0.0.1:5110",
            robot_ip="192.168.1.205",
        )
    )

    gripper: str | None = None
