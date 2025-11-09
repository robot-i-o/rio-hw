from dataclasses import dataclass, field


@dataclass
class DualXhandFr3Station:
    @dataclass
    class ArmCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "192.168.1.111"
        max_pos_speed: float = 0.25
        max_rot_speed: float = 0.6
        robot_model: str = "fr3"
        driver: str = "panda_py"

    @dataclass
    class HandCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "192.168.1.111"
        robot_model: str = "xhand1"

    arm: str | None = "FrankaArm"
    arm_cfg: ArmCfg = field(
        default_factory=lambda: DualXhandFr3Station.ArmCfg(
            addr="127.0.0.1:5110",
            robot_ip="172.16.0.2",
        )
    )

    hand: str | None = "XhandHand"
    hand_cfg: HandCfg = field(
        default_factory=lambda: DualXhandFr3Station.HandCfg(
            addr="127.0.0.1:5120",
            robot_ip="172.16.0.2",
        )
    )

    arm2: str | None = None
    arm2_cfg: ArmCfg = field(
        default_factory=lambda: DualXhandFr3Station.ArmCfg(
            addr="127.0.0.1:5210",
            robot_ip="172.16.1.2",
        )
    )

    hand2: str | None = None
    hand2_cfg: HandCfg = field(
        default_factory=lambda: DualXhandFr3Station.HandCfg(
            addr="127.0.0.1:5220",
            robot_ip="172.16.1.2",
        )
    )
