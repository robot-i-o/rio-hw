from dataclasses import dataclass, field


@dataclass
class LeapHandXarm7Station:
    @dataclass
    class ArmCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "192.168.1.111"
        robot_model: str = "xarm7"
        max_pos_speed: float = 0.25
        max_rot_speed: float = 0.6

    @dataclass
    class HandCfg:
        addr: str = "127.0.0.1:5555"

        robot_port: str = "/dev/ttyUSB0"
        robot_model: str = "leapv1"

    arm: str | None = "XarmArm"
    arm_cfg: ArmCfg = field(
        default_factory=lambda: LeapHandXarm7Station.ArmCfg(
            addr="127.0.0.1:5110",
            robot_ip="192.168.1.228",
        )
    )

    hand: str | None = "LeapHand"
    hand_cfg: HandCfg = field(
        default_factory=lambda: LeapHandXarm7Station.HandCfg(
            addr="127.0.0.1:5120",
            robot_port="/dev/ttyUSB0",
        )
    )
