from dataclasses import dataclass, field


@dataclass
class Xarm7Station:
    @dataclass
    class ArmCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "192.168.1.111"
        robot_model: str = "xarm7"
        max_pos_speed: float = 0.25
        max_rot_speed: float = 0.6

    @dataclass
    class GripperCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "192.168.1.111"
        robot_model: str = "g1"

    arm: str | None = "XarmArm"
    arm_cfg: ArmCfg = field(
        default_factory=lambda: Xarm7Station.ArmCfg(
            addr="127.0.0.1:5110",
            robot_ip="192.168.1.228",
        )
    )

    gripper: str | None = "XarmGripper"
    gripper_cfg: GripperCfg = field(
        default_factory=lambda: Xarm7Station.GripperCfg(
            addr="127.0.0.1:5120",
            robot_ip="192.168.1.228",
        )
    )
