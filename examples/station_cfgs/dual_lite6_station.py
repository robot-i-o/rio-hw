from dataclasses import dataclass, field


@dataclass
class DualLite6Station:
    @dataclass
    class ArmCfg:
        robot_ip: str = "192.168.1.111"
        addr: str = "127.0.0.1:5010"
        max_pos_speed: float = 0.25
        max_rot_speed: float = 0.6
        robot_model: str = "lite6"

    @dataclass
    class GripperCfg:
        robot_ip: str = "192.168.1.111"
        addr: str = "127.0.0.1:5020"
        robot_model: str = "lite6"

    arm: str | None = "XArm"
    arm_cfg: ArmCfg = field(
        default_factory=lambda: DualLite6Station.ArmCfg(
            robot_ip="192.168.2.176",
            addr="127.0.0.1:5010",
        )
    )

    gripper: str | None = None
    gripper_cfg: GripperCfg = field(
        default_factory=lambda: DualLite6Station.GripperCfg(
            robot_ip="192.168.2.176",
            addr="127.0.0.1:5020",
        )
    )

    arm2: str | None = "XArm"
    arm2_cfg: ArmCfg = field(
        default_factory=lambda: DualLite6Station.ArmCfg(
            robot_ip="192.168.3.181",
            addr="127.0.0.1:5110",
        )
    )

    gripper2: str | None = None
    gripper2_cfg: GripperCfg = field(
        default_factory=lambda: DualLite6Station.GripperCfg(
            robot_ip="192.168.3.181",
            addr="127.0.0.1:5120",
        )
    )
