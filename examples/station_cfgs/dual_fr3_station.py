from dataclasses import dataclass, field


@dataclass
class DualFr3Station:
    @dataclass
    class TeleopCfg:
        addr: str = "127.0.0.1:5000"

    @dataclass
    class ArmCfg:
        robot_ip: str = "192.168.1.111"
        addr: str = "127.0.0.1:5010"
        max_pos_speed: float = 0.25
        max_rot_speed: float = 0.6
        robot_model: str = "fr3"
        driver: str = "panda_py"

    @dataclass
    class GripperCfg:
        robot_ip: str = "192.168.1.111"
        addr: str = "127.0.0.1:5020"
        robot_model: str = "fr3_hand"

    teleop: str = "Spacemouse"
    teleop_cfg: TeleopCfg = field(default_factory=lambda: DualFr3Station.TeleopCfg())

    arm: str | None = "Franka"
    arm_cfg: ArmCfg = field(
        default_factory=lambda: DualFr3Station.ArmCfg(
            robot_ip="172.16.0.2",
            addr="127.0.0.1:5010",
        )
    )

    gripper: str | None = "FrankaGripper"
    gripper_cfg: GripperCfg = field(
        default_factory=lambda: DualFr3Station.GripperCfg(
            robot_ip="172.16.0.2",
            addr="127.0.0.1:5020",
        )
    )

    arm2: str | None = "Franka"
    arm2_cfg: ArmCfg = field(
        default_factory=lambda: DualFr3Station.ArmCfg(
            robot_ip="172.16.1.2",
            addr="127.0.0.1:5110",
        )
    )

    gripper2: str | None = None
    gripper2_cfg: GripperCfg = field(
        default_factory=lambda: DualFr3Station.GripperCfg(
            robot_ip="172.16.1.2",
            addr="127.0.0.1:5120",
        )
    )
