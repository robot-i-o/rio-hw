from dataclasses import dataclass, field


@dataclass
class XArm7Station:
    @dataclass
    class TeleopCfg:
        addr: str = "127.0.0.1:5100"

    @dataclass
    class ArmCfg:
        robot_ip: str = "192.168.1.111"
        addr: str = "127.0.0.1:5110"
        robot_model: str = "xarm7"
        max_pos_speed: float = 0.25
        max_rot_speed: float = 0.6

    @dataclass
    class GripperCfg:
        robot_ip: str = "192.168.1.111"
        addr: str = "127.0.0.1:5120"
        robot_model: str = "g1"

    teleop: str = "Spacemouse"
    teleop_cfg: TeleopCfg = field(default_factory=lambda: XArm7Station.TeleopCfg())

    arm: str | None = "XArm"
    arm_cfg: ArmCfg = field(default_factory=lambda: XArm7Station.ArmCfg(robot_ip="192.168.1.228"))

    gripper: str | None = "XArmGripper"
    gripper_cfg: GripperCfg = field(default_factory=lambda: XArm7Station.GripperCfg(robot_ip="192.168.1.228"))
