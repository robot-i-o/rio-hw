from dataclasses import dataclass, field


@dataclass
class Lite6DuoLfStation:
    @dataclass
    class ArmCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "192.168.1.111"
        robot_model: str = "lite6"
        max_pos_speed: float = 0.25
        max_rot_speed: float = 0.6
        robot_controller: str = "joint_pos"

    @dataclass
    class GripperCfg:
        addr: str = "127.0.0.1:5555"

        robot_port: str = "/dev/ttyUSB0"
        robot_controller: str = "task_pos"

    arm_lead: str | None = "XarmArm"
    arm_lead_cfg: ArmCfg = field(
        default_factory=lambda: Lite6DuoLfStation.ArmCfg(
            addr="127.0.0.1:5210",
            robot_ip="192.168.5.169",
            robot_controller="guide",
        )
    )

    gripper_lead: str | None = "WavesharePiperGripper"
    gripper_lead_cfg: GripperCfg = field(
        default_factory=lambda: Lite6DuoLfStation.GripperCfg(
            addr="127.0.0.1:5220",
            robot_port="/dev/ttyUSB0",
            robot_controller="guide",
        )
    )

    arm2_lead: str | None = "XarmArm"
    arm2_lead_cfg: ArmCfg = field(
        default_factory=lambda: Lite6DuoLfStation.ArmCfg(
            addr="127.0.0.1:5310",
            robot_ip="192.168.4.156",
            robot_controller="guide",
        )
    )

    gripper2_lead: str | None = "WavesharePiperGripper"
    gripper2_lead_cfg: GripperCfg = field(
        default_factory=lambda: Lite6DuoLfStation.GripperCfg(
            addr="127.0.0.1:5320",
            robot_port="/dev/ttyUSB1",
            robot_controller="guide",
        )
    )

    arm: str | None = "XarmArm"
    arm_cfg: ArmCfg = field(
        default_factory=lambda: Lite6DuoLfStation.ArmCfg(
            addr="127.0.0.1:5410",
            robot_ip="192.168.2.176",
        )
    )

    gripper: str | None = "WavesharePiperGripper"
    gripper_cfg: GripperCfg = field(
        default_factory=lambda: Lite6DuoLfStation.GripperCfg(
            addr="127.0.0.1:5420",
            robot_port="/dev/ttyUSB3",
        )
    )

    arm2: str | None = "XarmArm"
    arm2_cfg: ArmCfg = field(
        default_factory=lambda: Lite6DuoLfStation.ArmCfg(
            addr="127.0.0.1:5510",
            robot_ip="192.168.3.181",
        )
    )

    gripper2: str | None = "WavesharePiperGripper"
    gripper2_cfg: GripperCfg = field(
        default_factory=lambda: Lite6DuoLfStation.GripperCfg(
            addr="127.0.0.1:5520",
            robot_port="/dev/ttyUSB2",
        )
    )
