from dataclasses import dataclass, field


@dataclass
class Fr3DuoStation:
    @dataclass
    class ArmCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "192.168.1.111"
        robot_model: str = "fr3"
        driver: str = "panda_py"
        max_pos_speed: float = 0.25
        max_rot_speed: float = 0.6
        robot_controller: str = "task_pos"

    @dataclass
    class GripperCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "192.168.1.111"
        robot_model: str = "fr3_hand"

    arm: str | None = "FrankaArm"
    arm_cfg: ArmCfg = field(
        default_factory=lambda: Fr3DuoStation.ArmCfg(
            addr="127.0.0.1:5110",
            robot_ip="172.16.0.2",
        )
    )

    gripper: str | None = "FrankaGripper"
    gripper_cfg: GripperCfg = field(
        default_factory=lambda: Fr3DuoStation.GripperCfg(
            addr="127.0.0.1:5120",
            robot_ip="172.16.0.2",
        )
    )

    arm2: str | None = "FrankaArm"
    arm2_cfg: ArmCfg = field(
        default_factory=lambda: Fr3DuoStation.ArmCfg(
            addr="127.0.0.1:5210",
            robot_ip="172.16.1.2",
        )
    )

    gripper2: str | None = "FrankaGripper"
    gripper2_cfg: GripperCfg = field(
        default_factory=lambda: Fr3DuoStation.GripperCfg(
            addr="127.0.0.1:5220",
            robot_ip="172.16.1.2",
        )
    )
