from dataclasses import dataclass, field


@dataclass
class DualLite6Station:
    @dataclass
    class ArmCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "192.168.1.111"
        max_pos_speed: float = 0.25
        max_rot_speed: float = 0.6
        robot_model: str = "lite6"

    @dataclass
    class GripperCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "192.168.1.111"
        robot_model: str = "lite6"

    arm: str | None = "XarmArm"
    arm_cfg: ArmCfg = field(
        default_factory=lambda: DualLite6Station.ArmCfg(
            addr="127.0.0.1:5110",
            robot_ip="192.168.2.176",
        )
    )

    gripper: str | None = None
    gripper_cfg: GripperCfg = field(
        default_factory=lambda: DualLite6Station.GripperCfg(
            addr="127.0.0.1:5120",
            robot_ip="192.168.2.176",
        )
    )

    arm2: str | None = "XarmArm"
    arm2_cfg: ArmCfg = field(
        default_factory=lambda: DualLite6Station.ArmCfg(
            addr="127.0.0.1:5210",
            robot_ip="192.168.3.181",
        )
    )

    gripper2: str | None = None
    gripper2_cfg: GripperCfg = field(
        default_factory=lambda: DualLite6Station.GripperCfg(
            addr="127.0.0.1:5220",
            robot_ip="192.168.3.181",
        )
    )

    @dataclass
    class CameraCfg:
        serial: str = ""
        model: str = "D400"

    camera: str | None = "Realsense"
    camera_cfg: CameraCfg = field(
        default_factory=lambda: DualLite6Station.CameraCfg(
            serial="",
            model="D400",
        )
    )

    camera2: str | None = "Realsense"
    camera2_cfg: CameraCfg = field(
        default_factory=lambda: DualLite6Station.CameraCfg(
            serial="",
            model="D400",
        )
    )

    camerah: str | None = "Realsense"
    camerah_cfg: CameraCfg = field(
        default_factory=lambda: DualLite6Station.CameraCfg(
            serial="",
            model="D400",
        )
    )

    @dataclass
    class SceneCfg:
        scene_names: dict[str, str] = field(
            default_factory=lambda: {
                "arm": "arm_right",
                "arm2": "arm_left",
                "gripper": "gripper_right",
                "gripper2": "gripper_left",
                "camera": "camera_wrist_right",
                "camera2": "camera_wrist_left",
                "camerah": "camera_head",
            }
        )
        scene_models: dict[str, str] | None = None
