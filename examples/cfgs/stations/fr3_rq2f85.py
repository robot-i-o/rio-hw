from dataclasses import dataclass, field


@dataclass
class Fr3Rq2f85Station:
    @dataclass
    class ArmCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "192.168.1.111"
        max_pos_speed: float = 0.25
        max_rot_speed: float = 0.6
        # max_pos_speed: float = 0.5
        # max_rot_speed: float = 5.0
        robot_model: str = "fr3"
        driver: str = "panda_py"
        robot_controller: str = "joint_pos"
        # robot_controller: str = "task_pos"

    @dataclass
    class GripperCfg:
        addr: str = "127.0.0.1:5555"

        robot_port: str = "/dev/ttyUSB0"
        connection_type: str = "RTU"

    arm: str | None = "FrankaArm"
    arm_cfg: ArmCfg = field(
        default_factory=lambda: Fr3Rq2f85Station.ArmCfg(
            addr="127.0.0.1:5110",
            robot_ip="172.16.1.2",
        )
    )

    gripper: str | None = "RobotiqGripper"
    gripper_cfg: GripperCfg = field(
        default_factory=lambda: Fr3Rq2f85Station.GripperCfg(
            addr="127.0.0.1:5120",
            robot_port="/dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DAANTKO8-if00-port0",
        )
    )
