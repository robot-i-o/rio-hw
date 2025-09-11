import os
from dataclasses import dataclass, field
from importlib import import_module


def make_node(mw, module, node, node_kwargs):
    if node is None:
        node_server = lambda: None
        node_client = None
    else:
        module = import_module(f"recontrol.{module}")
        NodeServer = getattr(module, f"{node}Server", None)
        NodeClient = getattr(module, f"{node}Client", None)
        if NodeServer is None or NodeClient is None:
            raise ImportError(node)
        node_server = lambda: NodeServer(mw, **node_kwargs)
        node_client = lambda: NodeClient(mw, **node_kwargs)
    return node_server, node_client


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


STATIONS = [
    XArm7Station,
]

STATION = os.environ.get("STATION", "XArm7Station")
StationCfg = next(s for s in STATIONS if s.__name__ == STATION)
