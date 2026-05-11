from dataclasses import dataclass, field


@dataclass
class G1Station:
    @dataclass
    class HumanoidCfg:
        addr: str = "127.0.0.1:5555"

        robot_ip: str = "eth0"

    humanoid: str | None = "UnitreeG1"
    humanoid_cfg: HumanoidCfg = field(
        default_factory=lambda: G1Station.HumanoidCfg(
            addr="127.0.0.1:5110",
            robot_ip="enx9c69d33c49cc",
        )
    )
