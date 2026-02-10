import os
from dataclasses import asdict
from importlib import import_module

from . import station_cfgs

STATION = os.environ.get("STATION", station_cfgs.__all__[-1])
StationCfg = getattr(station_cfgs, STATION)


def make_node(mw, module, node, node_kwargs):
    if node is None:
        node_server = None
        node_client = None
    else:
        module = import_module(f"rio_hw.{module}")
        NodeServer = getattr(module, f"{node}Server", None)
        NodeClient = getattr(module, f"{node}Client", None)
        if NodeServer is None or NodeClient is None:
            raise ImportError(node)
        node_server = lambda: NodeServer(mw, **node_kwargs)
        node_client = lambda: NodeClient(mw, **node_kwargs)
    return node_server, node_client


class RealEnv:
    @classmethod
    def make_nodes(cls, args, **kwargs):
        servers = {}
        clients = {}

        # teleop
        try:
            teleop_module = kwargs.get("teleop_module", "interfaces")
            teleop_cfg = kwargs.get("teleop_cfg", asdict(args.teleop_cfg))
            servers["teleop"], clients["teleop"] = make_node(args.mw, teleop_module, args.teleop, teleop_cfg)
        except AttributeError:
            servers["teleop"], clients["teleop"] = None, None
        try:
            teleop2_module = kwargs.get("teleop2_module", "interfaces")
            teleop2_cfg = kwargs.get("teleop2_cfg", asdict(args.teleop2_cfg))
            servers["teleop2"], clients["teleop2"] = make_node(args.mw, teleop2_module, args.teleop2, teleop2_cfg)
        except AttributeError:
            servers["teleop2"], clients["teleop2"] = None, None

        # arm
        try:
            arm_module = kwargs.get("arm_module", "robots")
            arm_cfg = kwargs.get("arm_cfg", asdict(args.arm_cfg))
            servers["arm"], clients["arm"] = make_node(args.mw, arm_module, args.arm, arm_cfg)
        except AttributeError:
            servers["arm"], clients["arm"] = None, None
        try:
            arm2_module = kwargs.get("arm2_module", "robots")
            arm2_cfg = kwargs.get("arm2_cfg", asdict(args.arm2_cfg))
            servers["arm2"], clients["arm2"] = make_node(args.mw, arm2_module, args.arm2, arm2_cfg)
        except AttributeError:
            servers["arm2"], clients["arm2"] = None, None

        # gripper
        try:
            gripper_module = kwargs.get("gripper_module", "robots")
            gripper_cfg = kwargs.get("gripper_cfg", asdict(args.gripper_cfg))
            servers["gripper"], clients["gripper"] = make_node(args.mw, gripper_module, args.gripper, gripper_cfg)
        except AttributeError:
            servers["gripper"], clients["gripper"] = None, None
        try:
            gripper2_module = kwargs.get("gripper2_module", "robots")
            gripper2_cfg = kwargs.get("gripper2_cfg", asdict(args.gripper2_cfg))
            servers["gripper2"], clients["gripper2"] = make_node(args.mw, gripper2_module, args.gripper2, gripper2_cfg)
        except AttributeError:
            servers["gripper2"], clients["gripper2"] = None, None

        # hand
        try:
            hand_module = kwargs.get("hand_module", "robots")
            hand_cfg = kwargs.get("hand_cfg", asdict(args.hand_cfg))
            servers["hand"], clients["hand"] = make_node(args.mw, hand_module, args.hand, hand_cfg)
        except AttributeError:
            servers["hand"], clients["hand"] = None, None
        try:
            hand2_module = kwargs.get("hand2_module", "robots")
            hand2_cfg = kwargs.get("hand2_cfg", asdict(args.hand2_cfg))
            servers["hand2"], clients["hand2"] = make_node(args.mw, hand2_module, args.hand2, hand2_cfg)
        except AttributeError:
            servers["hand2"], clients["hand2"] = None, None

        return servers, clients
