import os
from dataclasses import asdict
from importlib import import_module

from . import station_cfgs

STATION = os.environ.get("STATION", station_cfgs.__all__[-1])
StationCfg = getattr(station_cfgs, STATION)

PACKAGE = os.environ.get("_PACKAGE", "rio_hw")


def make_node(mw, module, node, node_kwargs):
    if node is None:
        node_server = None
        node_client = None
    else:
        module = import_module(module)
        NodeServer = getattr(module, f"{node}Server", None)
        NodeClient = getattr(module, f"{node}Client", None)
        if NodeServer is None or NodeClient is None:
            raise ImportError(node)
        node_server = lambda: NodeServer(mw, **node_kwargs)
        node_client = lambda: NodeClient(mw, **node_kwargs)
    return node_server, node_client


class RealEnv:
    class IntegratedGripper:
        """Wraps an arm_gripper node to expose only the integrated gripper api."""

        def __init__(self, arm_client):
            self._arm = arm_client

        def get_state(self, *args, **kwargs):
            state = self._arm.get_state(*args, **kwargs)
            return {k: v for k, v in state.items() if k.startswith("gripper_")}

        def moveG(self, *args, **kwargs):
            return self._arm.moveG(*args, **kwargs)

    @classmethod
    def make_nodes(cls, args, **kwargs):
        servers = {}
        clients = {}

        # teleop
        try:
            teleop_module = kwargs.get("teleop_module", f"{PACKAGE}.interfaces")
            teleop_cfg = kwargs.get("teleop_cfg", asdict(args.teleop_cfg))
            servers["teleop"], clients["teleop"] = make_node(args.mw, teleop_module, args.teleop, teleop_cfg)
        except AttributeError:
            servers["teleop"], clients["teleop"] = None, None
        try:
            teleop2_module = kwargs.get("teleop2_module", f"{PACKAGE}.interfaces")
            teleop2_cfg = kwargs.get("teleop2_cfg", asdict(args.teleop2_cfg))
            servers["teleop2"], clients["teleop2"] = make_node(args.mw, teleop2_module, args.teleop2, teleop2_cfg)
        except AttributeError:
            servers["teleop2"], clients["teleop2"] = None, None

        # arm_lead
        try:
            arm_lead_module = kwargs.get("arm_lead_module", f"{PACKAGE}.robots")
            arm_lead_cfg = kwargs.get("arm_lead_cfg", asdict(args.arm_lead_cfg))
            servers["arm_lead"], clients["arm_lead"] = make_node(args.mw, arm_lead_module, args.arm_lead, arm_lead_cfg)
        except AttributeError:
            servers["arm_lead"], clients["arm_lead"] = None, None
        try:
            arm2_lead_module = kwargs.get("arm2_lead_module", f"{PACKAGE}.robots")
            arm2_lead_cfg = kwargs.get("arm2_lead_cfg", asdict(args.arm2_lead_cfg))
            servers["arm2_lead"], clients["arm2_lead"] = make_node(args.mw, arm2_lead_module, args.arm2_lead, arm2_lead_cfg)
        except AttributeError:
            servers["arm2_lead"], clients["arm2_lead"] = None, None

        # gripper_lead
        try:
            if getattr(args, "gripper_lead", None) in ("arm_lead",):
                servers["gripper_lead"], clients["gripper_lead"] = None, None
            else:
                gripper_lead_module = kwargs.get("gripper_lead_module", f"{PACKAGE}.robots")
                gripper_lead_cfg = kwargs.get("gripper_lead_cfg", asdict(args.gripper_lead_cfg))
                servers["gripper_lead"], clients["gripper_lead"] = make_node(
                    args.mw, gripper_lead_module, args.gripper_lead, gripper_lead_cfg
                )
        except AttributeError:
            servers["gripper_lead"], clients["gripper_lead"] = None, None
        try:
            if getattr(args, "gripper2_lead", None) in ("arm2_lead",):
                servers["gripper2_lead"], clients["gripper2_lead"] = None, None
            else:
                gripper2_lead_module = kwargs.get("gripper2_lead_module", f"{PACKAGE}.robots")
                gripper2_lead_cfg = kwargs.get("gripper2_lead_cfg", asdict(args.gripper2_lead_cfg))
                servers["gripper2_lead"], clients["gripper2_lead"] = make_node(
                    args.mw, gripper2_lead_module, args.gripper2_lead, gripper2_lead_cfg
                )
        except AttributeError:
            servers["gripper2_lead"], clients["gripper2_lead"] = None, None

        # arm
        try:
            arm_module = kwargs.get("arm_module", f"{PACKAGE}.robots")
            arm_cfg = kwargs.get("arm_cfg", asdict(args.arm_cfg))
            servers["arm"], clients["arm"] = make_node(args.mw, arm_module, args.arm, arm_cfg)
        except AttributeError:
            servers["arm"], clients["arm"] = None, None
        try:
            arm2_module = kwargs.get("arm2_module", f"{PACKAGE}.robots")
            arm2_cfg = kwargs.get("arm2_cfg", asdict(args.arm2_cfg))
            servers["arm2"], clients["arm2"] = make_node(args.mw, arm2_module, args.arm2, arm2_cfg)
        except AttributeError:
            servers["arm2"], clients["arm2"] = None, None

        # gripper
        try:
            if getattr(args, "gripper", None) in ("arm",):
                servers["gripper"], clients["gripper"] = None, None
            else:
                gripper_module = kwargs.get("gripper_module", f"{PACKAGE}.robots")
                gripper_cfg = kwargs.get("gripper_cfg", asdict(args.gripper_cfg))
                servers["gripper"], clients["gripper"] = make_node(args.mw, gripper_module, args.gripper, gripper_cfg)
        except AttributeError:
            servers["gripper"], clients["gripper"] = None, None
        try:
            if getattr(args, "gripper2", None) in ("arm2",):
                servers["gripper2"], clients["gripper2"] = None, None
            else:
                gripper2_module = kwargs.get("gripper2_module", f"{PACKAGE}.robots")
                gripper2_cfg = kwargs.get("gripper2_cfg", asdict(args.gripper2_cfg))
                servers["gripper2"], clients["gripper2"] = make_node(args.mw, gripper2_module, args.gripper2, gripper2_cfg)
        except AttributeError:
            servers["gripper2"], clients["gripper2"] = None, None

        return servers, clients
