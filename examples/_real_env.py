import os
from dataclasses import asdict
from importlib import import_module

from .cfgs import stations as StationCfgs

STATION = os.environ.get("STATION", StationCfgs.__all__[-1])
StationCfg = getattr(StationCfgs, STATION)

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

        node_defs = [
            # (node name, default module)
            ("teleop", f"{PACKAGE}.interfaces"),
            ("teleop2", f"{PACKAGE}.interfaces"),
            ("arm_lead", f"{PACKAGE}.robots"),
            ("arm2_lead", f"{PACKAGE}.robots"),
            ("gripper_lead", f"{PACKAGE}.robots"),
            ("gripper2_lead", f"{PACKAGE}.robots"),
            ("arm", f"{PACKAGE}.robots"),
            ("arm2", f"{PACKAGE}.robots"),
            ("gripper", f"{PACKAGE}.robots"),
            ("gripper2", f"{PACKAGE}.robots"),
        ]

        for name, default_module in node_defs:
            try:
                module = kwargs.get(f"{name}_module", default_module)
                cfg = kwargs.get(f"{name}_cfg", asdict(getattr(args, f"{name}_cfg")))
                servers[name], clients[name] = make_node(args.mw, module, getattr(args, name), cfg)
            except AttributeError:
                servers[name], clients[name] = None, None

        return servers, clients
