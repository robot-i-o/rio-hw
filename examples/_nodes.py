import os
from importlib import import_module

from . import station_cfgs

STATION = os.environ.get("STATION", station_cfgs.__all__[-1])
StationCfg = getattr(station_cfgs, STATION)


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
