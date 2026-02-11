import importlib
import inspect
import multiprocessing as mp
import sys
from collections.abc import Callable

from . import middlewares
from .middlewares import SERVERLESS_MW
from .serializers import CloudpickleSerializer


def __factory__(role, mw, node_module, node_name):
    _Node = getattr(importlib.import_module(node_module), node_name)
    module = sys.modules.get(node_module) or inspect.getmodule(node_module)
    name = f"{node_name}{mw}{role}"
    Cls = getattr(module, name, None)
    if Cls is None:
        Cls = Factory(role, mw, _Node)  # need to recreate class in case of multiprocessing
    return Cls.__new__(Cls)  # not __init__(), using __setstate__() instead


def __factory_reduce__(self):
    args = getattr(self.__class__, "__factory_args__", None)
    if args is None:  # fallback to default __reduce__
        fn, args = self.__class__, ()
    else:
        fn = __factory__
    getstate_fn = getattr(self, "__getstate__", None)
    state = getstate_fn() if getstate_fn is not None else getattr(self, "__dict__", {})
    return (fn, args, state)


def Factory(role, mw, _Node):
    node_module, node_name = _Node.__module__, _Node.__name__
    module = sys.modules.get(node_module) or inspect.getmodule(_Node)
    name = f"{node_name}{mw}{role}"
    Cls = getattr(module, name, None)
    if Cls is not None:
        return Cls

    # dynamic inheritance
    MwCls = getattr(middlewares, f"{mw}{role}")
    if role == "Server":
        bases = (MwCls,) if mw in SERVERLESS_MW else (_Node, MwCls)
    elif role == "Client":
        bases = (_Node, MwCls) if mw in SERVERLESS_MW else (MwCls,)
    else:
        raise ValueError(role)
    # custom attributes
    attrs = {k: getattr(_Node, k) for k in ("__api__", "__pub__", "__req__")}
    attrs["__nodename__"] = node_name
    attrs["__factory_args__"] = (role, mw, node_module, node_name)
    attrs["__reduce__"] = __factory_reduce__

    # NOTE: _Node and MwCls need to both have Node as a parent class
    # so that mro() order used by type() correctly resolves _Node -> MwCls -> Node

    def class_factory(module, name, bases, attrs=None):
        Cls = type(name, bases, {})
        Cls.__module__ = module.__name__
        if attrs is not None:
            for attr, value in attrs.items():
                setattr(Cls, attr, value)
        return Cls

    Cls = class_factory(module, name, bases, attrs)
    setattr(module, name, Cls)  # register new class for module
    return Cls


def ServerFactory(mw, _Node, *args, **kwargs):
    Cls = Factory("Server", mw, _Node)
    return Cls(*args, **kwargs)


def ClientFactory(mw, _Node, *args, **kwargs):
    Cls = Factory("Client", mw, _Node)
    return Cls(*args, **kwargs)


class ServerManager:
    """Helper class to manage multiple servers, each in its own separate process."""

    def __init__(self, mw: str, server_fns: list[Callable | None], start_method: str = "spawn", timeout: float = 5.0):
        server_fns = list(filter(lambda fn: fn is not None, server_fns))
        if mw in SERVERLESS_MW:
            [fn() for fn in server_fns]
            server_fns = []
        self.server_fns = server_fns
        self.start_method = start_method
        self.timeout = timeout
        self.__post_init__()

    def __post_init__(self):
        ctx = mp.get_context(self.start_method)
        self.start_event = ctx.Event()
        self.stop_event = ctx.Event()
        self.ready_barrier = ctx.Barrier(len(self.server_fns) + 1)  # add one for main process
        server_fns = [CloudpickleSerializer.pack(server_fn) for server_fn in self.server_fns]  # to deal with lambda fns
        args = (self.start_event, self.stop_event, self.ready_barrier)
        self.procs = [ctx.Process(target=self._worker, args=(server_fn, *args), daemon=True) for server_fn in server_fns]

    @staticmethod
    def _worker(server_fn, start_event, stop_event, ready_barrier):
        ready = False
        try:
            server_fn = CloudpickleSerializer.unpack(server_fn)
            server = server_fn()
            start_event.wait()
            if server is not None:
                server.start()
            ready_barrier.wait()
            ready = True
            stop_event.wait()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            import traceback  # noqa

            print(e, traceback.format_exc())
            exit()
        finally:
            if ready:
                if server is not None:
                    server.stop()

    def start(self):
        for proc in self.procs:
            proc.start()
        self.start_event.set()
        self.ready_barrier.wait(timeout=self.timeout)
        for proc in self.procs:
            assert proc.is_alive()

    def stop(self):
        self.stop_event.set()
        for proc in self.procs:
            proc.join(self.timeout)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_traceback):
        self.stop()
