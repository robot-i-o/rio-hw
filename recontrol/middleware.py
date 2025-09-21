import inspect
import multiprocessing as mp
from collections.abc import Callable

from . import middlewares
from .middlewares import Node
from .serializers import CloudpickleSerializer

SERVERLESS_MW = ("Shm", "Thread")


def class_factory(name, bases, module, attrs=None):
    Cls = type(name, bases, {})
    Cls.__module__ = module.__name__
    # setattr(module, name, Cls)  # register new class for module
    if attrs is not None:
        for attr, value in attrs.items():
            setattr(Cls, attr, value)
    return Cls


def ServerFactory(mw, _Node, *args, **kwargs):
    # dynamic inheritance
    MwCls = getattr(middlewares, f"{mw}Server")
    Module = inspect.getmodule(_Node)
    if mw in SERVERLESS_MW:
        attrs = {k: getattr(_Node, k) for k in ("__api__", "__pub__", "__req__")}
        Cls = class_factory(f"{_Node.__name__}{mw}Server", (MwCls,), Module, attrs)
    else:
        Cls = class_factory(f"{_Node.__name__}{mw}Server", (_Node, MwCls), Module)
    return Cls(*args, **kwargs)


def ClientFactory(mw, _Node, *args, **kwargs):
    # dynamic inheritance
    MwCls = getattr(middlewares, f"{mw}Client")
    Module = inspect.getmodule(_Node)
    if mw in SERVERLESS_MW:
        Cls = class_factory(f"{_Node.__name__}{mw}Client", (_Node, MwCls), Module)
    else:
        attrs = {k: getattr(_Node, k) for k in ("__api__", "__pub__", "__req__")}
        Cls = class_factory(f"{_Node.__name__}{mw}Client", (MwCls,), Module, attrs)
    return Cls(*args, **kwargs)


class ServerManager(Node):
    """Helper class to manage multiple servers, each in its own separate process."""

    def __init__(self, mw: str, server_fns: list[Callable], start_method: str = "spawn", timeout: float = 5.0):
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
