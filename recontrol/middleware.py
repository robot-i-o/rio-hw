import multiprocessing as mp
from collections.abc import Callable

from . import middlewares
from .middlewares import Node
from .serializers import CloudpickleSerializer


def ServerFactory(mw, _Node, *args, **kwargs):
    # dynamic inheritance
    MwCls = getattr(middlewares, f"{mw}Server")
    if mw in ("Shm", "Thread"):
        Cls = type(f"{mw}Server", (MwCls,), {})
        for attr in ("__api__", "__pub__", "__req__"):
            setattr(Cls, attr, getattr(_Node, attr))
    else:
        Cls = type(f"{mw}Server", (_Node, MwCls), {})
    return Cls(*args, **kwargs)


def ClientFactory(mw, _Node, *args, **kwargs):
    # dynamic inheritance
    MwCls = getattr(middlewares, f"{mw}Client")
    if mw in ("Shm", "Thread"):
        Cls = type(f"{mw}Client", (_Node, MwCls), {})
    else:
        Cls = type(f"{mw}Client", (MwCls,), {})
        for attr in ("__api__", "__pub__", "__req__"):
            setattr(Cls, attr, getattr(_Node, attr))
    return Cls(*args, **kwargs)


class ServerManager(Node):
    """Helper class to manage multiple servers, each in its own separate process."""

    def __init__(self, mw: str, server_fns: list[Callable], start_method: str = "spawn", timeout: float = 5.0):
        if mw in ("Shm", "Thread"):
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
