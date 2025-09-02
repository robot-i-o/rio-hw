from . import middlewares


def ServerFactory(mw, _Node, *args, **kwargs):
    # dynamic inheritance
    MwCls = getattr(middlewares, f"{mw}Server")
    if mw in ("Shm", "Thread"):
        Cls = type(f"{mw}Server", (MwCls,), {})
        Cls.__api__ = _Node.__api__
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
        Cls.__api__ = _Node.__api__
    return Cls(*args, **kwargs)
