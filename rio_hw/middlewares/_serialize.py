import functools
import inspect
from types import MethodType


def get_fn(cls, fn_name):
    # get descriptor object instead of the underlying attribute
    fn_descriptor = inspect.getattr_static(cls, fn_name)
    if isinstance(fn_descriptor, classmethod) or isinstance(fn_descriptor, staticmethod):
        fn = fn_descriptor.__func__
    else:
        fn = fn_descriptor
    assert callable(fn)
    return fn_descriptor, fn


def wrap_fn_pack(cls, fn_name, fn_descriptor, fn, fn_wrapper):
    wrapped = functools.wraps(fn)(fn_wrapper)
    if isinstance(fn_descriptor, classmethod):
        wrapped = classmethod(wrapped)
    elif isinstance(fn_descriptor, staticmethod):
        wrapped = staticmethod(wrapped)
    setattr(cls, fn_name, wrapped)


def wrap_fn_unpack(self, fn_name, fn_wrapper):
    fn_wrapper.__name__ = fn_name
    fn_wrapper.__qualname__ = f"{self.__class__.__name__}.{fn_name}"
    fn_wrapper = MethodType(fn_wrapper, self)
    setattr(self, fn_name, fn_wrapper)
