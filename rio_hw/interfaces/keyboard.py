import threading
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node

try:
    from pynput import keyboard
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        keyboard = None  # type: ignore


class Keyboard(Node):
    __api__ = [
        "get_state",
        "get_all_state",
    ]
    __pub__ = True
    __req__ = False

    def __init__(
        self,
        n_key_rollover: int = 6,
        dtype=np.float32,
        *,
        freq: int = 100,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        self.n_key_rollover = n_key_rollover
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_request = None
        self.example_data = {
            "alphanumeric_state": np.zeros((self.n_key_rollover,), dtype=np.byte),
            "special_state": np.zeros((self.n_key_rollover,), dtype=int),
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        lock = threading.Lock()
        alphanumeric_keys = set()
        special_keys = set()

        def get_key(key):
            try:  # alphanumeric keyboard.KeyCode
                c = True
                k = ord(key.char)
            except AttributeError:  # special keyboard.Key
                c = False
                k = key.value.vk
            return c, k

        def on_press(key):
            if key is not None:
                if getattr(key, "char", False) is None and key.vk == 0:  # handle unknown key <0>
                    return
                c, k = get_key(key)
                with lock:
                    alphanumeric_keys.add(k) if c else special_keys.add(k)

        def on_release(key):
            if key is not None:
                if getattr(key, "char", False) is None and key.vk == 0:  # handle unknown key <0>
                    return
                c, k = get_key(key)
                with lock:
                    alphanumeric_keys.discard(k) if c else special_keys.discard(k)

        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        try:
            # Main loop
            rate = time.Rate(self.freq)
            not_pub_ready = True
            while not self.exit_event.is_set():
                alphanumeric_state = np.zeros((self.n_key_rollover,), dtype=np.byte)
                special_state = np.zeros((self.n_key_rollover,), dtype=int)
                for i, key in enumerate(alphanumeric_keys):
                    alphanumeric_state[i] = key
                for i, key in enumerate(special_keys):
                    special_state[i] = key

                # Store current state in ring buffer
                data = {
                    "alphanumeric_state": alphanumeric_state,
                    "special_state": special_state,
                    "timestamp": time.now(),
                }
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            listener.join()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()


def KeyboardServer(mw, *args, **kwargs):
    return ServerFactory(mw, Keyboard, *args, **kwargs)


def KeyboardClient(mw, *args, **kwargs):
    return ClientFactory(mw, Keyboard, *args, **kwargs)
