import asyncio
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..middleware import ClientFactory
from ..node import Node

try:
    import sshkeyboard
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        sshkeyboard = None  # type: ignore

# USB HID usage codes for special keys (matching pynput's Key.value.vk values where possible).
_SPECIAL_KEY_CODES = {
    "backspace": 0x2A,
    "tab": 0x2B,
    "enter": 0x28,
    "esc": 0x29,
    "space": 0x2C,
    "delete": 0x4C,
    "up": 0x52,
    "down": 0x51,
    "left": 0x50,
    "right": 0x4F,
    "home": 0x4A,
    "end": 0x4D,
    "pageup": 0x4B,
    "pagedown": 0x4E,
    "insert": 0x49,
    "f1": 0x3A,
    "f2": 0x3B,
    "f3": 0x3C,
    "f4": 0x3D,
    "f5": 0x3E,
    "f6": 0x3F,
    "f7": 0x40,
    "f8": 0x41,
    "f9": 0x42,
    "f10": 0x43,
    "f11": 0x44,
    "f12": 0x45,
}


class SshKeyboard(Node):
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
        alphanumeric_keys = set()
        special_keys = set()

        def on_press(key):
            if len(key) == 1:
                alphanumeric_keys.add(ord(key))
            else:
                special_keys.add(_SPECIAL_KEY_CODES.get(key, hash(key) % (2**31)))

        def on_release(key):
            if len(key) == 1:
                alphanumeric_keys.discard(ord(key))
            else:
                special_keys.discard(_SPECIAL_KEY_CODES.get(key, hash(key) % (2**31)))

        async def poll_loop():
            not_pub_ready = True
            period = 1.0 / self.freq
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
                await asyncio.sleep(period)

        async def main():
            listener_task = asyncio.create_task(
                sshkeyboard.listen_keyboard_manual(
                    on_press=on_press,
                    on_release=on_release,
                    until=None,
                    lower=True,
                    sequential=True,
                    delay_second_char=0.333,
                    delay_other_chars=0.01,
                    sleep=0.001,
                    max_thread_pool_workers=1,
                )
            )
            try:
                await poll_loop()
            except KeyboardInterrupt:
                pass
            finally:
                sshkeyboard.stop_listening()
                await listener_task

        try:
            asyncio.run(main())
        except KeyboardInterrupt:
            pass

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()


def SshKeyboardServer(mw, *args, **kwargs):
    return None


def SshKeyboardClient(mw, *args, **kwargs):
    # force "Thread" middleware since sshkeyboard must be used in the same process
    return ClientFactory("Thread", SshKeyboard, *args, **kwargs)
