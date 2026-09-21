import os
import select
import threading
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node

try:
    import evdev
    from evdev import ecodes
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        evdev = None  # type: ignore
        ecodes = None  # type: ignore

# evdev keycodes run to KEY_MAX (767); counters are indexed directly by keycode.
KEY_COUNT = 768


def find_keyboard(
    device_path: str | None = None,
    vendor_id: int | None = None,
    product_id: int | None = None,
    device_name: str | None = None,
):
    """Resolve an evdev input device from a path, USB id, or name substring.

    Matching is tried in order: explicit path, then vendor/product id, then a
    case-insensitive name substring. Devices without key events (mice, vendor HID
    interfaces) are skipped so composite devices resolve to their keyboard interface.

    Args:
        device_path: Explicit device node, e.g. "/dev/input/event27" or a
            "/dev/input/by-id/..." symlink.
        vendor_id: USB vendor id to match, e.g. 0x3553.
        product_id: USB product id to match, e.g. 0xC011.
        device_name: Substring matched against the device name.

    Returns:
        The matching `evdev.InputDevice`.

    Raises:
        RuntimeError: If no device matches, or if `/dev/input` is unreadable.
    """
    if device_path is not None:
        return evdev.InputDevice(device_path)

    paths = evdev.list_devices()
    if not paths:
        raise RuntimeError(
            "No readable devices in /dev/input. This is almost always a permissions "
            "issue: add the user to the 'input' group (and reconnect the session), or "
            "install a udev rule granting access to the device."
        )

    candidates = []
    for path in paths:
        device = evdev.InputDevice(path)
        if ecodes.EV_KEY not in device.capabilities():
            continue  # skip mouse-only and vendor HID interfaces
        if vendor_id is not None and device.info.vendor != vendor_id:
            continue
        if product_id is not None and device.info.product != product_id:
            continue
        if device_name is not None and device_name.lower() not in device.name.lower():
            continue
        candidates.append(device)

    if not candidates:
        visible = ", ".join(f"{evdev.InputDevice(p).name} ({p})" for p in paths)
        raise RuntimeError(
            f"No input device matched vendor_id={vendor_id}, product_id={product_id}, "
            f"device_name={device_name!r}. Visible devices: {visible}"
        )

    # A composite device (button pad, gaming keyboard) exposes several key-capable
    # interfaces, often all advertising letter keys. udev marks the real keyboard
    # interface with an "-event-kbd" symlink, which is a far better discriminator
    # than the reported capabilities.
    kbd_paths = set()
    by_id = os.path.join("/dev/input/by-id")
    if os.path.isdir(by_id):
        for name in os.listdir(by_id):
            if name.endswith("-event-kbd"):
                kbd_paths.add(os.path.realpath(os.path.join(by_id, name)))

    for device in candidates:
        if os.path.realpath(device.path) in kbd_paths:
            return device
    return candidates[0]


class EvdevKeyboard(Node):
    """Keyboard/button node reading the Linux evdev input layer directly.

    Unlike the pynput-backed `Keyboard`, this reads `/dev/input/event*` on the machine
    the node runs on, so it needs no X/Wayland display, no controlling terminal, and no
    window focus. That makes it the node to use for buttons physically attached to a
    robot host that is only reachable over SSH.

    Published state carries both the currently held keys and a monotonic per-keycode
    press counter. Consumers should diff the counter against the previously seen value
    rather than reading held state, so a tap shorter than one publish period is still
    observed exactly once.
    """

    __api__ = [
        "get_state",
        "get_all_state",
    ]
    __pub__ = True
    __req__ = False

    def __init__(
        self,
        device_path: str | None = None,
        vendor_id: int | None = None,
        product_id: int | None = None,
        device_name: str | None = None,
        grab: bool = True,
        n_key_rollover: int = 6,
        dtype=np.float32,
        *,
        freq: int = 100,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        """Initialize the node.

        Args:
            device_path: Explicit device node. Prefer a "/dev/input/by-id/..." symlink
                over "/dev/input/eventN", which is not stable across reboots.
            vendor_id: USB vendor id to match when `device_path` is not given.
            product_id: USB product id to match when `device_path` is not given.
            device_name: Name substring to match when `device_path` is not given.
            grab: Take exclusive access (EVIOCGRAB) so presses do not also reach the
                console. Leave enabled for a dedicated button pad; disable if the
                device is a keyboard the operator also types on.
            n_key_rollover: Number of simultaneously held keys reported.
            dtype: Floating point dtype for the node.
            freq: Publish rate in Hz.
            max_buffer_size: Ring buffer depth.
        """
        self.device_path = device_path
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.device_name = device_name
        self.grab = grab
        self.n_key_rollover = n_key_rollover
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_request = None
        self.example_data = {
            "alphanumeric_state": np.zeros((self.n_key_rollover,), dtype=np.byte),
            "special_state": np.zeros((self.n_key_rollover,), dtype=int),
            "keycode_state": np.zeros((self.n_key_rollover,), dtype=int),
            "press_count": np.zeros((KEY_COUNT,), dtype=np.uint16),
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        device = find_keyboard(
            device_path=self.device_path,
            vendor_id=self.vendor_id,
            product_id=self.product_id,
            device_name=self.device_name,
        )
        if self.verbose:
            print(f"[EvdevKeyboard] reading {device.name} ({device.path})")

        if self.grab:
            try:
                device.grab()
            except OSError as e:
                # Another process already holds the device; keep running ungrabbed
                # rather than failing the whole station.
                print(f"[EvdevKeyboard] could not grab {device.path}: {e}")

        lock = threading.Lock()
        held_keys: set[int] = set()
        press_count = np.zeros((KEY_COUNT,), dtype=np.uint16)

        def drain_events():
            """Apply all pending input events to the held-key set and press counters."""
            while True:
                r, _, _ = select.select([device.fd], [], [], 0)
                if not r:
                    return
                try:
                    events = list(device.read())
                except BlockingIOError:
                    return
                except OSError as e:  # device unplugged
                    print(f"[EvdevKeyboard] device read failed: {e}")
                    self.exit_event.set()
                    return
                with lock:
                    for event in events:
                        if event.type != ecodes.EV_KEY:
                            continue
                        if event.value == 1:  # key down
                            held_keys.add(event.code)
                            if event.code < KEY_COUNT:
                                # Wraps at 2**16; consumers diff modulo the dtype.
                                press_count[event.code] += 1
                        elif event.value == 0:  # key up
                            held_keys.discard(event.code)

        try:
            # Main loop
            rate = time.Rate(self.freq)
            not_pub_ready = True
            while not self.exit_event.is_set():
                drain_events()

                alphanumeric_state = np.zeros((self.n_key_rollover,), dtype=np.byte)
                special_state = np.zeros((self.n_key_rollover,), dtype=int)
                keycode_state = np.zeros((self.n_key_rollover,), dtype=int)
                with lock:
                    n_alpha = 0
                    n_special = 0
                    for i, code in enumerate(sorted(held_keys)):
                        if i < self.n_key_rollover:
                            keycode_state[i] = code
                        char = KEYCODE_TO_CHAR.get(code)
                        if char is not None:
                            if n_alpha < self.n_key_rollover:
                                alphanumeric_state[n_alpha] = ord(char)
                                n_alpha += 1
                        elif n_special < self.n_key_rollover:
                            special_state[n_special] = code
                            n_special += 1
                    current_press_count = press_count.copy()

                # Store current state in ring buffer
                data = {
                    "alphanumeric_state": alphanumeric_state,
                    "special_state": special_state,
                    "keycode_state": keycode_state,
                    "press_count": current_press_count,
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
            if self.grab:
                try:
                    device.ungrab()
                except OSError:
                    pass
            device.close()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()


def _build_keycode_to_char() -> dict[int, str]:
    """Map evdev keycodes for letters and digits to their characters.

    Lets `alphanumeric_state` match the schema published by the pynput-backed
    `Keyboard` node, so existing consumers work against either node.
    """
    mapping = {}
    if ecodes is None:
        return mapping
    for char in "abcdefghijklmnopqrstuvwxyz0123456789":
        code = getattr(ecodes, f"KEY_{char.upper()}", None)
        if code is not None:
            mapping[code] = char
    return mapping


KEYCODE_TO_CHAR = _build_keycode_to_char()


def EvdevKeyboardServer(mw, *args, **kwargs):
    return ServerFactory(mw, EvdevKeyboard, *args, **kwargs)


def EvdevKeyboardClient(mw, *args, **kwargs):
    return ClientFactory(mw, EvdevKeyboard, *args, **kwargs)
