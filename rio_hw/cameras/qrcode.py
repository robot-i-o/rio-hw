from typing import TYPE_CHECKING

import cv2
import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node

try:
    import qrcode
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        qrcode = None  # type: ignore


def get_connected_cameras():
    return ("qrcode_0",), ("Qrcode",)


def generate_qr(data: str):
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=8,
        border=8,
    )
    qr.add_data(data)
    qr.make(fit=True)
    img = qr.make_image(fill="black", back_color="white")
    img = np.array(img.convert("RGB"))
    return img


class Qrcode(Node):
    __api__ = [
        "get_state",
        "get_all_state",
    ]
    __pub__ = True
    __req__ = False

    def __init__(
        self,
        serial: str = "qrcode_0",
        model: str = "Qrcode",
        resolution: tuple[int, int] = (480, 640),
        enable_color: bool = True,
        enable_depth: bool = False,
        bgr: bool = False,
        *,
        freq: int = 30,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        self.serial = serial
        self.model = model
        self.resolution = resolution
        self.enable_color = enable_color
        self.enable_depth = enable_depth
        self.bgr = bgr
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_camera_state = {}
        example_camera_state["camera_receive_timestamp"] = 0.0
        example_camera_state["camera_capture_timestamp"] = 0.0
        if self.enable_color:
            h, w = self.resolution
            example_camera_state["color"] = np.zeros((h, w, 3), dtype=np.uint8)

        self.example_request = None
        self.example_data = {
            **example_camera_state,
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        try:
            rate = time.Rate(self.freq)
            not_pub_ready = True
            while not self.exit_event.is_set():
                receive_time = time.now()

                camera_state = {}
                camera_state["camera_receive_timestamp"] = receive_time
                camera_state["camera_capture_timestamp"] = receive_time
                if self.enable_color:
                    h, w = self.resolution
                    qr_img = generate_qr(str(receive_time))
                    color = cv2.resize(qr_img, (w, h))
                    if self.bgr:
                        color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
                    camera_state["color"] = color

                data = {
                    **camera_state,
                    "timestamp": receive_time,
                }
                self.ring_buffer.put(data, wait=False)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()


def QrcodeServer(mw, *args, **kwargs):
    return ServerFactory(mw, Qrcode, *args, **kwargs)


def QrcodeClient(mw, *args, **kwargs):
    return ClientFactory(mw, Qrcode, *args, **kwargs)
