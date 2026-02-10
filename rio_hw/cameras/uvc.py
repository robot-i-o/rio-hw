from typing import TYPE_CHECKING

import cv2
import numpy as np
from threadpoolctl import threadpool_limits

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node

try:
    import cv2
    import cv2_enumerate_cameras
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        cv2 = None  # type: ignore
        cv2_enumerate_cameras = None  # type: ignore


def get_connected_cameras():
    serials, models = [], []
    for camera_info in cv2_enumerate_cameras.enumerate_cameras():
        serial = str(camera_info.index)
        model = camera_info.name
        serials.append(serial)
        models.append(model)
    if len(serials) > 0:
        # sort serials and models by serials
        serials, models = zip(*sorted(zip(serials, models, strict=True)), strict=True)
    return serials, models


class Uvc(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "set_default_settings",
    ]
    __pub__ = True
    __req__ = False

    def __init__(
        self,
        serial: str | int,
        model: str,
        resolution: tuple[int, int] | None = (480, 640),
        resolution_depth: tuple[int, int] | None = None,
        enable_color: bool = True,
        enable_depth: bool = False,
        dtype=np.float32,
        *,
        freq: int = 30,
        max_buffer_size: int | None = 30,
        **kwargs,
    ):
        assert not enable_depth, "Depth is not supported"
        self.serial = serial
        self.model = model
        self.resolution = resolution
        self.enable_color = enable_color
        self.enable_depth = enable_depth
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_camera_state = {}
        example_camera_state["camera_receive_timestamp"] = 0.0
        example_camera_state["camera_capture_timestamp"] = 0.0
        if self.enable_color:
            shape = tuple(self.resolution)
            example_camera_state["color"] = np.zeros(shape=(*shape, 3), dtype=np.uint8)

        self.example_request = None
        self.example_data = {
            **example_camera_state,
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        # limit threads
        threadpool_limits(1)
        cv2.setNumThreads(1)

        cap = cv2.VideoCapture(int(self.serial))

        # set resolution and fps
        h, w = self.resolution
        fps = self.freq
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        # set fps
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FPS, fps)

        try:
            # Main loop
            rate = time.Rate(self.freq)
            self.pub_ready_event.set()
            while not self.exit_event.is_set():
                # Wait for new frame to arrive
                ret = cap.grab()
                assert ret
                ret, frame = cap.retrieve()
                assert ret
                receive_time = time.now()
                capture_time = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000

                color_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                camera_state = {}
                camera_state["camera_receive_timestamp"] = receive_time
                camera_state["camera_capture_timestamp"] = capture_time
                if self.enable_color:
                    camera_state["color"] = color_frame

                # Store current state in ring buffer
                data = {
                    **camera_state,
                    "timestamp": receive_time,
                }
                self.ring_buffer.put(data, wait=False)
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            cap.release()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def set_default_settings(self):
        pass


def UvcServer(mw, *args, **kwargs):
    return ServerFactory(mw, Uvc, *args, **kwargs)


def UvcClient(mw, *args, **kwargs):
    return ClientFactory(mw, Uvc, *args, **kwargs)
