import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import cv2
import numpy as np
from threadpoolctl import threadpool_limits

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request

try:
    import cv2_enumerate_cameras
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
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


COMMON_RESOLUTIONS = [
    (240, 320),
    (480, 640),
    (600, 800),
    (768, 1024),
    (720, 1280),
    (960, 1280),
    (1080, 1920),
    (1440, 2560),
    (2160, 3840),
]


def _probe_on_capture(capture, warmup_s=0.1):
    """Probe available resolutions on an already-open capture."""
    found = []
    for h, w in COMMON_RESOLUTIONS:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        warmup_start = time.now()
        while time.now() - warmup_start < warmup_s:
            capture.read()
        ret, frame = capture.read()
        if ret:
            actual_h, actual_w = frame.shape[:2]
            if (actual_h, actual_w) not in found:
                found.append((actual_h, actual_w))
    return sorted(found)


class RequestType(Enum):
    SET_PROPERTY = auto()


class Uvc(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "set_exposure",
        "set_brightness",
        "set_contrast",
        "set_saturation",
        "set_auto_exposure",
        "set_auto_white_balance",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        serial: str | int,
        model: str,
        resolution: tuple[int, int] = (480, 640),
        resolution_depth: tuple[int, int] | None = None,
        enable_color: bool = True,
        enable_depth: bool = False,
        bgr: bool = False,
        warmup_s: float = 0.5,
        backend: str = "CAP_ANY",
        fourcc: str | None = None,
        dtype=np.float32,
        *,
        freq: int = 30,
        max_buffer_size: int | None = 30,
        **kwargs,
    ):
        assert enable_color and not enable_depth, "Depth is not supported"
        self.serial = serial
        self.model = model
        self.resolution = resolution
        self.enable_color = enable_color
        self.enable_depth = enable_depth
        self.bgr = bgr
        self.warmup_s = warmup_s
        self.backend = backend
        self.fourcc = fourcc
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_camera_state = {}
        example_camera_state["camera_receive_timestamp"] = 0.0
        example_camera_state["camera_capture_timestamp"] = 0.0
        if self.enable_color:
            h, w = self.resolution
            example_camera_state["color"] = np.zeros((h, w, 3), dtype=np.uint8)

        example_request_params = {
            "property_id": 0,
            "value": 0.0,
        }

        self.example_request = {
            "type": next(iter(RequestType)).value,
            **example_request_params,
        }
        self.example_data = {
            **example_camera_state,
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pubreq
        super().__post_init__()

    def pubreq(self):
        threadpool_limits(1)
        cv2.setNumThreads(1)

        backend = getattr(cv2, self.backend)
        capture = cv2.VideoCapture(int(self.serial), backend)
        if not capture.isOpened():
            raise ConnectionError(f"Failed to open camera {self.serial}")
        self._configure_settings(capture)

        # Read frames for warmup_s seconds
        warmup_start = time.now()
        while time.now() - warmup_start < self.warmup_s:
            capture.read()

        # Verify resolution after warmup
        ret, frame = capture.read()
        if not ret:
            raise RuntimeError(f"Failed to read frame from camera {self.serial}")
        actual_h, actual_w = frame.shape[:2]
        h, w = self.resolution
        if actual_w != w or actual_h != h:
            available = _probe_on_capture(capture, self.warmup_s)
            raise RuntimeError(f"Failed to set resolution=({h}, {w}), got ({actual_h}, {actual_w}). Available: {available}")

        try:
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                ret, frame = capture.read()

                if not ret:
                    print(f"Warning: failed to read frame from camera {self.serial}")
                    continue

                receive_time = time.now()

                camera_state = {}
                camera_state["camera_receive_timestamp"] = receive_time
                camera_state["camera_capture_timestamp"] = receive_time
                if self.enable_color:
                    camera_state["color"] = frame if self.bgr else cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # Store current state in ring buffer
                data = {
                    **camera_state,
                    "timestamp": receive_time,
                }
                self.ring_buffer.put(data, wait=False)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False

                # Fetch requests from queue
                try:
                    reqs = self.request_queue.get_all()
                    if isinstance(reqs, dict):
                        reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
                except queue.Empty:
                    reqs = []
                for r in reqs:
                    req = Request(RequestType(r.pop("type")), r)
                    if req.type == RequestType.SET_PROPERTY:
                        prop_id = int(req.params["property_id"])
                        value = float(req.params["value"])
                        capture.set(prop_id, value)
                    else:
                        raise ValueError(req.type)
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            capture.release()

    def _configure_settings(self, capture):
        if self.fourcc is not None:
            fourcc_code = cv2.VideoWriter_fourcc(*self.fourcc)
            success = capture.set(cv2.CAP_PROP_FOURCC, fourcc_code)
            if not success:
                print(f"Warning: failed to set FOURCC to {self.fourcc}")

        h, w = self.resolution
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

        fps_ok = capture.set(cv2.CAP_PROP_FPS, float(self.freq))
        actual_fps = capture.get(cv2.CAP_PROP_FPS)
        if not fps_ok and abs(self.freq - actual_fps) > 0.1:
            print(f"Warning: failed to set FPS={self.freq} (actual={actual_fps})")

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def _set_property(self, prop_id, value: float):
        req = {
            "type": RequestType.SET_PROPERTY.value,
            "property_id": prop_id,
            "value": value,
        }
        self.request_queue.put(req)

    def set_exposure(self, exposure=None):
        if exposure is None:
            self._set_property(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0)
        else:
            self._set_property(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
            self._set_property(cv2.CAP_PROP_EXPOSURE, exposure)

    def set_brightness(self, brightness):
        self._set_property(cv2.CAP_PROP_BRIGHTNESS, brightness)

    def set_contrast(self, contrast):
        self._set_property(cv2.CAP_PROP_CONTRAST, contrast)

    def set_saturation(self, saturation):
        self._set_property(cv2.CAP_PROP_SATURATION, saturation)

    def set_auto_exposure(self, enabled: bool):
        self._set_property(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0 if enabled else 1.0)

    def set_auto_white_balance(self, enabled: bool):
        self._set_property(cv2.CAP_PROP_AUTO_WB, 1.0 if enabled else 0.0)


def UvcServer(mw, *args, **kwargs):
    return ServerFactory(mw, Uvc, *args, **kwargs)


def UvcClient(mw, *args, **kwargs):
    return ClientFactory(mw, Uvc, *args, **kwargs)
