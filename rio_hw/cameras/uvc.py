import platform
import queue
from enum import Enum, auto
from pathlib import Path

import cv2
import numpy as np
from threadpoolctl import threadpool_limits

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request

MAX_OPENCV_INDEX = 60


def get_connected_cameras():
    serials, models = [], []
    targets = (
        [str(p) for p in sorted(Path("/dev").glob("video*"), key=lambda p: p.name)]
        if platform.system() == "Linux"
        else list(range(MAX_OPENCV_INDEX))
    )
    for target in targets:
        cap = cv2.VideoCapture(target)
        if cap.isOpened():
            try:
                model = cap.getBackendName()
            except Exception:
                model = "unknown"
            serials.append(str(target))
            models.append(model or "unknown")
            cap.release()
    if len(serials) > 0:
        serials, models = zip(*sorted(zip(serials, models, strict=True)), strict=True)
    return serials, models


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
        resolution: tuple[int, int] | None = (480, 640),
        resolution_depth: tuple[int, int] | None = None,
        enable_color: bool = True,
        enable_depth: bool = False,
        fourcc: str | None = None,
        bgr: bool = False,
        warmup_s: float = 2.0,
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
        self.fourcc = fourcc
        self.bgr = bgr
        self.warmup_s = warmup_s
        self.dtype = dtype

        self.capture = None
        self.backend = cv2.CAP_ANY

        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        h, w = self.resolution if self.resolution else (480, 640)

        example_camera_state = {}
        example_camera_state["camera_receive_timestamp"] = 0.0
        example_camera_state["camera_capture_timestamp"] = 0.0
        if self.enable_color:
            example_camera_state["color"] = np.zeros((h, w, 3), dtype=np.uint8)

        self.example_request = {
            "type": RequestType.SET_PROPERTY.value,
            "property_id": 0,
            "value": 0.0,
        }
        self.example_data = {
            **example_camera_state,
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pubreq
        super().__post_init__()

    @property
    def is_connected(self) -> bool:
        return isinstance(self.capture, cv2.VideoCapture) and self.capture.isOpened()

    def pubreq(self):
        threadpool_limits(1)
        cv2.setNumThreads(1)

        self.capture = cv2.VideoCapture(int(self.serial), self.backend)
        if not self.capture.isOpened():
            raise ConnectionError(f"Failed to open camera {self.serial}")
        self._configure_settings()

        # Warmup: read frames for warmup_s seconds
        warmup_start = time.now()
        while time.now() - warmup_start < self.warmup_s:
            self.capture.read()

        try:
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                ret, frame = self.capture.read()

                if not ret:
                    print(f"Warning: failed to read frame from camera {self.serial}")
                    continue

                receive_time = time.now()

                camera_state = {}
                camera_state["camera_receive_timestamp"] = receive_time
                camera_state["camera_capture_timestamp"] = receive_time
                if self.enable_color:
                    camera_state["color"] = frame if self.bgr else cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

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
                        self.capture.set(prop_id, value)
                    else:
                        raise ValueError(req.type)
                rate.precise_sleep()
        finally:
            if self.capture is not None:
                self.capture.release()
                self.capture = None

    def _configure_settings(self):
        """Configure camera settings after connection."""
        if not self.is_connected:
            raise RuntimeError("Camera not connected")

        if self.fourcc is not None:
            fourcc_code = cv2.VideoWriter_fourcc(*self.fourcc)
            success = self.capture.set(cv2.CAP_PROP_FOURCC, fourcc_code)
            if not success:
                print(f"Warning: failed to set FOURCC to {self.fourcc}")

        if self.resolution is not None:
            h, w = self.resolution
            width_ok = self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            height_ok = self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

            actual_w = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

            if not width_ok or actual_w != w:
                raise RuntimeError(f"Failed to set width={w} (actual={actual_w})")
            if not height_ok or actual_h != h:
                raise RuntimeError(f"Failed to set height={h} (actual={actual_h})")

        fps_ok = self.capture.set(cv2.CAP_PROP_FPS, float(self.freq))
        actual_fps = self.capture.get(cv2.CAP_PROP_FPS)
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
