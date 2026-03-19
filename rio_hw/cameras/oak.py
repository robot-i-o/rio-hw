import queue
from typing import TYPE_CHECKING

import cv2
import numpy as np
from threadpoolctl import threadpool_limits

from .. import time
from ..middleware import ClientFactory, ServerFactory

try:
    import depthai as dai
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        dai = None  # type: ignore


def get_connected_cameras():
    """Get all connected OAK cameras.

    Returns:
        Tuple of (serials, models) sorted by serial number.
    """
    serials, models = [], []
    for device in dai.Device.getAllAvailableDevices():
        serials.append(device.getMxId())
        models.append("OAK")
    if len(serials) > 0:
        serials, models = zip(*sorted(zip(serials, models, strict=True)), strict=True)
    return serials, models


class Oak:
    __api__ = [
        "get_state",
        "get_all_state",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        serial: str,
        resolution: tuple[int, int] | None = (800, 1280),
        enable_color: bool = True,
        bgr: bool = False,
        frame_type: str = "isp",
        *,
        freq: int = 60,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        assert frame_type in ("isp", "video")
        self.serial = serial
        self.resolution = resolution
        self.enable_color = enable_color
        self.bgr = bgr
        self.frame_type = frame_type
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_camera_state = {}
        example_camera_state["camera_receive_timestamp"] = 0.0
        example_camera_state["camera_capture_timestamp"] = 0.0
        if self.enable_color:
            h, w = self.resolution
            example_camera_state["color"] = np.zeros(shape=(h, w, 3), dtype=np.uint8)

        self.example_request = {}
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

        # Build DAI pipeline
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setInterleaved(False)
        color_order = dai.ColorCameraProperties.ColorOrder.BGR if self.bgr else dai.ColorCameraProperties.ColorOrder.RGB
        cam_rgb.setColorOrder(color_order)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        cam_rgb.setFps(self.freq)

        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName(self.frame_type)
        if self.frame_type == "isp":
            cam_rgb.isp.link(xout.input)
        elif self.frame_type == "video":
            cam_rgb.video.link(xout.input)

        device_info = dai.DeviceInfo(self.serial)
        device = dai.Device(pipeline, device_info, maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS)

        try:
            output_queue = device.getOutputQueue(name=self.frame_type, maxSize=1, blocking=False)

            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                in_frame = output_queue.get()
                receive_time = time.now()

                frame = in_frame.getCvFrame()
                capture_time = in_frame.getTimestampDevice().total_seconds()

                camera_state = {}
                camera_state["camera_receive_timestamp"] = receive_time
                camera_state["camera_capture_timestamp"] = capture_time
                if self.enable_color:
                    camera_state["color"] = frame

                data = {
                    **camera_state,
                    "timestamp": receive_time,
                }
                self.ring_buffer.put(data, wait=False)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False

                # Drain request queue
                try:
                    self.request_queue.get_all()
                except queue.Empty:
                    pass

                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            device.close()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()


def OakServer(mw, *args, **kwargs):
    return ServerFactory(mw, Oak, *args, **kwargs)


def OakClient(mw, *args, **kwargs):
    return ClientFactory(mw, Oak, *args, **kwargs)
