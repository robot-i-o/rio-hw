import json
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
    import pyrealsense2 as rs  # type: ignore
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        rs = None  # type: ignore


def get_connected_cameras():
    serials, models = [], []
    for device in rs.context().devices:
        if device.get_info(rs.camera_info.name).lower() == "platform camera":
            continue
        serial = device.get_info(rs.camera_info.serial_number)
        model = device.get_info(rs.camera_info.product_line)
        if model in ("D400", "L500"):
            serials.append(serial)
            models.append(model)
    if len(serials) > 0:
        # sort serials and models by serials
        serials, models = zip(*sorted(zip(serials, models, strict=True)), strict=True)
    return serials, models


def enable_global_time(pipeline_profile):
    device = pipeline_profile.get_device()
    # Try all sensors until we find one that supports global_time. This ensures compatibility with different realsense models.
    for sensor in device.query_sensors():
        if sensor.supports(rs.option.global_time_enabled):
            sensor.set_option(rs.option.global_time_enabled, 1)
            return


class CameraModel(Enum):
    D400 = auto()
    L500 = auto()


class RequestType(Enum):
    SET_COLOR_OPTION = auto()
    SET_DEPTH_OPTION = auto()


class Realsense(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "set_default_settings",
        "set_exposure",
        "set_white_balance",
        "set_brightness",
        "set_contrast",
        "set_depth_preset",
        "set_depth_exposure",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        serial: str,
        model: str,
        resolution: tuple[int, int] | None = (720, 1280),
        resolution_depth: tuple[int, int] | None = (768, 1024),
        enable_color: bool = True,
        enable_depth: bool = False,
        advanced_mode_config: str | None = None,
        timeout_ms: float = 1000.0,
        dtype=np.float32,
        *,
        freq: int = 30,
        max_buffer_size: int | None = 30,
        **kwargs,
    ):
        model = CameraModel[model.upper()]
        self.serial = serial
        self.model = model
        self.resolution = resolution
        self.resolution_depth = resolution_depth
        self.enable_color = enable_color
        self.enable_depth = enable_depth
        self.advanced_mode_config = advanced_mode_config
        self.timeout_ms = timeout_ms
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_request_params = {
            "option_enum": 0,
            "option_value": 0.0,
        }

        example_camera_state = {}
        example_camera_state["camera_receive_timestamp"] = 0.0
        example_camera_state["camera_capture_timestamp"] = 0.0
        if self.enable_color:
            shape = tuple(self.resolution)
            example_camera_state["color"] = np.zeros(shape=(*shape, 3), dtype=np.uint8)
        if self.enable_depth:
            shape = tuple(self.resolution)  # still use color resolution for depth after alignment
            # shape = tuple(self.resolution_depth)
            example_camera_state["depth"] = np.zeros(shape=shape, dtype=np.uint16)
            example_camera_state["depth_units"] = 0.0

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
        # limit threads
        threadpool_limits(1)
        cv2.setNumThreads(1)

        # Reset cameras to ensure they are in a good state before starting streaming.
        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            dev.hardware_reset()
        time.sleep(self.timeout_ms / 1000.0)

        fps = self.freq
        align = rs.align(rs.stream.color)
        # Enable the streams from all the intel realsense devices
        rs_config = rs.config()
        if self.enable_color:
            h, w = self.resolution
            rs_config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        if self.enable_depth:
            h, w = self.resolution_depth
            rs_config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        rs_config.enable_device(self.serial)

        # start pipeline
        pipeline = rs.pipeline()
        pipeline_profile = pipeline.start(rs_config)

        # report global time
        # https://github.com/IntelRealSense/librealsense/pull/3909
        enable_global_time(pipeline_profile)

        # setup advanced mode
        if self.advanced_mode_config is not None:
            advanced_mode_config = json.load(open(self.advanced_mode_config), "r")
            json_text = json.dumps(advanced_mode_config)
            device = pipeline_profile.get_device()
            advanced_mode = rs.rs400_advanced_mode(device)
            advanced_mode.load_json(json_text)

        depth_units = 0.0
        if self.enable_depth:
            depth_sensor = pipeline_profile.get_device().first_depth_sensor()
            depth_units = depth_sensor.get_depth_scale()

        try:
            # Main loop
            rate = time.Rate(self.freq)
            self.pub_ready_event.set()
            while not self.exit_event.is_set():
                try:
                    frameset = pipeline.wait_for_frames(timeout_ms=self.timeout_ms)
                except RuntimeError:
                    print("Frame timeout - camera may have disconnected")
                    continue

                receive_time = time.now()
                # align frames to color
                frameset = align.process(frameset)

                # grab data
                camera_state = {}
                camera_state["camera_receive_timestamp"] = receive_time
                # realsense report in ms
                camera_state["camera_capture_timestamp"] = frameset.get_timestamp() / 1000
                # NOTE: need np.copy to stream in threaded mode: https://github.com/IntelRealSense/realsense-ros/issues/2460
                if self.enable_color:
                    color_frame = frameset.get_color_frame()
                    camera_state["color"] = np.copy(np.asarray(color_frame.get_data()))
                    t = color_frame.get_timestamp() / 1000
                    camera_state["camera_capture_timestamp"] = t
                if self.enable_depth:
                    camera_state["depth"] = np.copy(np.asarray(frameset.get_depth_frame().get_data()))
                    camera_state["depth_units"] = depth_units

                # Store current state in ring buffer
                data = {
                    **camera_state,
                    "timestamp": receive_time,
                }
                self.ring_buffer.put(data, wait=False)

                # Fetch requests from queue
                try:
                    reqs = self.request_queue.get_all()
                    if isinstance(reqs, dict):
                        reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
                except queue.Empty:
                    reqs = []
                for r in reqs:
                    req = Request(RequestType(r.pop("type")), r)
                    if req.type == RequestType.SET_COLOR_OPTION:
                        sensor = pipeline_profile.get_device().first_color_sensor()
                        option = rs.option(req.params.get("option_enum"))
                        value = float(req.params.get("option_value"))
                        sensor.set_option(option, value)
                    elif req.type == RequestType.SET_DEPTH_OPTION:
                        sensor = pipeline_profile.get_device().first_depth_sensor()
                        option = rs.option(req.params.get("option_enum"))
                        value = float(req.params.get("option_value"))
                        sensor.set_option(option, value)
                    else:
                        raise ValueError(req.type)
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            rs_config.disable_all_streams()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def _set_color_option(self, option, value: float):
        req = {
            "type": RequestType.SET_COLOR_OPTION.value,
            "option_enum": option.value,
            "option_value": value,
        }
        self.request_queue.put(req)

    def _set_depth_option(self, option, value: float):
        req = {
            "type": RequestType.SET_DEPTH_OPTION.value,
            "option_enum": option.value,
            "option_value": value,
        }
        self.request_queue.put(req)

    def set_default_settings(self):
        raise NotImplementedError

    def set_exposure(self, exposure=None, gain=None):
        if exposure is None and gain is None:
            # auto exposure
            self._set_color_option(rs.option.enable_auto_exposure, 1.0)
        else:
            # manual exposure
            self._set_color_option(rs.option.enable_auto_exposure, 0.0)
            if exposure is not None:
                self._set_color_option(rs.option.exposure, exposure)
            if gain is not None:
                self._set_color_option(rs.option.gain, gain)

    def set_white_balance(self, white_balance=None):
        if white_balance is None:
            # auto white balance
            self._set_color_option(rs.option.enable_auto_white_balance, 1.0)
        else:
            # manual white balance
            self._set_color_option(rs.option.enable_auto_white_balance, 0.0)
            self._set_color_option(rs.option.white_balance, white_balance)

    def set_brightness(self, brightness=None):
        if brightness is None:
            self._set_color_option(rs.option.brightness, 0.0)
        else:
            self._set_color_option(rs.option.brightness, brightness)

    def set_contrast(self, contrast=None):
        if contrast is None:
            self._set_color_option(rs.option.contrast, 0)
        else:
            self._set_color_option(rs.option.contrast, contrast)

    def set_depth_preset(self, preset=None):
        visual_preset = {
            "Custom": 0,
            "Default": 1,
            "Hand": 2,
            "High Accuracy": 3,
            "High Density": 4,
        }
        if preset is None:
            self._set_depth_option(rs.option.visual_preset, visual_preset["Default"])
        else:
            self._set_depth_option(rs.option.visual_preset, visual_preset[preset])

    def set_depth_exposure(self, exposure=None, gain=None):
        if exposure is None and gain is None:
            # auto exposure
            self._set_depth_option(rs.option.enable_auto_exposure, 1.0)
        else:
            # manual exposure
            self._set_depth_option(rs.option.enable_auto_exposure, 0.0)
            if exposure is not None:
                self._set_depth_option(rs.option.exposure, exposure)
            if gain is not None:
                self._set_depth_option(rs.option.gain, gain)


def RealsenseServer(mw, *args, **kwargs):
    return ServerFactory(mw, Realsense, *args, **kwargs)


def RealsenseClient(mw, *args, **kwargs):
    return ClientFactory(mw, Realsense, *args, **kwargs)
