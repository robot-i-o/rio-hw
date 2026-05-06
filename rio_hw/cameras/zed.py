import queue
import sys
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
    import pyzed.sl as sl
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        sl = None  # type: ignore


def get_connected_cameras():
    serials, models = [], []
    cameras = sl.Camera.get_device_list()
    for cam in cameras:
        serial = str(cam.serial_number)
        model = cam.camera_model.name
        serials.append(serial)
        models.append(model)
    if len(serials) > 0:
        # sort serials and models by serials
        serials, models = zip(*sorted(zip(serials, models, strict=True)), strict=True)
    return serials, models


CameraModel = sl.MODEL if sl is not None else None


class RequestType(Enum):
    SET_CAMERA_SETTINGS = auto()


class Zed(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "set_default_settings",
        "set_exposure",
        "set_white_balance",
        "set_brightness",
        "set_contrast",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        serial: int | str,
        model: str,
        resolution: tuple[int, int] | None = (720, 1280),
        resolution_depth: tuple[int, int] | None = None,
        enable_color: bool = True,
        enable_depth: bool = False,
        bgr: bool = False,
        image_side: str | None = "LEFT",
        concatenate_images: bool = False,
        depth_mode: str = "NEURAL",
        depth_stabilization: int = 0,
        depth_minimum_distance: float = 0.1,
        image_flip: str = "OFF",
        dtype=np.float32,
        *,
        freq: int = 30,
        max_buffer_size: int | None = 30,
        **kwargs,
    ):
        self.serial = serial
        self.model = model
        self.resolution = resolution
        self.enable_color = enable_color
        self.enable_depth = enable_depth
        self.bgr = bgr
        self.image_side = image_side
        self.concatenate_images = concatenate_images
        self.depth_mode = depth_mode
        self.depth_stabilization = depth_stabilization
        self.depth_minimum_distance = depth_minimum_distance
        self.image_flip = image_flip
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_request_params = {
            "setting_enum": 0,
            "setting_value": 0,
        }

        example_camera_state = {}
        example_camera_state["camera_receive_timestamp"] = 0.0
        example_camera_state["camera_capture_timestamp"] = 0.0
        if self.enable_color:
            h, w = self.resolution
            if self.concatenate_images:
                # Side-by-side image
                example_camera_state["color"] = np.zeros(shape=(h, w * 2, 3), dtype=np.uint8)
            elif self.image_side is not None:
                assert self.image_side.lower() in ("left", "right")
                example_camera_state["color"] = np.zeros(shape=(h, w, 3), dtype=np.uint8)
            else:
                # Separate left and right images
                example_camera_state["color_left"] = np.zeros(shape=(h, w, 3), dtype=np.uint8)
                example_camera_state["color_right"] = np.zeros(shape=(h, w, 3), dtype=np.uint8)
        if self.enable_depth:
            h, w = self.resolution
            example_camera_state["depth"] = np.zeros(shape=(h, w), dtype=np.float32)

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

        fps = self.freq
        h, w = self.resolution

        # Initialize ZED camera
        cam = sl.Camera()
        init_params = sl.InitParameters()

        # Set resolution
        resolution_map = {
            (2208, 1242): sl.RESOLUTION.HD2K,
            (1920, 1200): sl.RESOLUTION.HD1200,
            (1920, 1080): sl.RESOLUTION.HD1080,
            (1280, 720): sl.RESOLUTION.HD720,
            (960, 600): sl.RESOLUTION.SVGA,
            (672, 376): sl.RESOLUTION.VGA,
        }
        init_params.camera_resolution = resolution_map.get((w, h), sl.RESOLUTION.HD720)

        # Set FPS
        assert fps in (15, 30, 60, 100, 120)
        init_params.camera_fps = fps
        init_params.grab_compute_capping_fps = self.freq  # cap the grab compute to ensure stable FPS

        # Set depth parameters
        depth_mode_map = {
            "NONE": sl.DEPTH_MODE.NONE,
            "PERFORMANCE": sl.DEPTH_MODE.PERFORMANCE,
            "QUALITY": sl.DEPTH_MODE.QUALITY,
            "ULTRA": sl.DEPTH_MODE.ULTRA,
            "NEURAL": sl.DEPTH_MODE.NEURAL,
        }
        init_params.depth_mode = depth_mode_map.get(self.depth_mode, sl.DEPTH_MODE.NEURAL)
        if not self.enable_depth:
            init_params.depth_mode = sl.DEPTH_MODE.NONE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_minimum_distance = self.depth_minimum_distance
        init_params.depth_stabilization = self.depth_stabilization

        # Set image flip
        flip_mode_map = {
            "OFF": sl.FLIP_MODE.OFF,
            "ON": sl.FLIP_MODE.ON,
            "AUTO": sl.FLIP_MODE.AUTO,
        }
        init_params.camera_image_flip = flip_mode_map.get(self.image_flip, sl.FLIP_MODE.OFF)

        # Set serial number
        init_params.set_from_serial_number(int(self.serial))

        # Open camera
        err = cam.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(err)

        # Create runtime parameters
        runtime_params = sl.RuntimeParameters()

        # Create image containers
        if self.concatenate_images:
            sbs_img = sl.Mat()
        else:
            left_img = sl.Mat()
            right_img = sl.Mat()

        if self.enable_depth:
            left_depth = sl.Mat()
            right_depth = sl.Mat()

        # Default resolution for retrieval
        zed_resolution = sl.Resolution(0, 0)
        sys.stdout.flush()
        try:
            # Main loop
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                # Grab frame
                code = cam.grab(runtime_params)
                if code <= sl.ERROR_CODE.SUCCESS:
                    receive_time = time.now()

                    # Get capture timestamp (in milliseconds)
                    capture_timestamp = cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds() / 1000.0

                    # Prepare camera state
                    camera_state = {}
                    camera_state["camera_receive_timestamp"] = receive_time
                    camera_state["camera_capture_timestamp"] = capture_timestamp

                    # Retrieve color images
                    if self.enable_color:
                        if self.concatenate_images:
                            cam.retrieve_image(sbs_img, sl.VIEW.SIDE_BY_SIDE, resolution=zed_resolution)
                            color = sbs_img.get_data()
                            code = cv2.COLOR_BGRA2BGR if self.bgr else cv2.COLOR_BGR2RGB
                            color = cv2.cvtColor(color, code)
                            camera_state["color"] = color
                        elif self.image_side is not None:
                            cam.retrieve_image(left_img, sl.VIEW[self.image_side.upper()], resolution=zed_resolution)
                            color = left_img.get_data()
                            code = cv2.COLOR_BGRA2BGR if self.bgr else cv2.COLOR_BGR2RGB
                            color = cv2.cvtColor(color, code)
                            camera_state["color"] = color
                        else:
                            cam.retrieve_image(left_img, sl.VIEW.LEFT, resolution=zed_resolution)
                            cam.retrieve_image(right_img, sl.VIEW.RIGHT, resolution=zed_resolution)
                            color_left, color_right = left_img.get_data(), right_img.get_data()
                            code = cv2.COLOR_BGRA2BGR if self.bgr else cv2.COLOR_BGR2RGB
                            color_left = cv2.cvtColor(color_left, code)
                            color_right = cv2.cvtColor(color_right, code)
                            camera_state["color_left"] = color_left
                            camera_state["color_right"] = color_right

                    # Retrieve depth map
                    if self.enable_depth:
                        if self.concatenate_images:
                            raise NotImplementedError
                        elif self.image_side is not None:
                            cam.retrieve_measure(left_depth, sl.MEASURE.DEPTH, resolution=zed_resolution)
                            camera_state["depth"] = left_depth.get_data().copy()
                        else:
                            cam.retrieve_measure(left_depth, sl.MEASURE.DEPTH, resolution=zed_resolution)
                            cam.retrieve_measure(right_depth, sl.MEASURE.DEPTH, resolution=zed_resolution)
                            camera_state["depth_left"] = left_depth.get_data().copy()
                            camera_state["depth_right"] = right_depth.get_data().copy()

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
                    if req.type == RequestType.SET_CAMERA_SETTINGS:
                        setting = sl.VIDEO_SETTINGS(req.params.get("setting_enum"))
                        value = int(req.params.get("setting_value"))
                        cam.set_camera_settings(setting, value)
                    else:
                        raise ValueError(req.type)
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            cam.close()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def _set_camera_setting(self, setting, value: int):
        req = {
            "type": RequestType.SET_CAMERA_SETTINGS.value,
            "setting_enum": setting.value,
            "setting_value": value,
        }
        self.request_queue.put(req)

    def set_default_settings(self):
        self._set_camera_setting(sl.VIDEO_SETTINGS.BRIGHTNESS, -1)
        self._set_camera_setting(sl.VIDEO_SETTINGS.CONTRAST, -1)
        self._set_camera_setting(sl.VIDEO_SETTINGS.HUE, -1)
        self._set_camera_setting(sl.VIDEO_SETTINGS.SATURATION, -1)
        self._set_camera_setting(sl.VIDEO_SETTINGS.SHARPNESS, -1)
        self._set_camera_setting(sl.VIDEO_SETTINGS.GAIN, -1)
        self._set_camera_setting(sl.VIDEO_SETTINGS.EXPOSURE, -1)
        self._set_camera_setting(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, -1)

    def set_exposure(self, exposure=None, gain=None):
        if exposure is None and gain is None:
            # auto exposure
            self._set_camera_setting(sl.VIDEO_SETTINGS.AEC_AGC, 1)
        else:
            # manual exposure
            self._set_camera_setting(sl.VIDEO_SETTINGS.AEC_AGC, 0)
            if exposure is not None:
                self._set_camera_setting(sl.VIDEO_SETTINGS.EXPOSURE, exposure)
            if gain is not None:
                self._set_camera_setting(sl.VIDEO_SETTINGS.GAIN, gain)

    def set_white_balance(self, white_balance=None):
        if white_balance is None:
            # auto white balance
            self._set_camera_setting(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 1)
        else:
            # manual white balance
            self._set_camera_setting(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 0)
            self._set_camera_setting(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, white_balance)

    def set_brightness(self, brightness: int):
        self._set_camera_setting(sl.VIDEO_SETTINGS.BRIGHTNESS, brightness)

    def set_contrast(self, contrast: int):
        self._set_camera_setting(sl.VIDEO_SETTINGS.CONTRAST, contrast)


def ZedServer(mw, *args, **kwargs):
    return ServerFactory(mw, Zed, *args, **kwargs)


def ZedClient(mw, *args, **kwargs):
    return ClientFactory(mw, Zed, *args, **kwargs)
