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
    import pyorbbecsdk as ob
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        ob = None  # type: ignore


def get_connected_cameras():
    context = ob.Context()
    device_list = context.query_devices()
    serials, models = [], []
    for i in range(device_list.get_count()):
        serial = device_list.get_device_serial_number_by_index(i)
        device = device_list.get_device_by_index(i)
        model = device.get_device_info().get_name()
        serials.append(serial)
        models.append(model)
    if len(serials) > 0:
        serials, models = zip(*sorted(zip(serials, models, strict=True)), strict=True)
    return serials, models


class CameraModel(Enum):
    FEMTO_BOLT = auto()
    FEMTO_MEGA = auto()


class RequestType(Enum):
    SET_COLOR_OPTION = auto()
    SET_DEPTH_OPTION = auto()


class Orbbec(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "set_exposure",
        "set_depth_exposure",
        "set_white_balance",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        serial: str,
        camera_model: str = "femto_bolt",
        resolution: tuple[int, int] = (960, 1280),
        resolution_depth: tuple[int, int] | None = None,
        enable_color: bool = True,
        enable_depth: bool = False,
        enable_ir: bool = False,
        bgr: bool = False,
        min_depth: float = 0.02,  # meters
        max_depth: float = 10.0,  # meters
        timeout_ms: float = 1000.0,
        dtype=np.float32,
        *,
        freq: int = 30,
        max_buffer_size: int = 30,
        **kwargs,
    ):
        camera_model = CameraModel[camera_model.upper()]
        self.serial = serial
        self.camera_model = camera_model
        self.resolution = resolution
        self.resolution_depth = resolution_depth
        self.enable_color = enable_color
        self.enable_depth = enable_depth
        self.enable_ir = enable_ir
        self.bgr = bgr
        self.min_depth = min_depth
        self.max_depth = max_depth
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
            shape = tuple(self.resolution)  # use color resolution after alignment
            example_camera_state["depth"] = np.zeros(shape=shape, dtype=self.dtype)
            example_camera_state["depth_scale"] = 0.0
        if self.enable_ir:
            shape = tuple(self.resolution)
            example_camera_state["ir"] = np.zeros(shape=shape, dtype=np.uint8)

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

        fps = self.freq

        # Get device by serial
        context = ob.Context()
        device_list = context.query_devices()
        device = device_list.get_device_by_serial_number(self.serial)
        pipeline = ob.Pipeline(device)
        config = ob.Config()

        # Configure color stream
        if self.enable_color:
            h, w = self.resolution
            color_profile_list = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
            color_profile = color_profile_list.get_video_stream_profile(w, h, ob.OBFormat.BGRA, fps)
            config.enable_stream(color_profile)

        # Configure depth stream
        if self.enable_depth:
            depth_profile_list = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
            if self.resolution_depth is not None:
                dh, dw = self.resolution_depth
                depth_profile = depth_profile_list.get_video_stream_profile(dw, dh, ob.OBFormat.Y16, fps)
            else:
                depth_profile = depth_profile_list.get_default_video_stream_profile()
            config.enable_stream(depth_profile)

        # Configure infrared stream
        if self.enable_ir:
            ir_profile_list = pipeline.get_stream_profile_list(ob.OBSensorType.IR_SENSOR)
            ir_profile = ir_profile_list.get_default_video_stream_profile()
            config.enable_stream(ir_profile)

        # Setup alignment if both color and depth are enabled
        align_filter = None
        if self.enable_depth and self.enable_color:
            config.set_frame_aggregate_output_mode(ob.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
            align_filter = ob.AlignFilter(ob.OBStreamType.COLOR_STREAM)

        try:
            pipeline.start(config)

            # Warm-up frames
            for _ in range(30):
                pipeline.wait_for_frames(int(self.timeout_ms))

            # Main loop
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                try:
                    frameset = pipeline.wait_for_frames(int(self.timeout_ms))
                except Exception:
                    print("Frame timeout - camera may have disconnected")
                    continue
                if frameset is None:
                    continue

                receive_time = time.now()

                # Apply alignment filter
                if align_filter is not None:
                    frameset = align_filter.process(frameset).as_frame_set()

                # Grab data
                camera_state = {}
                camera_state["camera_receive_timestamp"] = receive_time
                camera_state["camera_capture_timestamp"] = frameset.get_timestamp() / 1000

                if self.enable_color:
                    color_frame = frameset.get_color_frame()
                    if color_frame is not None:
                        color_data = np.copy(np.asarray(color_frame.get_data()))
                        color_image = color_data.reshape((color_frame.get_height(), color_frame.get_width(), 4))[..., :3]
                        if not self.bgr:
                            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                        camera_state["color"] = color_image
                        camera_state["camera_capture_timestamp"] = color_frame.get_timestamp() / 1000

                if self.enable_depth:
                    depth_frame = frameset.get_depth_frame()
                    if depth_frame is not None:
                        width = depth_frame.get_width()
                        height = depth_frame.get_height()
                        depth_scale = depth_frame.get_depth_scale() * 0.001  # mm → meters
                        depth_data = np.copy(np.frombuffer(depth_frame.get_data(), dtype=np.uint16))
                        depth_data = depth_data.reshape((height, width))
                        depth_data = depth_data.astype(self.dtype) * depth_scale
                        depth_data = np.where((depth_data > self.min_depth) & (depth_data < self.max_depth), depth_data, 0)
                        camera_state["depth"] = depth_data
                        camera_state["depth_scale"] = depth_scale

                if self.enable_ir:
                    ir_frame = frameset.get_ir_frame()
                    if ir_frame is not None:
                        camera_state["ir"] = np.copy(np.asarray(ir_frame.get_data()))

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
                    option = ob.OBPropertyID(req.params.get("option_enum"))
                    option_name = option.name
                    is_bool = option_name.endswith("BOOL")
                    if req.type in (RequestType.SET_COLOR_OPTION, RequestType.SET_DEPTH_OPTION):
                        if is_bool:
                            device.set_bool_property(option, bool(req.params.get("option_value")))
                        else:
                            device.set_int_property(option, int(req.params.get("option_value")))
                    else:
                        raise ValueError(req.type)
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            pipeline.stop()

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

    def set_exposure(self, exposure=None, gain=None):
        if exposure is None and gain is None:
            self._set_color_option(ob.OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, 1.0)
        else:
            self._set_color_option(ob.OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, 0.0)
            if exposure is not None:
                self._set_color_option(ob.OBPropertyID.OB_PROP_COLOR_EXPOSURE_INT, exposure)
            if gain is not None:
                self._set_color_option(ob.OBPropertyID.OB_PROP_COLOR_GAIN_INT, gain)

    def set_depth_exposure(self, exposure=None, gain=None):
        if exposure is None and gain is None:
            self._set_depth_option(ob.OBPropertyID.OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, 1.0)
        else:
            self._set_depth_option(ob.OBPropertyID.OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, 0.0)
            if exposure is not None:
                self._set_depth_option(ob.OBPropertyID.OB_PROP_DEPTH_EXPOSURE_INT, exposure)
            if gain is not None:
                self._set_depth_option(ob.OBPropertyID.OB_PROP_DEPTH_GAIN_INT, gain)

    def set_white_balance(self, white_balance=None):
        if white_balance is None:
            self._set_color_option(ob.OBPropertyID.OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, 1.0)
        else:
            self._set_color_option(ob.OBPropertyID.OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, 0.0)
            self._set_color_option(ob.OBPropertyID.OB_PROP_COLOR_WHITE_BALANCE_INT, white_balance)


def OrbbecServer(mw, *args, **kwargs):
    return ServerFactory(mw, Orbbec, *args, **kwargs)


def OrbbecClient(mw, *args, **kwargs):
    return ClientFactory(mw, Orbbec, *args, **kwargs)
