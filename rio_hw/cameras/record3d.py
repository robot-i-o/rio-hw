import threading as th
from enum import Enum
from typing import TYPE_CHECKING

import cv2
import numpy as np
import scipy.spatial.transform as st
from threadpoolctl import threadpool_limits

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node

try:
    import record3d
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        record3d = None  # type: ignore


def get_connected_cameras():
    serials, models = [], []
    devs = record3d.Record3DStream.get_connected_devices()
    for dev in devs:
        serial = dev.udid
        model = dev.product_id
        serials.append(serial)
        models.append(model)
    if len(serials) > 0:
        # sort serials and models by serials
        serials, models = zip(*sorted(zip(serials, models, strict=True)), strict=True)
    return serials, models


class DeviceType(Enum):
    TRUEDEPTH = 0
    LIDAR = 1


class Record3d(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "set_default_settings",
    ]
    __pub__ = True
    __req__ = False

    def __init__(
        self,
        serial: str,
        model: str,
        resolution: tuple[int, int] | None = (1440, 1920),
        resolution_depth: tuple[int, int] | None = None,
        enable_color: bool = True,
        enable_depth: bool = True,
        enable_intrinsics: bool = False,
        enable_extrinsics: bool = False,
        dtype=np.float32,
        *,
        freq: int = 30,
        max_buffer_size: int | None = 30,
        **kwargs,
    ):
        assert resolution in ((720, 960), (1440, 1920))
        self.serial = serial
        self.model = model
        self.resolution = resolution
        self.enable_color = enable_color
        self.enable_depth = enable_depth
        self.enable_intrinsics = enable_intrinsics
        self.enable_extrinsics = enable_extrinsics
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_camera_state = {}
        example_camera_state["camera_receive_timestamp"] = 0.0
        example_camera_state["camera_capture_timestamp"] = 0.0
        if self.enable_color:
            shape = tuple(self.resolution)
            example_camera_state["color"] = np.zeros(shape=(*shape, 3), dtype=np.uint8)
        if self.enable_depth:
            shape = tuple(self.resolution)
            example_camera_state["depth"] = np.zeros(shape=(*shape, 1), dtype=np.float32)
        if self.enable_intrinsics:
            example_camera_state["intrinsics"] = np.zeros((3, 3), dtype=self.dtype)
        if self.enable_extrinsics:
            # 4x4 extrinsic matrix (camera pose in world coordinates)
            example_camera_state["extrinsics"] = np.eye(4, dtype=self.dtype)

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

        # Find device
        dev_idx = -1
        devs = record3d.Record3DStream.get_connected_devices()
        for i, dev in enumerate(devs):
            if dev.udid == self.serial and dev.product_id == self.model:
                dev_idx = i
                break
        if dev_idx == -1:
            raise RuntimeError
        dev = devs[dev_idx]

        frame_event = th.Event()
        stop_event = th.Event()
        DEVICE_TYPE__TRUEDEPTH = 0
        DEVICE_TYPE__LIDAR = 1

        def on_new_frame():
            frame_event.set()

        def on_stream_stopped():
            stop_event.set()

        def get_intrinsic_mat_from_coeffs(self, coeffs):
            mat = [
                [coeffs.fx, 0.0, coeffs.tx],
                [0.0, coeffs.fy, coeffs.ty],
                [0.0, 0.0, 1.0],
            ]
            return np.array(mat, dtype=self.dtype)

        dev = devs[dev_idx]
        session = record3d.Record3DStream()
        session.on_new_frame = on_new_frame
        session.on_stream_stopped = on_stream_stopped
        session.connect(dev)  # Initiate connection and start capturing
        device_type = session.get_device_type()

        try:
            # Main loop
            rate = time.Rate(self.freq)
            not_pub_ready = True
            while not self.exit_event.is_set():
                # Wait for new frame to arrive
                frame_event.wait()
                receive_time = time.now()

                # Copy the newly arrived RGBD frame
                depth = session.get_depth_frame()
                rgb = session.get_rgb_frame()
                # confidence = session.get_confidence_frame()
                intrinsic_mat = get_intrinsic_mat_from_coeffs(session.get_intrinsic_mat())
                camera_pose = session.get_camera_pose()

                if rgb is None:
                    frame_event.clear()
                    continue

                # Postprocess it
                if device_type == DEVICE_TYPE__TRUEDEPTH:
                    depth = cv2.flip(depth, 1)
                    rgb = cv2.flip(rgb, 1)
                    depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]))
                elif device_type == DEVICE_TYPE__LIDAR:
                    depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]))
                else:
                    raise ValueError(device_type)

                # Quaternion + world position (accessible via camera_pose.[qx|qy|qz|qw|tx|ty|tz])
                qx, qy, qz, qw, px, py, pz = (
                    camera_pose.qx,
                    camera_pose.qy,
                    camera_pose.qz,
                    camera_pose.qw,
                    camera_pose.tx,
                    camera_pose.ty,
                    camera_pose.tz,
                )
                extrinsic_mat = np.eye(4, dtype=self.dtype)
                extrinsic_mat[:3, :3] = st.Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
                extrinsic_mat[:3, -1] = [px, py, pz]

                frame_event.clear()
                if stop_event.is_set():
                    raise RuntimeError

                camera_state = {}
                camera_state["camera_receive_timestamp"] = receive_time
                camera_state["camera_capture_timestamp"] = receive_time
                if self.enable_color:
                    camera_state["color"] = rgb
                if self.enable_depth:
                    camera_state["depth"] = depth
                if self.enable_intrinsics:
                    camera_state["intrinsics"] = intrinsic_mat
                if self.enable_extrinsics:
                    camera_state["extrinsics"] = extrinsic_mat

                # Store current state in ring buffer
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
        finally:
            session.disconnect()

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def set_default_settings(self):
        pass


def Record3dServer(mw, *args, **kwargs):
    return ServerFactory(mw, Record3d, *args, **kwargs)


def Record3dClient(mw, *args, **kwargs):
    return ClientFactory(mw, Record3d, *args, **kwargs)
