import asyncio
import pathlib
import queue
from typing import TYPE_CHECKING

import numpy as np
import scipy.spatial.transform as st

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node

try:
    import vuer
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        vuer = None  # type: ignore


class Vuer(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "push_frame",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        vr_resolution: tuple[int, int] = (480, 640),
        vuer_file_dir: str | None = None,
        *,
        freq: int = 60,
        max_buffer_size: int = 30,
        dtype=np.float32,
        **kwargs,
    ):
        self.resolution = vr_resolution
        if vuer_file_dir is None:
            vuer_file_dir = (pathlib.Path.home() / ".cache" / "vuer").resolve()
        self.vuer_file_dir = vuer_file_dir
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        example_data_left = {
            "left_controller": False,  # controller enabled
            "left_pos": np.zeros((3,), dtype=np.float32),
            "left_quat": np.zeros((4,), dtype=np.float32),
            "left_a": False,
            "left_b": False,
            "left_trigger": 0.0,
            "left_grip": 0.0,
            "left_joystick": np.zeros((2,), dtype=np.float32),
        }
        example_data_right = {
            "right_controller": False,  # controller enabled
            "right_pos": np.zeros((3,), dtype=np.float32),
            "right_quat": np.zeros((4,), dtype=np.float32),
            "right_a": False,
            "right_b": False,
            "right_trigger": 0.0,
            "right_grip": 0.0,
            "right_joystick": np.zeros((2,), dtype=np.float32),
        }

        self.example_request = {
            "rgb": np.zeros(shape=(*self.resolution, 3), dtype=np.uint8),
        }
        self.example_data = {
            **example_data_left,
            **example_data_right,
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pubreq
        super().__post_init__()

    def pubreq(self):
        key_file = pathlib.Path(self.vuer_file_dir / "key.pem")
        cert_file = pathlib.Path(self.vuer_file_dir / "cert.pem")

        assert key_file.exists(), f"Key file not found at {key_file.resolve()}"
        assert cert_file.exists(), f"Cert file not found at {cert_file.resolve()}"

        app = vuer.Vuer(host="0.0.0.0", key_file=str(key_file), cert_file=str(cert_file), queue_len=3)
        self.not_pub_ready = True

        @app.add_handler("CONTROLLER_MOVE")
        async def handler(
            event,
            session: vuer.VuerSession,
        ):
            # WebXR gives column-major mat44 matrix
            try:
                left_mat44 = np.array(event.value["left"]).reshape((4, 4))
                left_pos, left_quat = self._convert_pose(left_mat44)

                left_state = event.value["leftState"]

                left_trigger = left_state["triggerValue"]
                left_a = left_state["aButton"]
                left_b = left_state["bButton"]
                left_grip = left_state.get("squeezeValue", 0.0)
                left_joystick = left_state.get("thumbstickValue", [0.0, 0.0])

                left_data = {
                    "left_controller": True,
                    "left_pos": left_pos,
                    "left_quat": left_quat,
                    "left_trigger": left_trigger,
                    "left_a": left_a,
                    "left_b": left_b,
                    "left_grip": left_grip,
                    "left_joystick": left_joystick,
                }
            except Exception:
                left_data = {}

            try:
                right_mat44 = np.array(event.value["right"]).reshape((4, 4))
                right_pos, right_quat = self._convert_pose(right_mat44)

                right_state = event.value["rightState"]

                right_trigger = right_state["triggerValue"]
                right_a = right_state["aButton"]
                right_b = right_state["bButton"]
                right_grip = right_state.get("gripValue", 0.0)
                right_joystick = right_state.get("thumbstickValue", [0.0, 0.0])

                right_data = {
                    "right_controller": True,
                    "right_pos": right_pos,
                    "right_quat": right_quat,
                    "right_trigger": right_trigger,
                    "right_a": right_a,
                    "right_b": right_b,
                    "right_grip": right_grip,
                    "right_joystick": right_joystick,
                }
            except Exception:
                right_data = {}

            # Store current state in ring buffer
            data = {**left_data, **right_data, "timestamp": time.now()}
            self.ring_buffer.put(data)
            if self.not_pub_ready:
                self.pub_ready_event.set()
                self.not_pub_ready = False

        @app.spawn(start=False)
        async def main(session: vuer.VuerSession):
            # Important: You need to set the `stream` option to `True` to start
            # streaming the controller movement.
            session.upsert @ vuer.schemas.MotionControllers(stream=True, key="motion-controller", left=True, right=True)
            session.remove @ "grid"

            try:
                while not self.exit_event.is_set():
                    # Fetch requests from queue
                    try:
                        reqs = self.request_queue.get_all()
                        if isinstance(reqs, dict):
                            reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
                    except queue.Empty:
                        reqs = []
                    for r in reqs:
                        rgb = r["rgb"][:, :, [2, 1, 0]]
                        session.upsert(
                            vuer.schemas.ImageBackground(
                                rgb,
                                height=0.5,
                                distanceToCamera=2,
                                format="jpeg",
                                quality=80,
                                key="background-mono",
                                interpolate=True,
                            ),
                            to="bgChildren",
                        )
                    await asyncio.sleep(1.0 / self.freq)
            except Exception as e:
                print(e)

        try:
            app.run()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"Vuer encountered an error: {e}")

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def push_frame(self, frame):
        frame = np.array(frame, dtype=self.dtype)
        req = {
            "rgb": frame,
        }
        self.request_queue.put(req)

    def _convert_pose(self, pose: np.array):
        pos = pose[3, :3]
        rot33 = pose[:3, :3].T
        rot = st.Rotation.from_matrix(rot33)

        # webxr to right-hand coordinate transform
        coordinate_transform = st.Rotation.from_matrix([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        pos = coordinate_transform.apply(pos)
        quat = (coordinate_transform * rot * coordinate_transform.inv()).as_quat()
        return pos, quat


def VuerServer(mw, *args, **kwargs):
    return ServerFactory(mw, Vuer, *args, **kwargs)


def VuerClient(mw, *args, **kwargs):
    return ClientFactory(mw, Vuer, *args, **kwargs)
