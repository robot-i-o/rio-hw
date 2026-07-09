import queue
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from .. import time
from ..middleware import ClientFactory, ServerFactory
from ..node import Node
from ..request import Request

try:
    from pynetft import NetFT
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        NetFT = None  # type: ignore


class RequestType(Enum):
    BIAS = auto()


class AtiFt(Node):
    __api__ = [
        "get_state",
        "get_all_state",
        "bias",
    ]
    __pub__ = True
    __req__ = True

    def __init__(
        self,
        host: str = "192.168.1.1",
        port: int = 49152,
        num_samples: int = 1,
        count_per_force: float = 1000000,
        count_per_torque: float = 999.999,
        bias_on_start: bool = False,
        read_timeout: float | None = 0.02,
        dtype=np.float32,
        *,
        freq: int = 100,
        max_buffer_size: int = 30,
        max_queue_size: int = 128,
        **kwargs,
    ):
        """
        Args:
            host: IP address of the ATI Net F/T sensor.
            port: UDP RDT port of the sensor.
            num_samples: Number of samples to request per reading.
            count_per_force: Sensor counts per force unit.
            count_per_torque: Sensor counts per torque unit.
            bias_on_start: Whether to zero the sensor at startup.
            read_timeout: Socket read timeout in seconds, or None to block.
            dtype: numpy dtype for force and torque arrays.
            freq: polling frequency in Hz.
            max_buffer_size: ring buffer size.
            max_queue_size: request queue size.
        """
        self.host = host
        self.port = port
        self.num_samples = num_samples
        self.count_per_force = count_per_force
        self.count_per_torque = count_per_torque
        self.bias_on_start = bias_on_start
        self.read_timeout = read_timeout
        self.dtype = dtype
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, max_queue_size=max_queue_size, **kwargs)

    def __post_init__(self):
        example_sensor_state = {
            "force": np.zeros((3,), dtype=self.dtype),
            "torque": np.zeros((3,), dtype=self.dtype),
            "wrench": np.zeros((6,), dtype=self.dtype),
            "ft_status": np.uint32(0),
            "rdt_sequence": np.uint32(0),
            "ft_sequence": np.uint32(0),
            "sensor_receive_timestamp": 0.0,
        }

        self.example_request = {
            "type": next(iter(RequestType)).value,
        }
        self.example_data = {
            **example_sensor_state,
            "timestamp": time.now(),
        }
        self.worker = None
        self.run = self.pubreq
        super().__post_init__()

    def pubreq(self):
        if NetFT is None:
            raise ImportError("pynetft is required to use AtiFt. Install rio_hw[sensors].")

        sensor = NetFT(
            host=self.host,
            port=self.port,
            num_samples=self.num_samples,
            count_per_force=self.count_per_force,
            count_per_torque=self.count_per_torque,
        )
        sensor.connect()
        if self.read_timeout is not None and getattr(sensor, "sock", None) is not None:
            sensor.sock.settimeout(self.read_timeout)
        if self.bias_on_start:
            sensor.bias()

        try:
            rate = time.Rate(self.freq)
            self.req_ready_event.set()
            not_pub_ready = True
            while not self.exit_event.is_set():
                try:
                    reqs = self.request_queue.get_all()
                    if isinstance(reqs, dict):
                        reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
                except queue.Empty:
                    reqs = []
                for r in reqs:
                    req = Request(RequestType(r.pop("type")), r)
                    if req.type == RequestType.BIAS:
                        sensor.bias()
                    else:
                        raise RuntimeError(req.type)

                try:
                    resp = sensor.get_converted_data()
                    receive_time = time.now()
                except TimeoutError:
                    rate.precise_sleep()
                    continue

                data = {
                    **self._response_to_state(resp, receive_time),
                    "timestamp": time.now(),
                }
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False
                rate.precise_sleep()
        except KeyboardInterrupt:
            pass
        finally:
            sensor.disconnect()

    def _response_to_state(self, resp, receive_time: float):
        wrench = np.asarray(resp.FTData, dtype=self.dtype)
        if wrench.shape != (6,):
            raise ValueError(f"Expected 6-axis force/torque data, got shape {wrench.shape}")
        return {
            "force": wrench[:3].copy(),
            "torque": wrench[3:].copy(),
            "wrench": wrench,
            "ft_status": np.uint32(resp.status),
            "rdt_sequence": np.uint32(resp.rdt_sequence),
            "ft_sequence": np.uint32(resp.ft_sequence),
            "sensor_receive_timestamp": receive_time,
        }

    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def bias(self):
        req = {
            "type": RequestType.BIAS.value,
        }
        self.request_queue.put(req)


def AtiFtServer(mw, *args, **kwargs):
    return ServerFactory(mw, AtiFt, *args, **kwargs)


def AtiFtClient(mw, *args, **kwargs):
    return ClientFactory(mw, AtiFt, *args, **kwargs)
