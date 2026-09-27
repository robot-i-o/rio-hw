import queue

import numpy as np

import rio_hw.sensors.ati_ft as ati_ft_module
from rio_hw.sensors.ati_ft import AtiFt


class _FakeResponse:
    rdt_sequence = 11
    ft_sequence = 22
    status = 33
    FTData = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


class _FakeSocket:
    def __init__(self):
        self.timeout = None

    def settimeout(self, timeout):
        self.timeout = timeout


class _FakeNetFt:
    instances = []

    def __init__(self, host, port, num_samples, count_per_force, count_per_torque):
        self.host = host
        self.port = port
        self.num_samples = num_samples
        self.count_per_force = count_per_force
        self.count_per_torque = count_per_torque
        self.sock = _FakeSocket()
        self.connected = False
        self.biased = False
        self.disconnected = False
        self.instances.append(self)

    def connect(self):
        self.connected = True

    def bias(self):
        self.biased = True

    def get_converted_data(self):
        return _FakeResponse()

    def disconnect(self):
        self.disconnected = True


class _FakeRingBuffer:
    def __init__(self):
        self.items = []

    def put(self, data, wait=True):
        self.items.append(data)


class _FakeEvent:
    def __init__(self):
        self.is_set_calls = 0
        self.was_set = False

    def is_set(self):
        self.is_set_calls += 1
        return self.is_set_calls > 1

    def set(self):
        self.was_set = True


class _FakeRequestQueue:
    def get_all(self):
        raise queue.Empty


class _FakeRate:
    def __init__(self, freq):
        self.freq = freq

    def precise_sleep(self):
        pass


def test_ati_ft_publishes_converted_wrench(monkeypatch):
    _FakeNetFt.instances = []
    monkeypatch.setattr(ati_ft_module, "NetFT", _FakeNetFt)
    monkeypatch.setattr(ati_ft_module.time, "Rate", _FakeRate)

    sensor = AtiFt(
        host="192.168.1.1",
        port=49152,
        num_samples=1,
        count_per_force=1000000,
        count_per_torque=999.999,
        bias_on_start=True,
        read_timeout=0.05,
        dtype=np.float64,
        freq=100,
    )
    sensor.freq = 100
    sensor.ring_buffer = _FakeRingBuffer()
    sensor.request_queue = _FakeRequestQueue()
    sensor.pub_ready_event = _FakeEvent()
    sensor.req_ready_event = _FakeEvent()
    sensor.exit_event = _FakeEvent()

    sensor.pubreq()

    netft = _FakeNetFt.instances[0]
    assert netft.connected
    assert netft.biased
    assert netft.disconnected
    assert netft.sock.timeout == 0.05
    assert sensor.pub_ready_event.was_set
    assert sensor.req_ready_event.was_set

    data = sensor.ring_buffer.items[0]
    np.testing.assert_array_equal(data["force"], np.array([1.0, 2.0, 3.0], dtype=np.float64))
    np.testing.assert_array_equal(data["torque"], np.array([4.0, 5.0, 6.0], dtype=np.float64))
    np.testing.assert_array_equal(data["wrench"], np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float64))
    assert data["ft_status"] == 33
    assert data["rdt_sequence"] == 11
    assert data["ft_sequence"] == 22
    assert "timestamp" in data
