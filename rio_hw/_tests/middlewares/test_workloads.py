import contextlib
import importlib
import multiprocessing as mp
import queue
from enum import Enum, auto

import numpy as np
import pytest
from prettytable import PrettyTable

from ... import time
from ...middleware import ClientFactory, ServerFactory, ServerManager
from ...middlewares import SERVERLESS_MW
from ...middlewares import __all__ as MW_ALL
from ...node import Node
from ...request import Request
from .conftest import WorkloadConfig

_cfg = WorkloadConfig.from_env()
_timings = []  # [(middleware, test_name, [latency_s, ...]), ...]
_throughputs = []  # [(mw, test_name, n_nodes, total_msgs, duration_s), ...]


def _simulate_node_work(n_iters):
    """Simulate CPU-bound node work (state processing, interpolation) that holds the GIL."""
    s = 0
    for i in range(n_iters):
        s += i


def _make_timing_table(rows):
    table = PrettyTable()
    table.field_names = ["Middleware", "Median (ms)", "Std (ms)", "P5 (ms)", "P95 (ms)"]
    table.align["Middleware"] = "l"
    for col in table.field_names[1:]:
        table.align[col] = "r"
    for mw, latencies in sorted(rows, key=lambda x: np.median(x[1])):
        arr = np.array(latencies)
        p1, p99 = np.percentile(arr, [1, 99])
        trimmed = arr[(arr >= p1) & (arr <= p99)]
        if len(trimmed) == 0:
            trimmed = arr
        ms = trimmed * 1000
        table.add_row(
            [
                mw,
                f"{np.median(ms):.4f}",
                f"{np.std(ms):.4f}",
                f"{np.percentile(ms, 5):.4f}",
                f"{np.percentile(ms, 95):.4f}",
            ]
        )
    return table


def print_workload_timing_table(terminalreporter):
    if not _timings:
        return
    by_test = {}
    for mw, test, latencies in _timings:
        by_test.setdefault(test, []).append((mw, latencies))
    for test_name, rows in by_test.items():
        table = _make_timing_table(rows)
        terminalreporter.write_line("")
        terminalreporter.write_line(f"Middleware Latency: {test_name} (N={_cfg.n_iters})")
        for line in table.get_string().splitlines():
            terminalreporter.write_line(line)


def _make_throughput_table(rows):
    table = PrettyTable()
    table.field_names = ["Middleware", "Total msg/s", "Per-node msg/s", "Total msgs"]
    table.align["Middleware"] = "l"
    for col in table.field_names[1:]:
        table.align[col] = "r"
    for mw, n_nodes, total_msgs, duration in sorted(rows, key=lambda x: x[2] / x[3], reverse=True):
        total_rate = total_msgs / duration
        per_node_rate = total_rate / n_nodes
        table.add_row(
            [
                mw,
                f"{total_rate:.1f}",
                f"{per_node_rate:.1f}",
                f"{total_msgs}",
            ]
        )
    return table


def print_workload_throughput_table(terminalreporter):
    if not _throughputs:
        return
    by_test = {}
    for mw, test, n_nodes, total_msgs, duration in _throughputs:
        by_test.setdefault(test, []).append((mw, n_nodes, total_msgs, duration))
    for test_name, rows in by_test.items():
        table = _make_throughput_table(rows)
        terminalreporter.write_line("")
        terminalreporter.write_line(
            f"Middleware Throughput: {test_name} (N={_cfg.concurrent_nodes}, duration={_cfg.throughput_duration}s)"
        )
        for line in table.get_string().splitlines():
            terminalreporter.write_line(line)


def _get_state(ring_buffer, out=None):
    if hasattr(ring_buffer, "count") and ring_buffer.count == 0:
        return None
    result = ring_buffer.get(out=out)
    if isinstance(result, list) and len(result) == 0:
        return None
    return result


class PubReqNode(Node):
    class RequestType(Enum):
        SEND_REQUEST = auto()

    __api__ = ["get_state", "get_all_state", "send_request"]
    __pub__ = True
    __req__ = True

    def __init__(self, *, freq=100, max_buffer_size=30, max_queue_size=100, **kwargs):
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, max_queue_size=max_queue_size, **kwargs)

    def __post_init__(self):
        self.example_data = {"data": np.zeros(_cfg.payload_nelems), "timestamp": time.now()}
        self.example_request = {"type": next(iter(self.RequestType)).value, "data": np.zeros(_cfg.payload_nelems)}
        self.worker = None
        self.run = self.pubreq
        super().__post_init__()

    def pubreq(self):
        self.pub_ready_event.set()
        self.req_ready_event.set()
        try:
            while not self.exit_event.is_set():
                _simulate_node_work(_cfg.node_work_iters)
                try:
                    reqs = self.request_queue.get_all()
                    if isinstance(reqs, dict):
                        reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
                except queue.Empty:
                    reqs = []
                for r in reqs:
                    req = Request(self.RequestType(r.pop("type")), r)
                    assert req.type == self.RequestType.SEND_REQUEST
                    data = {"data": req.params["data"], "timestamp": time.now()}
                    self.ring_buffer.put(data)
                time.sleep(_cfg.spin_sleep)
        except KeyboardInterrupt:
            pass

    def get_state(self, out=None):
        return _get_state(self.ring_buffer, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    def send_request(self, data):
        self.request_queue.put({"type": self.RequestType.SEND_REQUEST.value, "data": data})


class PubOnlyNode(Node):
    __api__ = ["get_state"]
    __pub__ = True
    __req__ = False

    def __init__(self, *, freq=100, max_buffer_size=30, **kwargs):
        super().__init__(freq=freq, max_buffer_size=max_buffer_size, **kwargs)

    def __post_init__(self):
        self.example_data = {"value": np.zeros(_cfg.payload_nelems), "timestamp": time.now()}
        self.worker = None
        self.run = self.pub
        super().__post_init__()

    def pub(self):
        try:
            rng = np.random.RandomState(0)
            not_pub_ready = True
            while not self.exit_event.is_set():
                _simulate_node_work(_cfg.node_work_iters)
                data = {"value": rng.rand(_cfg.payload_nelems).astype(_cfg.payload_dtype), "timestamp": time.now()}
                self.ring_buffer.put(data)
                if not_pub_ready:
                    self.pub_ready_event.set()
                    not_pub_ready = False
        except KeyboardInterrupt:
            pass

    def get_state(self, out=None):
        return _get_state(self.ring_buffer, out=out)


class ReqOnlyNode(Node):
    class RequestType(Enum):
        ECHO = auto()

    __api__ = ["echo"]
    __pub__ = False
    __req__ = True

    def __init__(self, *, freq=100, max_queue_size=100, **kwargs):
        super().__init__(freq=freq, max_queue_size=max_queue_size, **kwargs)

    def __post_init__(self):
        self.example_request = {"type": next(iter(self.RequestType)).value, "data": np.zeros(_cfg.payload_nelems)}
        self.worker = None
        self.run = self._run_loop
        super().__post_init__()

    def _run_loop(self):
        self.req_ready_event.set()
        try:
            while not self.exit_event.is_set():
                _simulate_node_work(_cfg.node_work_iters)
                try:
                    reqs = self.request_queue.get_all()
                    if isinstance(reqs, dict):
                        reqs = [{k: reqs[k][i] for k in reqs.keys()} for i in range(len(reqs["type"]))]
                except queue.Empty:
                    reqs = []
                for r in reqs:
                    req = Request(self.RequestType(r.pop("type")), r)
                    assert req.type == self.RequestType.ECHO
                time.sleep(_cfg.spin_sleep)
        except KeyboardInterrupt:
            pass

    def echo(self, data):
        return {"data": np.array(data)}


def _make_kwargs(mw, node_cls, node_idx):
    """Returns (server_kwargs, client_kwargs) for a given node index."""
    _NODE_CLASSES = [PubReqNode, PubOnlyNode, ReqOnlyNode]
    mw_idx = MW_ALL.index(mw)
    test_idx = _NODE_CLASSES.index(node_cls)
    stride_test = _cfg.concurrent_nodes * _cfg.ports_per_node
    stride_mw = len(_NODE_CLASSES) * stride_test
    # port = base + mw_offset + test_offset + node_offset
    port = _cfg.base_port + mw_idx * stride_mw + test_idx * stride_test + node_idx * _cfg.ports_per_node
    addr = f"127.0.0.1:{port}"
    if mw in SERVERLESS_MW:
        server_kwargs = {}
        client_kwargs = {"shm_addr": addr} if mw in ("Shm", "Shmf") else {}
    else:
        server_kwargs = {"addr": addr}
        client_kwargs = {"addr": addr}
    return server_kwargs, client_kwargs


def _poll(fn, timeout=2.0, interval=0.05):
    deadline = time.now() + timeout
    while time.now() < deadline:
        try:
            result = fn()
        except Exception:
            result = None
        if result is not None and not (isinstance(result, list) and len(result) == 0):
            return result
        time.sleep(interval)
    return None


def _call_with_retry(fn, *args, timeout=2.0, interval=0.1, **kwargs):
    deadline = time.now() + timeout
    last_exc = None
    while time.now() < deadline:
        try:
            return fn(*args, **kwargs)
        except Exception as e:
            last_exc = e
            time.sleep(interval)
    raise last_exc


@pytest.fixture(params=MW_ALL)
def middleware(request):
    mw = request.param
    mod = importlib.import_module(f"...middlewares.{mw.lower()}", package=__package__)
    if getattr(mod, mw.lower(), ...) is None:
        pytest.skip(f"{mw} dependency not installed")
    start_method = "fork" if mw in ("Shmf",) else "spawn"
    mp.set_start_method(start_method, force=True)
    return mw


def _state_newer_than(get_fn, since):
    r = get_fn()
    return r if r is not None and r.get("timestamp", 0) > since else None


# -- Benchmark functions (round-robin across all clients) ----------------------


def _benchmark_latency(clients, n_iters, operate):
    latencies = []
    for _ in range(n_iters):
        for i, client in enumerate(clients):
            t0 = time.now()
            operate(i, client)
            latencies.append(time.now() - t0)
    return latencies


def _benchmark_throughput(clients, duration, operate):
    count = 0
    deadline = time.now() + duration
    while time.now() < deadline:
        for i, client in enumerate(clients):
            operate(i, client)
            count += 1
    return count


# -- Test functions ------------------------------------------------------------


def _run_scenario(name, node_cls, warmup, operate, middleware, workload):
    server_fns = []
    client_fns = []
    for i in range(workload.concurrent_nodes):
        s_kw, c_kw = _make_kwargs(middleware, node_cls, i)
        server_fns.append(lambda kw=s_kw: ServerFactory(middleware, node_cls, **kw))
        client_fns.append(lambda kw=c_kw: ClientFactory(middleware, node_cls, **kw))

    start_method = "fork" if middleware in ("Shmf",) else "spawn"
    with ServerManager(middleware, server_fns, start_method=start_method):
        time.sleep(0.5)
        with contextlib.ExitStack() as stack:
            clients = [stack.enter_context(fn()) for fn in client_fns]

            # warmup + correctness
            for i, client in enumerate(clients):
                warmup(i, client)

            # latency (round-robin across all clients from main thread)
            latencies = _benchmark_latency(clients, workload.n_iters, operate)
            _timings.append((middleware, name, latencies))

            # throughput (round-robin across all clients from main thread)
            t0 = time.now()
            total = _benchmark_throughput(clients, workload.throughput_duration, operate)
            dur = time.now() - t0
            _throughputs.append((middleware, name, workload.concurrent_nodes, total, dur))


def test_pub_req(middleware, workload):
    def warmup(i, client):
        data = np.random.RandomState(i).rand(_cfg.payload_nelems).astype(_cfg.payload_dtype)
        _call_with_retry(client.send_request, data)
        result = _poll(client.get_state)
        assert result is not None, f"Node {i}: Timed out waiting for data"
        np.testing.assert_array_almost_equal(result["data"], data)

    def operate(i, client):
        data = np.random.RandomState(i).rand(_cfg.payload_nelems).astype(_cfg.payload_dtype)
        since = time.now()
        client.send_request(data)
        _poll(lambda ts=since, c=client: _state_newer_than(c.get_state, ts), interval=_cfg.spin_sleep)

    _run_scenario("pub_req", PubReqNode, warmup, operate, middleware, workload)


def test_pub_only(middleware, workload):
    def warmup(i, client):
        result = _poll(client.get_state)
        assert result is not None, f"Node {i}: Timed out waiting for data"

    def operate(_i, client):
        client.get_state()

    _run_scenario("pub_only", PubOnlyNode, warmup, operate, middleware, workload)


def test_req_only(middleware, workload):
    def warmup(i, client):
        data = np.random.RandomState(i).rand(_cfg.payload_nelems).astype(_cfg.payload_dtype)
        result = _call_with_retry(client.echo, data)
        assert result is not None, f"Node {i}: no result"
        np.testing.assert_array_almost_equal(result["data"], data)

    def operate(i, client):
        data = np.random.RandomState(i).rand(_cfg.payload_nelems).astype(_cfg.payload_dtype)
        client.echo(data)

    _run_scenario("req_only", ReqOnlyNode, warmup, operate, middleware, workload)
