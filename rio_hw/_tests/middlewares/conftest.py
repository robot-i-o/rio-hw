import os
from dataclasses import dataclass, fields

import numpy as np
import pytest


@dataclass
class WorkloadConfig:
    base_port: int = 5000
    payload_dtype: str = "float32"
    payload_bytes: int = 2048
    n_iters: int = 100
    spin_sleep: float = 0.00001
    concurrent_nodes: int = 4
    node_work_iters: int = 10000
    ports_per_node: int = 2  # some nodes like Zmq need multiple ports
    throughput_duration: float = 2.0

    @property
    def payload_nelems(self) -> int:
        return self.payload_bytes // np.dtype(self.payload_dtype).itemsize

    @classmethod
    def from_env(cls) -> "WorkloadConfig":
        kw = {}
        for f in fields(cls):
            val = os.environ.get(f.name.upper())
            if val is not None:
                kw[f.name] = f.type(val)
        return cls(**kw)


def pytest_addoption(parser):
    parser.addoption("--workload", action="store_true", default=False, help="Run middleware workload tests")
    for f in fields(WorkloadConfig):
        parser.addoption(
            f"--workload.{f.name}",
            dest=f"workload_{f.name}",
            type=f.type,
            default=f.default,
        )


@pytest.fixture
def workload(request) -> WorkloadConfig:
    if not request.config.getoption("--workload"):
        pytest.skip("needs --workload to run")
    cfg = WorkloadConfig(**{f.name: request.config.getoption(f"workload_{f.name}") for f in fields(WorkloadConfig)})
    for f in fields(WorkloadConfig):
        os.environ[f.name.upper()] = str(getattr(cfg, f.name))
    from . import test_workloads  # noqa: PLC0415

    test_workloads._cfg = cfg
    return cfg


def pytest_terminal_summary(terminalreporter, exitstatus, config):
    from .test_workloads import (  # noqa: PLC0415
        print_workload_throughput_table,
        print_workload_timing_table,
    )

    print_workload_timing_table(terminalreporter)
    print_workload_throughput_table(terminalreporter)
