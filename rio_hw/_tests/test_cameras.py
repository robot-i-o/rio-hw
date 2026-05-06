import importlib

import numpy as np
import pytest

from ..cameras import __all__ as CAMERAS_ALL


@pytest.fixture(params=CAMERAS_ALL)
def camera(request):
    name = request.param
    mod_name = f"rio_hw.cameras.{name.lower()}"
    try:
        mod = importlib.import_module(mod_name)
    except ImportError:
        pytest.skip(f"{name} dependency not installed")

    get_connected = getattr(mod, "get_connected_cameras", None)
    if get_connected is None:
        pytest.skip(f"{name} has no get_connected_cameras()")

    try:
        serials, models = get_connected()
    except Exception:
        pytest.skip(f"{name} failed to query connected cameras")

    if len(serials) == 0:
        pytest.skip(f"No {name} cameras connected")

    return {
        "name": name,
        "Server": getattr(mod, f"{name}Server"),
        "Client": getattr(mod, f"{name}Client"),
        "serial": serials[0],
        "model": models[0],
        "module": mod,
    }


def test_get_frame(camera):
    server = camera["Server"](
        "Shm",
        serial=camera["serial"],
        model=camera["model"],
        enable_color=True,
        enable_depth=False,
    )
    client = camera["Client"](
        "Shm",
        serial=camera["serial"],
        model=camera["model"],
        enable_color=True,
        enable_depth=False,
    )

    with server, client as cam:
        data = cam.get_state()

    assert "color" in data
    assert isinstance(data["color"], np.ndarray)
    assert data["color"].size > 0
    assert np.any(data["color"] != 0)
