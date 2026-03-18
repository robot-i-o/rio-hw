from __future__ import annotations

import numpy as np
import pytest

from .. import serializers
from ..serializers import __all__ as SERIALIZERS_ALL

_DATA = {
    "float": 3.14,
    "bool": True,
    "np.array": np.array([1.0, 2.0, 3.0]),
}


@pytest.fixture(params=SERIALIZERS_ALL)
def serializer(request):
    cls = getattr(serializers, request.param)
    try:
        cls.pack([1, 2, 3])
    except AttributeError:
        pytest.skip(f"{request.param} dependency not installed")
    return cls


@pytest.fixture(params=_DATA.keys())
def data(request):
    return _DATA[request.param]


def test_roundtrip(serializer, data):
    result = serializer.unpack(serializer.pack(data))
    np.testing.assert_array_almost_equal(result, data)
