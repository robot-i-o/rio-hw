import numpy as np
import pytest

from .. import serializers
from ..serializers import __all__ as SERIALIZERS_ALL

_DATA = {
    "float": 3.14,
    "bool": True,
    "np.array": np.array([1.0, 2.0, 3.0]),
    "list[np.array]": [np.array([1.0, 2.0]), np.array([3.0, 4.0])],
    "dict{np.array}": {"a": np.array([1.0, 2.0]), "b": np.array([3.0, 4.0])},
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
    if isinstance(data, dict):
        assert result.keys() == data.keys()
        for k in data:
            np.testing.assert_array_almost_equal(result[k], data[k])
    elif isinstance(data, list):
        assert len(result) == len(data)
        for r, d in zip(result, data, strict=True):
            np.testing.assert_array_almost_equal(r, d)
    else:
        np.testing.assert_array_almost_equal(result, data)
