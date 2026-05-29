import pickle
from typing import Any

import numpy as np


def pack_numpy(value: Any) -> Any:
    """Convert NumPy values to pickle-stable Python containers."""
    if isinstance(value, np.ndarray):
        contiguous = np.ascontiguousarray(value)
        return {
            "__ndarray__": True,
            "dtype": str(contiguous.dtype),
            "shape": contiguous.shape,
            "data": contiguous.tobytes(),
        }
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: pack_numpy(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return type(value)(pack_numpy(item) for item in value)
    return value


def unpack_numpy(value: Any) -> Any:
    """Reconstruct NumPy values from pickle-stable Python containers."""
    if isinstance(value, dict):
        if value.get("__ndarray__") is True:
            array = np.frombuffer(value["data"], dtype=np.dtype(value["dtype"]))
            return array.reshape(value["shape"]).copy()
        return {key: unpack_numpy(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return type(value)(unpack_numpy(item) for item in value)
    return value


def dumps_message(value: Any) -> bytes:
    return pickle.dumps(pack_numpy(value), protocol=pickle.HIGHEST_PROTOCOL)


def loads_message(message: bytes) -> Any:
    return unpack_numpy(pickle.loads(message))
