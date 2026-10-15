"""Openpi-compatible NumPy/msgpack representation without openpi imports.

Only numeric/boolean arrays and scalars are accepted. No pickle is used.
"""
import numpy as np


def _pack(value):
    if isinstance(value, (np.ndarray, np.generic)):
        if value.dtype.kind not in "biuf":
            raise ValueError(f"Unsupported wire dtype: {value.dtype}")
        if isinstance(value, np.ndarray):
            return {b"__ndarray__": True, b"data": value.tobytes(),
                    b"dtype": value.dtype.str, b"shape": value.shape}
        return {b"__npgeneric__": True, b"data": value.item(), b"dtype": value.dtype.str}
    raise TypeError(f"Cannot encode {type(value).__name__}")


def _unpack(value):
    if b"__ndarray__" in value or b"__npgeneric__" in value:
        dtype = np.dtype(value[b"dtype"])
        if dtype.kind not in "biuf":
            raise ValueError(f"Unsupported wire dtype: {dtype}")
        if b"__ndarray__" in value:
            return np.frombuffer(value[b"data"], dtype=dtype).reshape(value[b"shape"]).copy()
        return dtype.type(value[b"data"])
    return value


def pack(value):
    import msgpack
    return msgpack.packb(value, default=_pack)


def unpack(data):
    import msgpack
    return msgpack.unpackb(data, object_hook=_unpack, strict_map_key=False)
