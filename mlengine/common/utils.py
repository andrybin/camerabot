from typing import Dict, Any, List
import numpy as np

DEFAULT_GROUND_ELEVATION_STEP = 0.01


def _rle_encode_flat(flat: np.ndarray) -> List[int]:
    flat = np.asarray(flat).reshape(-1)
    if flat.size == 0:
        return []
    changes = np.concatenate(([0], np.flatnonzero(flat[1:] != flat[:-1]) + 1, [flat.size]))
    values = flat[changes[:-1]]
    counts = np.diff(changes)
    return [x for value, count in zip(values, counts) for x in (int(value), int(count))]


def _rle_decode_flat(rle: List[int], size: int) -> np.ndarray:
    flat = np.empty(size, dtype=np.int32)
    idx = 0
    for i in range(0, len(rle), 2):
        value, count = rle[i], rle[i + 1]
        flat[idx : idx + count] = value
        idx += count
    if idx != size:
        raise ValueError(f"RLE decodes to {idx} pixels, expected {size}")
    return flat


def rle_encode_delta_map(arr: np.ndarray, step: float = DEFAULT_GROUND_ELEVATION_STEP) -> Dict[str, Any]:
    arr = np.asarray(arr, dtype=np.float32)
    if arr.size == 0:
        return {"shape": list(arr.shape), "step": step, "rle": []}

    quantized = np.round(arr / step).astype(np.int32).ravel()
    deltas = np.empty_like(quantized)
    deltas[0] = quantized[0]
    deltas[1:] = np.diff(quantized)
    return {
        "shape": list(arr.shape),
        "step": step,
        "rle": _rle_encode_flat(deltas),
    }


def rle_decode_delta_map(data: Dict[str, Any]) -> np.ndarray:
    shape = tuple(data["shape"])
    step = float(data["step"])
    size = int(np.prod(shape))
    deltas = _rle_decode_flat(data["rle"], size)
    quantized = np.cumsum(deltas, dtype=np.int32)
    return (quantized.reshape(shape) * step).astype(np.float32)


def rle_encode_mask(mask: np.ndarray) -> Dict[str, Any]:
    flat = np.asarray(mask).reshape(-1)
    if flat.size == 0:
        return {"shape": list(mask.shape), "rle": []}
    return {"shape": list(mask.shape), "rle": _rle_encode_flat(flat)}


def rle_decode_mask(data: Dict[str, Any]) -> np.ndarray:
    shape = tuple(data["shape"])
    size = int(np.prod(shape))
    flat = _rle_decode_flat(data["rle"], size).astype(np.uint8)
    return flat.reshape(shape)