from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from shutil import rmtree

import numpy as np


from . import FileSerializer


@dataclass
class QDSLidarCodecCfg:
    pcd_dir: str = "pointclouds"
    file_suffix: str = ".pcd"
    scale: float = 100.0
    clear_on_init: bool = False


class QDSLidarCodec(FileSerializer):
    """Serializer wrapper for QuantizedDeltaSplitLidarCodec."""

    def __init__(self, cfg: QDSLidarCodecCfg):
        from cv_metrics.serialize import QDSLidarCodec as QuantizedDeltaSplitLidarCodec

        super().__init__()
        self.cfg = cfg
        self.pcd_dir = Path(cfg.pcd_dir)
        self.file_suffix = cfg.file_suffix
        self.codec = QuantizedDeltaSplitLidarCodec(scale=cfg.scale)

        if self.cfg.clear_on_init and self.pcd_dir.exists():
            rmtree(self.pcd_dir)
        self.pcd_dir.mkdir(parents=True, exist_ok=True)

    def _resolve_path(self, filename: str) -> Path:
        path = Path(filename)
        if path.suffix == self.file_suffix:
            return self.pcd_dir / path
        return self.pcd_dir / path.with_suffix(self.file_suffix)

    def encode(self, data: np.ndarray) -> bytes:
        per_lidar_points = self.codec.split_fused_by_lidar_id(data)
        return self.codec.encode(per_lidar_points)

    def decode(self, data: bytes) -> np.ndarray:
        return self.codec.decode(data)

    def load(self) -> np.ndarray:
        return self.codec.load(self._require_path())

    def load_as_array(self) -> np.ndarray:
        return self.load()

    def save(self, data: np.ndarray) -> Path:
        path = self._require_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        per_lidar_points = self.codec.split_fused_by_lidar_id(data)
        self.codec.save(per_lidar_points, path)
        return path

    def count(self) -> int:
        if not self.pcd_dir.exists():
            return 0
        return len(list(self.pcd_dir.glob(f"*{self.file_suffix}")))


@dataclass
class ArrayLidarSerializerCfg:
    """One point cloud per binary file: ``uint64 n_pts`` (LE) + ``n_pts * n_columns`` values."""

    pcd_dir: str = "pointclouds"
    file_suffix: str = ".bin"
    n_columns: int = 4
    dtype: str = "float32"
    clear_on_init: bool = False


class ArrayLidarSerializer(FileSerializer):
    """Read/write point clouds as raw binary: ``n_pts`` (uint64 LE) + row-major ``(n_pts, n_columns)``."""

    _COUNT_DTYPE = np.dtype("<u8")

    def __init__(self, cfg: ArrayLidarSerializerCfg):
        super().__init__()
        self.cfg = cfg
        self.pcd_dir = Path(cfg.pcd_dir)
        self.file_suffix = cfg.file_suffix
        self.dtype = np.dtype(cfg.dtype)
        self.n_columns = int(cfg.n_columns)

        if self.cfg.clear_on_init and self.pcd_dir.exists():
            rmtree(self.pcd_dir)
        self.pcd_dir.mkdir(parents=True, exist_ok=True)

    def _resolve_path(self, filename: str) -> Path:
        path = Path(filename)
        if path.suffix == self.file_suffix:
            return self.pcd_dir / path
        return self.pcd_dir / path.with_suffix(self.file_suffix)

    def _pack(self, arr: np.ndarray) -> bytes:
        flat = np.asarray(arr, dtype=self.dtype, order="C").ravel()
        n_pts = flat.size // self.n_columns
        arr = flat.reshape(n_pts, self.n_columns)
        header = np.array([n_pts], dtype=self._COUNT_DTYPE).tobytes()
        return header + arr.tobytes()

    def _unpack(self, blob: bytes) -> np.ndarray:
        n_pts = int(np.frombuffer(blob[:8], dtype=self._COUNT_DTYPE, count=1)[0])
        flat = np.frombuffer(blob, dtype=self.dtype, offset=8)
        return flat.reshape(n_pts, self.n_columns).copy()

    def load(self) -> np.ndarray:
        return self._unpack(self._require_path().read_bytes())

    def load_as_array(self) -> np.ndarray:
        return self.load()

    def save(self, data: np.ndarray) -> Path:
        path = self._require_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(self._pack(data))
        return path

    def encode(self, data: np.ndarray) -> bytes:
        return self._pack(data)

    def decode(self, data: bytes) -> np.ndarray:
        return self._unpack(data)

    def count(self) -> int:
        if not self.pcd_dir.exists():
            return 0
        return len(list(self.pcd_dir.glob(f"*{self.file_suffix}")))
