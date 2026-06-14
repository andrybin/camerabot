from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path

import numpy as np
from PIL import Image

class Serializer(ABC):
    @abstractmethod
    def encode(self, data: np.ndarray) -> bytes:
        ...

    @abstractmethod
    def decode(self, data: bytes) -> np.ndarray:
        ...

    @abstractmethod
    def load(self) -> np.ndarray | Image.Image:
        ...

    @abstractmethod
    def load_as_array(self) -> np.ndarray:
        ...

    @abstractmethod
    def save(self, data: np.ndarray) -> Path | str | list[str]:
        ...

    @abstractmethod
    def count(self) -> int:
        ...


class FileSerializer(Serializer):
    """Bind one on-disk file per ``__call__(filename)``."""

    def __init__(self) -> None:
        self._path_to_file: Path | None = None

    def __call__(self, filename: str):
        self._path_to_file = self._resolve_path(filename)
        return self

    def _require_path(self) -> Path:
        if self._path_to_file is None:
            raise ValueError("Path is not set. Call serializer with filename first.")
        return self._path_to_file

    @abstractmethod
    def _resolve_path(self, filename: str) -> Path:
        ...


class KeyedSerializer(Serializer):
    """Bind one or many keys (filename stems) in a shared archive."""

    def __init__(self) -> None:
        self._key: str | list[str] | None = None

    def __call__(self, filename_one_or_many: str | list[str]):
        if isinstance(filename_one_or_many, str):
            self._key = Path(filename_one_or_many).stem
        elif isinstance(filename_one_or_many, list):
            self._key = [Path(filename).stem for filename in filename_one_or_many]
        else:
            raise ValueError("filename_one_or_many must be a string or a list of strings")
        return self

    def _require_key(self) -> str:
        if self._key is None or isinstance(self._key, list):
            raise ValueError("A single key is not set. Call serializer with a filename first.")
        return self._key


from .image import (
    ImageArraySerializer,
    ImageArraySerializerCfg,
    ImageFileSerializer,
    ImageFileSerializerCfg,
)
from .pcd import (
    ArrayLidarSerializer,
    ArrayLidarSerializerCfg,
    QDSLidarCodec,
    QDSLidarCodecCfg,
)

__all__ = [
    "ArrayLidarSerializer",
    "ArrayLidarSerializerCfg",
    "BevMaskSerializer",
    "BevMaskSerializerCfg",
    "FileSerializer",
    "ImageArraySerializer",
    "ImageArraySerializerCfg",
    "ImageFileSerializer",
    "ImageFileSerializerCfg",
    "KeyedSerializer",
    "QDSLidarCodec",
    "QDSLidarCodecCfg",
    "Serializer",
]
