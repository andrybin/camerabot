from __future__ import annotations

import io
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np
from PIL import Image
from shutil import rmtree

from . import FileSerializer, KeyedSerializer


@dataclass
class ImageFileSerializerCfg:
    image_dir: str = "images"
    image_format: str = "PNG"
    mode: Literal["RGB", "L"] = "RGB"


@dataclass
class ImageArraySerializerCfg:
    array_filename: str = "array.npy"
    dtype: np.dtype = np.uint8
    mode: Literal["RGB", "L"] = "RGB"


class ImageFileSerializer(FileSerializer):
    def __init__(self, cfg: ImageFileSerializerCfg):
        super().__init__()
        self.cfg = cfg
        self.image_dir = Path(cfg.image_dir)
        self.image_format = cfg.image_format
        self.image_dir.mkdir(parents=True, exist_ok=True)

    def init_source(self):
        if self.image_dir.exists():
            rmtree(self.image_dir)
        self.image_dir.mkdir(parents=True, exist_ok=True)

    def _resolve_path(self, filename: str) -> Path:
        return self.image_dir / Path(filename).with_suffix(f".{self.image_format}").name

    def encode(self, data: np.ndarray) -> bytes:
        buf = io.BytesIO()
        Image.fromarray(data, mode=self.cfg.mode).save(buf, format=self.cfg.image_format)
        return buf.getvalue()

    def decode(self, data: bytes) -> np.ndarray:
        with Image.open(io.BytesIO(data)) as image:
            return np.array(image.convert(self.cfg.mode), dtype=np.uint8)

    def load(self) -> Image.Image:
        return Image.open(self._require_path())

    def load_as_array(self) -> np.ndarray:
        return np.array(self.load(), dtype=np.uint8)

    def save(self, data: np.ndarray) -> str:
        path = self._require_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as f:
            f.write(self.encode(data))
        return path.name

    def count(self) -> int:
        return len(list(self.image_dir.glob(f"*.{self.image_format}")))


class ImageArraySerializer(KeyedSerializer):
    def __init__(self, cfg: ImageArraySerializerCfg):
        super().__init__()
        self.cfg = cfg
        self.array_path = Path(cfg.array_filename)
        self.image_dir = self.array_path.parent
        self.array_path.parent.mkdir(parents=True, exist_ok=True)
        self.array = None

    def init_source(self):
        if self.array_path.exists():
            self.array_path.unlink()
        self.array_path.parent.mkdir(parents=True, exist_ok=True)

    def _read_array(self) -> dict[str, np.ndarray]:
        if self.array is None:
            if not self.array_path.exists() or self.array_path.stat().st_size == 0:
                self.array = {}
            else:
                self.array = np.load(self.array_path, allow_pickle=True).item()
        return self.array

    def _prepare(self, data: np.ndarray) -> np.ndarray:
        return np.asarray(data, dtype=self.cfg.dtype)

    def encode(self, data: np.ndarray) -> bytes:
        buf = io.BytesIO()
        np.save(buf, self._prepare(data), allow_pickle=False)
        return buf.getvalue()

    def decode(self, data: bytes) -> np.ndarray:
        return np.load(io.BytesIO(data), allow_pickle=False)

    def load(self) -> Image.Image:
        return Image.fromarray(self.load_as_array(), mode=self.cfg.mode)

    def load_as_array(self) -> np.ndarray:
        return self._read_array()[self._require_key()]

    def save(self, data: np.ndarray | list[np.ndarray]) -> str | list[str]:
        self.array_path.parent.mkdir(parents=True, exist_ok=True)
        array = self._read_array()
        if isinstance(data, np.ndarray):
            array[self._key] = self._prepare(data)
        elif isinstance(data, list):
            for key, item in zip(self._key, data):
                array[key] = self._prepare(item)
        else:
            raise ValueError("data must be a numpy array or a list of numpy arrays")
        np.save(self.array_path, array, allow_pickle=True)
        return self._key

    def count(self) -> int:
        return len(self._read_array())
