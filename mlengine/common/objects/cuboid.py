from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, fields
from typing import Any, Dict, Tuple

import numpy as np

from mlengine.common.type import FrameMarkupObject, FramePredictionObject

CuboidSize = Tuple[float, float, float]
CuboidCenter = Tuple[float, float, float]
CuboidOrientation = Tuple[float, float, float]


@dataclass(frozen=True)
class CuboidMarkup(FrameMarkupObject):
    label: str
    size: CuboidSize
    center: CuboidCenter
    orientation: CuboidOrientation
    velocity: float = 0.0
    object_id: int = 0
    attributes: Dict[str, Any] | None = None
    num_points: int | None = None

    def __post_init__(self) -> None:
        if self.attributes is None:
            object.__setattr__(self, "attributes", {})

    def to_array(self) -> np.ndarray:
        return np.array(
            [*self.center, *self.size, *self.orientation],
            dtype=np.float32,
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": "cuboid",
            "label": self.label,
            "size": list(self.size),
            "center": list(self.center),
            "orientation": list(self.orientation),
            "velocity": self.velocity,
            "object_id": self.object_id,
            "attributes": dict(self.attributes or {}),
            "num_points": self.num_points,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> CuboidMarkup:
        data = deepcopy(data)
        data.pop("type", None)
        allowed = {field.name for field in fields(cls)}
        kwargs = {key: value for key, value in data.items() if key in allowed}
        if "size" in kwargs:
            kwargs["size"] = tuple(kwargs["size"])
        if "center" in kwargs:
            kwargs["center"] = tuple(kwargs["center"])
        if "orientation" in kwargs:
            kwargs["orientation"] = tuple(kwargs["orientation"])
        return cls(**kwargs)


@dataclass(frozen=True)
class CuboidPredicted(CuboidMarkup, FramePredictionObject):
    confidence: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        result = super().to_dict()
        result["confidence"] = self.confidence
        return result

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> CuboidPredicted:
        data = deepcopy(data)
        data.pop("type", None)
        allowed = {field.name for field in fields(cls)}
        kwargs = {key: value for key, value in data.items() if key in allowed}
        if "size" in kwargs:
            kwargs["size"] = tuple(kwargs["size"])
        if "center" in kwargs:
            kwargs["center"] = tuple(kwargs["center"])
        if "orientation" in kwargs:
            kwargs["orientation"] = tuple(kwargs["orientation"])
        return cls(**kwargs)
