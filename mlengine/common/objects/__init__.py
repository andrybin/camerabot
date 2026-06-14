from __future__ import annotations

from typing import Type

# TODO Remove 
from behavclon.common import ControlCommandMarkup
from .bev_mask import BEVMaskMarkup, BEVMaskPredicted
from .cuboid import CuboidMarkup, CuboidPredicted
from mlengine.common.type import FrameMarkupObject, FramePredictionObject

OBJECTS_TYPE_MAPPING: dict[str, Type[FrameMarkupObject]] = {
    "bev_mask": BEVMaskMarkup,
    "cuboid": CuboidMarkup,
    "control_command": ControlCommandMarkup,
}

__all__ = [
    "BEVMaskMarkup",
    "BEVMaskPredicted",
    "ControlCommandMarkup",
    "CuboidMarkup",
    "CuboidPredicted",
    "FrameMarkupObject",
    "FramePredictionObject",
    "OBJECTS_TYPE_MAPPING",
]
