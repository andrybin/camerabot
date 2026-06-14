from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Type

import numpy as np

from mlengine.common.type import FrameMarkupObject, FramePredictionObject

PROPAGATION_VALUE = {"F": 1.0, "N": 0.0}
TURN_VALUE = {"L": -1.0, "N": 0.0, "R": 1.0}

__all__ = [
    "ControlCommandMarkup",
    "ControlCommandPredicted",
    "OBJECT_TYPE_MAPPING",
]


@dataclass
class ControlCommandMarkup(FrameMarkupObject):
    propagation: str
    turn: str

    def to_numpy(self) -> np.ndarray:
        return np.array(
            [PROPAGATION_VALUE[self.propagation], TURN_VALUE[self.turn]],
            dtype=np.float32,
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": "control_command",
            "propagation": self.propagation,
            "turn": self.turn,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> ControlCommandMarkup:
        return cls(
            propagation=data["propagation"],
            turn=data["turn"],
        )

    def flip_h(self) -> ControlCommandMarkup:
        if self.turn == "L":
            self.turn = "R"
        elif self.turn == "R":
            self.turn = "L"
        return self


@dataclass
class ControlCommandPredicted(FramePredictionObject):
    propagation: float
    turn: float

    def to_numpy(self) -> np.ndarray:
        return np.array([self.propagation, self.turn], dtype=np.float32)


OBJECT_TYPE_MAPPING: dict[str, Type[FrameMarkupObject]] = {
    "control_command": ControlCommandMarkup,
}
