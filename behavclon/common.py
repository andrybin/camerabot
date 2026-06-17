from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Type
import numpy as np
from mlengine.common.type import FrameMarkupObject, FramePredictionObject


PROPAGATION_VALUE = {"F": 1.0, "N": 0.0, "B": -1.0}
TURN_VALUE = {"L": -1.0, "N": 0.0, "R": 1.0}
CONTROL_CODE_THRESHOLD = 0.

TURN_CODE_FROM_VALUE = {v:k for k, v in TURN_VALUE.items()}


def encode_control_code(linear_x: float,
                        angular_z: float,
                        max_lin_speed: float,
                        max_ang_speed: float,
                        threshold: float = CONTROL_CODE_THRESHOLD
                        ) -> str:
    lin_norm = linear_x / max_lin_speed
    ang_norm = angular_z / max_ang_speed
    if lin_norm > threshold:
        propagation = "F"
    elif lin_norm < -threshold:
        propagation = "B"
    else:
        propagation = "N"
    if ang_norm < -threshold:
        turn = TURN_CODE_FROM_VALUE[-1]
    elif ang_norm > threshold:
        turn = TURN_CODE_FROM_VALUE[1]
    else:
        turn = "N"
    return f"{propagation}{turn}"


def parse_control_code(code: str) -> tuple[str, str]:
    """Parse recorder label into propagation and turn symbols."""
    if len(code) != 2:
        raise ValueError(f"Invalid control code: {code!r}")
    propagation, turn = code[0], code[1]
    if propagation not in PROPAGATION_VALUE:
        raise ValueError(f"Unknown propagation symbol: {propagation!r}")
    if turn not in TURN_VALUE:
        raise ValueError(f"Unknown turn symbol: {turn!r}")
    return propagation, turn


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
