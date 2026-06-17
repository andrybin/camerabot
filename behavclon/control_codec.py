"""Control-command label codec shared by behaviour_recorder and behavclon."""

from __future__ import annotations

PROPAGATION_VALUE = {"F": 1.0, "N": 0.0, "B": -1.0}
TURN_VALUE = {"L": -1.0, "N": 0.0, "R": 1.0}
CONTROL_CODE_THRESHOLD = 0.5

TURN_CODE_FROM_VALUE = {v:k for k, v in TURN_VALUE.items()}

__all__ = [
    "PROPAGATION_VALUE",
    "TURN_VALUE",
    "CONTROL_CODE_THRESHOLD",
    "encode_control_code",
    "parse_control_code",
]


def encode_control_code(
    linear_x: float,
    angular_z: float,
    *,
    max_lin_speed: float,
    max_ang_speed: float,
    threshold: float = CONTROL_CODE_THRESHOLD,
) -> str:
    """Map cmd_vel components to a two-letter recorder label (e.g. FN, BL, BR)."""
    if max_lin_speed <= 0.0 or max_ang_speed <= 0.0:
        raise ValueError("max_lin_speed and max_ang_speed must be positive")

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
