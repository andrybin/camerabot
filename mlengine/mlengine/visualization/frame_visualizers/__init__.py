from typing import Protocol

from mlengine.common.type import Frame
from .frame_bev import FrameBEVVisualizer, FrameBEVVisualizerCfg
from PIL import Image


__all__ = [
    "FrameVisualizer",
    "FrameBEVVisualizer",
    "FrameBEVVisualizerCfg",
]

class FrameVisualizer(Protocol):
    def visualize_frame(self, frame: Frame) -> Image.Image: ...
