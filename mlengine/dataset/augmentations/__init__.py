from typing import Protocol
from .pcd import PCDAugmentations, PCDAugmentationsCfg

__all__ = ["PCDAugmentations", "PCDAugmentationsCfg"]

class Augmentations(Protocol):
    def __call__(self, data): ...
    def apply_all(self): ...