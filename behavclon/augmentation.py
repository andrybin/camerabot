from __future__ import annotations

import copy
import random
from dataclasses import dataclass, fields

import torch
import torchvision.transforms.functional as TF

from behavclon.common import ControlCommandMarkup
from mlengine.common.type import Targets


@dataclass
class ImageAugmentationsCfg:
    flip_h: float | None = None


class ImageAugmentations:
    def __init__(self, cfg: ImageAugmentationsCfg):
        self.augmentations = []
        for cfg_field in fields(cfg):
            params = getattr(cfg, cfg_field.name)
            if params is None:
                continue
            method = getattr(self, cfg_field.name, None)
            if method is not None:
                self.augmentations.append((method, params))
        self._image: torch.Tensor | None = None
        self._targets: list[ControlCommandMarkup] | None = None

    def __call__(
        self,
        image: torch.Tensor,
        targets: list[ControlCommandMarkup] | None = None,
    ) -> ImageAugmentations:
        self._image = image
        self._targets = copy.deepcopy(targets) if targets is not None else None
        if self._targets is not None:
            assert all(isinstance(target, Targets) for target in self._targets), (
                "Targets must be a list of Targets"
            )
        return self

    def _aug_targets(self, aug_method: str, kwargs: dict | None = None) -> None:
        if kwargs is None:
            kwargs = {}
        if self._targets is None:
            return
        self._targets = [getattr(target, aug_method)(**kwargs) for target in self._targets]

    def apply_all(self) -> tuple[torch.Tensor, list[ControlCommandMarkup] | None]:
        if self._image is None:
            raise ValueError("Image is not set. Call __call__ first.")
        for aug_method, params in self.augmentations:
            if isinstance(params, dict):
                aug_method(**params)
            else:
                aug_method(params)
        return self.output()

    def output(self) -> tuple[torch.Tensor, list[ControlCommandMarkup] | None]:
        return self._image, self._targets

    def flip_h(self, prob: float = 0.5) -> ImageAugmentations:
        if random.random() < prob:
            self._image = TF.hflip(self._image)
            self._aug_targets("flip_h")
        return self
