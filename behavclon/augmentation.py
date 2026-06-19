from __future__ import annotations

import copy
import random
from dataclasses import asdict, dataclass, fields, is_dataclass

import cv2
import numpy as np

from behavclon.common import ControlCommandMarkup
from mlengine.common.type import Targets


def resize_longer_edge_center_crop_hwc(
    rgb: np.ndarray, img_w: int, img_h: int
) -> np.ndarray:
    """RGB uint8 HWC. Scale so max(H,W)==max(img_w,img_h), center-crop to img_h x img_w."""
    max_side = max(img_w, img_h)
    h, w = rgb.shape[:2]
    scale = max_side / max(h, w)
    new_w, new_h = round(w * scale), round(h * scale)
    resized = cv2.resize(rgb, (new_w, new_h), interpolation=cv2.INTER_AREA)
    if new_w < img_w or new_h < img_h:
        scale2 = max(img_w / new_w, img_h / new_h)
        new_w, new_h = round(new_w * scale2), round(new_h * scale2)
        resized = cv2.resize(resized, (new_w, new_h), interpolation=cv2.INTER_AREA)
    left, top = (new_w - img_w) // 2, (new_h - img_h) // 2
    return resized[top : top + img_h, left : left + img_w]


def preprocess_rgb_to_chw(rgb: np.ndarray, img_w: int, img_h: int) -> np.ndarray:
    cropped = resize_longer_edge_center_crop_hwc(rgb, img_w, img_h)
    return cropped.transpose(2, 0, 1).astype(np.float32) / 255.0


@dataclass
class BrightnessAugCfg:
    prob: float
    factor: list[float] | tuple[float, float] = (0.85, 1.15)


@dataclass
class ColorAugCfg:
    prob: float
    hue_delta: int = 10
    saturation_factor: list[float] | tuple[float, float] = (0.9, 1.1)


@dataclass
class ImageAugmentationsCfg:
    resize_center_crop: list[int] | tuple[int, int] | None = None
    flip_h: float | None = None
    brightness: BrightnessAugCfg | None = None
    color: ColorAugCfg | None = None

    def __post_init__(self) -> None:
        if isinstance(self.brightness, dict):
            self.brightness = BrightnessAugCfg(**self.brightness)
        if isinstance(self.color, dict):
            self.color = ColorAugCfg(**self.color)


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
        self._image: np.ndarray | None = None
        self._targets: list[ControlCommandMarkup] | None = None

    def __call__(
        self,
        image: np.ndarray,
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

    def apply_all(self) -> tuple[np.ndarray, list[ControlCommandMarkup] | None]:
        if self._image is None:
            raise ValueError("Image is not set. Call __call__ first.")
        for aug_method, params in self.augmentations:
            if is_dataclass(params):
                aug_method(**asdict(params))
            elif isinstance(params, dict):
                aug_method(**params)
            else:
                aug_method(params)
        return self.output()

    def output(self) -> tuple[np.ndarray, list[ControlCommandMarkup] | None]:
        return self._image, self._targets

    def resize_center_crop(
        self, img_size: list[int] | tuple[int, int]
    ) -> ImageAugmentations:
        img_w, img_h = int(img_size[0]), int(img_size[1])
        self._image = resize_longer_edge_center_crop_hwc(self._image, img_w, img_h)
        return self

    def flip_h(self, prob: float = 0.5) -> ImageAugmentations:
        if random.random() < prob:
            self._image = cv2.flip(self._image, 1)
            self._aug_targets("flip_h")
        return self

    def brightness(
        self,
        prob: float = 0.5,
        factor: tuple[float, float] | list[float] = (0.85, 1.15),
    ) -> ImageAugmentations:
        if random.random() < prob:
            scale = random.uniform(float(factor[0]), float(factor[1]))
            self._image = cv2.convertScaleAbs(self._image, alpha=scale, beta=0)
        return self

    def color(
        self,
        prob: float = 0.5,
        hue_delta: int = 10,
        saturation_factor: tuple[float, float] | list[float] = (0.9, 1.1),
    ) -> ImageAugmentations:
        if random.random() < prob:
            hsv = cv2.cvtColor(self._image, cv2.COLOR_RGB2HSV).astype(np.int16)
            hsv[:, :, 0] = (hsv[:, :, 0] + random.randint(-hue_delta, hue_delta)) % 180
            sat_scale = random.uniform(
                float(saturation_factor[0]), float(saturation_factor[1])
            )
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * sat_scale, 0, 255)
            self._image = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2RGB)
        return self
