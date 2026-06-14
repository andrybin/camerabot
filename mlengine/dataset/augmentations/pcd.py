from dataclasses import dataclass, fields
from typing import List
import random
import numpy as np

from mlengine.common.type import Targets


@dataclass
class PCDAugmentationsCfg:
    keep_columns: List[int] | None = None
    crop_y_bounds: tuple[float, float] | None = None
    crop_x_bounds: tuple[float, float] | None = None
    crop_z_bounds: tuple[float, float] | None = None
    flip_h: float | None = None
    flip_v: float | None = None
    remove_masked_pts_by_label: dict[str, str | float] | None = None


class PCDAugmentations:
    def __init__(self, cfg: PCDAugmentationsCfg):
        self._x_bounds = cfg.crop_x_bounds
        self._y_bounds = cfg.crop_y_bounds
        self.augmentations = []
        for cfg_field in fields(cfg):
            params = getattr(cfg, cfg_field.name)
            if params is None:
                continue
            method = getattr(self, cfg_field.name, None)
            if method is not None:
                self.augmentations.append((method, params))
        self._pcd = None
        self._targets: list[Targets] | None = None

    def __call__(self, pcd: np.ndarray, targets: list[Targets] | None = None) -> "PCDAugmentations":
        self._pcd = pcd
        self._targets = targets
        if self._targets is not None:
            assert all(isinstance(target, Targets) for target in self._targets), "Targets must be a list of Targets"
        return self

    def _aug_targets(self, aug_method: str, kwargs: dict = None):
        if kwargs is None:
            kwargs = {}
        if self._targets is None:
            return
        self._targets = [getattr(target, aug_method)(**kwargs) for target in self._targets]

    def apply_all(self) -> np.ndarray:
        if self._pcd is None:
            raise ValueError("PCD is not set. Call __call__ first.")
        for aug_method, params in self.augmentations:
            if isinstance(params, dict):
                aug_method(**params)
            else:
                aug_method(params)
        return self.output()

    def output(self) -> np.ndarray:
        return self._pcd, self._targets

    def keep_columns(self, columns: List[int]) -> "PCDAugmentations":
        self._pcd = self._pcd[:, columns]
        return self

    def crop_z_bounds(self, z_bounds: tuple[float, float]) -> "PCDAugmentations":
        mask = (self._pcd[:, 2] >= z_bounds[0]) & (self._pcd[:, 2] <= z_bounds[1])
        self._pcd = self._pcd[mask]
        return self

    def crop_y_bounds(self, y_bounds: tuple[float, float]) -> "PCDAugmentations":
        mask = (self._pcd[:, 1] >= y_bounds[0]) & (self._pcd[:, 1] <= y_bounds[1])
        self._pcd = self._pcd[mask]
        return self

    def crop_x_bounds(self, x_bounds: tuple[float, float]) -> "PCDAugmentations":
        mask = (self._pcd[:, 0] >= x_bounds[0]) & (self._pcd[:, 0] <= x_bounds[1])
        self._pcd = self._pcd[mask]
        return self

    def _mirror_pcd_axis(self, axis: int) -> None:
        self._pcd[:, axis] = -self._pcd[:, axis]

    def flip_h(self, prob: float = 0.5) -> "PCDAugmentations":
        if random.random() < prob:
            self._mirror_pcd_axis(0)
            self._aug_targets("flip_h")
        return self

    def flip_v(self, prob: float = 0.5) -> "PCDAugmentations":
        if random.random() < prob:
            self._mirror_pcd_axis(1)
            self._aug_targets("flip_v")
        return self

    def remove_masked_pts_by_label(self, label: str, prob: float = 0.5) -> "PCDAugmentations":
        if random.random() < prob:
            mask = np.zeros((self._pcd.shape[0],), dtype=np.uint8)
            for target in self._targets:
                segment = getattr(target, "segment_pcd_by_label", None)
                if segment is None:
                    continue
                point_ids = segment(self._pcd, label)
                mask[list(point_ids)] = 1
            self._pcd = self._pcd[mask == 0]
            self._aug_targets("clear_mask_by_label", {"label": label})
        return self