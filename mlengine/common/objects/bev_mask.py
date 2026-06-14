from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Set

import numpy as np

from mlengine.common.type import FrameMarkupObject, FramePredictionObject
from mlengine.common.utils import rle_decode_delta_map, rle_decode_mask, rle_encode_delta_map, rle_encode_mask


@dataclass
class BEVMask:
    labels: list[str]

    def get_labels(self) -> Set[str]:
        return set(self.labels)

    @property
    def label(self) -> str:
        if not self.labels:
            raise ValueError("BEV mask has no labels")
        return self.labels[0]

    def to_array(self) -> np.ndarray:
        raise NotImplementedError

    @classmethod
    def flip_h(cls, mask: np.ndarray) -> np.ndarray:
        return np.flip(mask, axis=-1)

    @classmethod
    def flip_v(cls, mask: np.ndarray) -> np.ndarray:
        return np.flip(mask, axis=-2)


@dataclass
class BEVMaskMarkup(BEVMask, FrameMarkupObject):
    multiclass_mask: np.ndarray
    ground_elevation_map: np.ndarray
    bev_x_bounds: tuple[float, float]
    bev_y_bounds: tuple[float, float]

    def __post_init__(self) -> None:
        mask = np.asarray(self.multiclass_mask, dtype=np.uint8)
        assert mask.ndim == 2, "Expected mask (H, W)"
        if self.labels and mask.max() > len(self.labels):
            raise ValueError("Max value of mask must be less than or equal to number of labels")
        self.multiclass_mask = mask
        assert self.ground_elevation_map.shape[-2:] == self.multiclass_mask.shape[-2:]

    def multiclass_to_multilayer_mask(self) -> np.ndarray:
        layers = [
            (self.multiclass_mask == class_id).astype(np.float32)
            for class_id in range(1, len(self.labels) + 1)
        ]
        return np.stack(layers, axis=0)

    def to_array(self) -> np.ndarray:
        return self.multiclass_to_multilayer_mask()

    def flip_h(self) -> BEVMaskMarkup:
        self.multiclass_mask = BEVMask.flip_h(self.multiclass_mask)
        self.ground_elevation_map = BEVMask.flip_h(self.ground_elevation_map)
        x_min, x_max = self.bev_x_bounds
        self.bev_x_bounds = (-x_max, -x_min)
        return self

    def flip_v(self) -> BEVMaskMarkup:
        self.multiclass_mask = BEVMask.flip_v(self.multiclass_mask)
        self.ground_elevation_map = BEVMask.flip_v(self.ground_elevation_map)
        y_min, y_max = self.bev_y_bounds
        self.bev_y_bounds = (-y_max, -y_min)
        return self

    def clear_mask_by_label(self, label: str) -> BEVMaskMarkup:
        if label not in self.labels:
            raise ValueError(f"Label {label} not found in {self.labels}")
        self.multiclass_mask[self.multiclass_mask == self.labels.index(label) + 1] = 0
        return self

    def segment_pcd_by_label(self, pcd: np.ndarray, label: str) -> set[int]:
        if label not in self.labels:
            raise ValueError(f"Label {label} not found in {self.labels}")

        class_id = self.labels.index(label) + 1
        pix_mask = self.multiclass_mask == class_id

        x_min, x_max = self.bev_x_bounds
        y_min, y_max = self.bev_y_bounds
        num_rows, num_cols = self.multiclass_mask.shape

        in_bounds = (
            (pcd[:, 0] >= x_min)
            & (pcd[:, 0] < x_max)
            & (pcd[:, 1] >= y_min)
            & (pcd[:, 1] < y_max)
        )
        if not np.any(in_bounds):
            return set()

        x_span = x_max - x_min
        y_span = y_max - y_min
        cols = ((pcd[in_bounds, 0] - x_min) / x_span * num_cols).astype(np.int32)
        rows = ((y_max - pcd[in_bounds, 1]) / y_span * num_rows).astype(np.int32)
        cols = np.clip(cols, 0, num_cols - 1)
        rows = np.clip(rows, 0, num_rows - 1)

        point_indices = np.flatnonzero(in_bounds)
        masked = pix_mask[rows, cols]
        return set(point_indices[masked].tolist())

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> BEVMaskMarkup:
        data = dict(data)
        data.pop("type", None)
        return cls(
            labels=data["labels"],
            multiclass_mask=rle_decode_mask(data.pop("multiclass_mask")),
            ground_elevation_map=rle_decode_delta_map(data.pop("ground_elevation_map")),
            bev_x_bounds=tuple(data.pop("bev_x_bounds")),
            bev_y_bounds=tuple(data.pop("bev_y_bounds")),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": "bev_mask",
            "labels": self.labels,
            "multiclass_mask": rle_encode_mask(self.multiclass_mask),
            "ground_elevation_map": rle_encode_delta_map(self.ground_elevation_map),
            "bev_x_bounds": list(self.bev_x_bounds),
            "bev_y_bounds": list(self.bev_y_bounds),
        }


@dataclass
class BEVMaskPredicted(BEVMask, FramePredictionObject):
    multilayer_probability_mask: np.ndarray

    def __post_init__(self) -> None:
        mask = np.asarray(self.multilayer_probability_mask, dtype=np.float32)
        if mask.ndim == 2:
            mask = mask[np.newaxis, ...]
        if mask.ndim != 3:
            raise ValueError(f"Expected mask (L, H, W) or (H, W), got shape {mask.shape}")
        self.multilayer_probability_mask = mask

        if len(self.labels) != mask.shape[0]:
            raise ValueError(
                f"Number of labels {len(self.labels)} does not match number of layers {mask.shape[0]}"
            )
        if len(self.labels) != len(set(self.labels)):
            raise ValueError("Labels must be unique")

        if mask.shape[0] > 1:
            summed = mask.sum(axis=0)
            if not np.allclose(summed, 1.0, atol=1e-3, rtol=1e-5):
                raise ValueError("Per-pixel layer probabilities must sum to 1 for multi-class masks")
        elif ((mask < -1e-6) | (mask > 1.0 + 1e-6)).any():
            raise ValueError("Single-layer probabilities must lie in [0, 1]")

    def get_mask_for_label(self, label: str) -> np.ndarray:
        return self.multilayer_probability_mask[self.labels.index(label)]

    def to_array(self) -> np.ndarray:
        return self.multilayer_probability_mask

    def flip_h(self) -> BEVMaskPredicted:
        self.multilayer_probability_mask = BEVMask.flip_h(self.multilayer_probability_mask)
        return self

    def flip_v(self) -> BEVMaskPredicted:
        self.multilayer_probability_mask = BEVMask.flip_v(self.multilayer_probability_mask)
        return self
