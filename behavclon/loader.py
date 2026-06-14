from __future__ import annotations

import copy
import random
from dataclasses import dataclass
from pathlib import Path
from typing import TypeAlias

import torch
from torch.utils.data import Dataset
from torchvision import transforms

from behavclon.augmentation import ImageAugmentations
from behavclon.common import ControlCommandMarkup
from mlengine.common.type import Markup
from mlengine.dataset.serializers import ImageFileSerializer

ImgSize: TypeAlias = tuple[int, int]


@dataclass
class BehaviourCloneSample:
    id: str
    image_filename: str
    target: ControlCommandMarkup


@dataclass
class BehaviourCloneDatasetCfg:
    data_dir: str
    img_size: list[int] | ImgSize = (128, 64)
    split: str = "train"
    val_frac: float = 0.1
    seed: int = 0
    markup_filename: str = "markup.json"
    image_serializer: ImageFileSerializer | None = None
    augmentations: ImageAugmentations | None = None


class BehaviourCloneDataset(Dataset):
    """Load behaviour-clone samples from markup.json + image serializer."""

    def __init__(self, cfg: BehaviourCloneDatasetCfg):
        self.cfg = cfg
        self.data_dir = Path(cfg.data_dir)
        if isinstance(cfg.img_size, list):
            self.img_size: ImgSize = (int(cfg.img_size[0]), int(cfg.img_size[1]))
        else:
            self.img_size = tuple(cfg.img_size)
        self.split = cfg.split
        self.val_frac = cfg.val_frac
        self.seed = cfg.seed
        self.markup_filename = cfg.markup_filename
        self.augmentations = cfg.augmentations
        if cfg.image_serializer is None:
            raise ValueError("image_serializer is required")
        self.image_serializer = cfg.image_serializer
        img_w, img_h = self.img_size
        self.transform = transforms.Compose(
            [
                transforms.Resize((img_h, img_w)),
                transforms.ToTensor(),
            ]
        )
        self.scene_frames: list[list[tuple[str, ControlCommandMarkup]]] = []
        self.samples: list[BehaviourCloneSample] = []
        self.prepare_samples()

    def load_markup(self) -> Markup:
        return Markup.from_file(self.data_dir / self.markup_filename)

    def prepare_samples(self) -> None:
        markup = self.load_markup()
        self.scene_frames = []
        for scene in markup.scenes:
            frames: list[tuple[str, ControlCommandMarkup]] = []
            for frame in scene.frames:
                if frame.image_filename is None:
                    continue
                control = next(
                    (obj for obj in frame.objects if isinstance(obj, ControlCommandMarkup)),
                    None,
                )
                if control is None:
                    continue
                frames.append((frame.image_filename, control))
            if frames:
                self.scene_frames.append(frames)

        all_indices: list[tuple[int, int]] = []
        for scene_idx, frames in enumerate(self.scene_frames):
            for frame_idx in range(len(frames)):
                all_indices.append((scene_idx, frame_idx))

        rng = random.Random(self.seed)
        rng.shuffle(all_indices)
        val_count = int(len(all_indices) * self.val_frac)
        val_set = set(all_indices[:val_count])

        self.samples = []
        for scene_idx, frame_idx in all_indices:
            is_val = (scene_idx, frame_idx) in val_set
            if self.split == "val" and not is_val:
                continue
            if self.split == "train" and is_val:
                continue
            image_filename, target = self.scene_frames[scene_idx][frame_idx]
            self.samples.append(
                BehaviourCloneSample(
                    id=Path(image_filename).stem,
                    image_filename=image_filename,
                    target=target,
                )
            )

    def _load_image_tensor(self, image_filename: str) -> torch.Tensor:
        image = self.image_serializer(image_filename).load()
        return self.transform(image.convert(self.image_serializer.cfg.mode))

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> tuple[str, torch.Tensor, list[ControlCommandMarkup]]:
        sample = self.samples[idx]
        image = self._load_image_tensor(sample.image_filename)
        targets = [copy.deepcopy(sample.target)]

        if self.augmentations is not None:
            image, targets = self.augmentations(image, targets).apply_all()

        return sample.id, image, targets
