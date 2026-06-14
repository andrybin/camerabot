from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import TypeAlias

import numpy as np
import torch
from mlengine.common.type import Markup, Targets
from mlengine.common.logging import setup_logger
from mlengine.dataset.augmentations import Augmentations
from mlengine.dataset.serializers import Serializer
from torch.utils.data import Dataset
from tqdm import tqdm

TInputs: TypeAlias = torch.Tensor
TTargets: TypeAlias = torch.Tensor
TSampleId: TypeAlias = str | int


logger = setup_logger("dataset:descrete_dataset_loader")

@dataclass
class DescreteSample:
    id: TSampleId
    inputs: list[str]
    targets: list[Targets]
    data: np.ndarray | None = None


@dataclass
class DescreateDatasetCfg:
    labels: list[str]
    data_root: str = ""
    markup_filename: str = "markup.json"
    data_serializer: Serializer | None = None
    augmentations: Augmentations | None = None
    cache_in_ram: bool = False


class DescreateDataset(Dataset):

    def __init__(self, cfg: DescreateDatasetCfg):
        self.labels = cfg.labels
        self.data_root = Path(cfg.data_root)
        self.markup_filename = cfg.markup_filename
        self.markup = self.load_markup()
        self.data_serializer = cfg.data_serializer
        self.augmentations = cfg.augmentations
        self.cache_in_ram = cfg.cache_in_ram
        self.samples: list[DescreteSample] = []
        self.prepare_samples()
        if self.cache_in_ram:
            self._preload_raw_data_to_ram()

    def load_markup(self) -> Markup:
        return Markup.from_file(self.data_root / self.markup_filename)

    def prepare_samples(self) -> None:
        self.samples = [
            DescreteSample(
                id=Path(frame.pcd_filename).stem,
                inputs=[frame.pcd_filename],
                targets=[obj for obj in frame.objects if obj.label in self.labels],
            )
            for frame in tqdm(self.markup.flatten_frames(), 
                              desc=f"Preparing samples from {self.markup_filename}")
        ]

    def _preload_raw_data_to_ram(self) -> None:
        total_bytes = 0
        for sample in self.samples:
            arr = self.data_serializer(sample.inputs[0]).load()
            sample.data = np.asarray(arr, dtype=np.float32)
            total_bytes += sample.data.nbytes
        n = len(self.samples)
        logger.info(f"Cached {n} raw data (~{total_bytes / 1e9:.2f} GB) in RAM")

    def load_inputs(self, sample: DescreteSample) -> TInputs:
        data = (sample.data
               if sample.data is not None
               else self.data_serializer(sample.inputs[0]).load())
        return np.asarray(data, dtype=np.float32)

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> tuple[TSampleId, TInputs, list[Targets]]:
        sample = self.samples[idx]
        inputs = self.load_inputs(sample)
        targets = copy.deepcopy(sample.targets)
        if self.augmentations is not None:
            inputs, targets = self.augmentations(inputs, targets).apply_all()
        return sample.id, torch.from_numpy(inputs), targets
