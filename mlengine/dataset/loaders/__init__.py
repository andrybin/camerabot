from typing import Protocol

from .descreate_dataset import DescreateDataset, DescreateDatasetCfg, TInputs, TTargets, TSampleId


class DatasetLoader(Protocol):
    def prepare_samples(self) -> None: ...

    def __getitem__(self, idx: int) -> tuple[TSampleId, TInputs, TTargets]: ...


__all__ = ["DatasetLoader", "DescreateDataset", "DescreateDatasetCfg"]