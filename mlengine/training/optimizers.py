from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import torch
from torch import nn
from torch.optim import Optimizer


@dataclass
class AdamWOptimizerCfg:
    lr: float = 1e-3
    weight_decay: float = 1e-4
    betas: tuple[float, float] = (0.9, 0.999)
    eps: float = 1e-8


class AdamWOptimizer:
    """Config-instantiable AdamW optimizer factory."""

    def __init__(self, cfg: AdamWOptimizerCfg):
        self.cfg = cfg

    def build(self, params: Iterable[nn.Parameter]) -> Optimizer:
        return torch.optim.AdamW(
            params,
            lr=self.cfg.lr,
            weight_decay=self.cfg.weight_decay,
            betas=self.cfg.betas,
            eps=self.cfg.eps,
        )
