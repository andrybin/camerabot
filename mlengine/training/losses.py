from collections.abc import Iterable
from dataclasses import dataclass

import torch
import torch.nn.functional as F
from torch import nn


@dataclass
class LossCfg:
    loss_names: list[str]
    loss_weights: list[float]


@dataclass
class WeightsRegularizationCfg:
    regularization_names: list[str]
    regularization_weights: list[float] | None = None


class Loss(nn.Module):
    def __init__(self, cfg: LossCfg):
        super().__init__()
        self.cfg = cfg
        self.losses = [getattr(self, loss_name) for loss_name in self.cfg.loss_names]
        self.loss_weights = self.cfg.loss_weights

    def binary_cross_entropy(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        return F.binary_cross_entropy_with_logits(logits, targets)

    def mse(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        return F.mse_loss(logits, targets)

    def forward(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        return sum(loss(logits, targets) * weight for loss, weight in zip(self.losses, self.loss_weights))


class WeightsRegularization(nn.Module):
    def __init__(self, cfg: WeightsRegularizationCfg):
        super().__init__()
        self.cfg = cfg
        self.regularizations = [
            getattr(self, regularization_name) for regularization_name in self.cfg.regularization_names
        ]
        self.regularization_weights = self.cfg.regularization_weights or [0.0] * len(self.regularizations)
        self._params: list[nn.Parameter] = []

    def set_params(self, params: Iterable[nn.Parameter]) -> None:
        self._params = [param for param in params if param.requires_grad]

    def l2_norm(self) -> torch.Tensor:
        if not self._params:
            return torch.zeros(())
        return sum(param.float().pow(2).sum() for param in self._params)

    def forward(self) -> torch.Tensor:
        total = torch.zeros((), device=self._params[0].device if self._params else "cpu")
        for reg_fn, weight in zip(self.regularizations, self.regularization_weights):
            total = total + reg_fn() * weight
        return total
