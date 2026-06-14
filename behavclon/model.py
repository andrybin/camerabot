from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TypeAlias

import torch
from torch import Tensor, nn
from torchvision.models import EfficientNet_B0_Weights, efficientnet_b0

from behavclon.common import ControlCommandPredicted

ImgSize: TypeAlias = tuple[int, int]


@dataclass
class BehaviourCloneModelCfg:
    img_size: ImgSize | list[int]
    frozen_bacbone: bool = False
    init_ckpt: str | None = None


class BehaviourCloneModel(nn.Module):
    """RGB frame -> control command (propagation, turn) in [-1, 1]."""

    def __init__(self, cfg: BehaviourCloneModelCfg):
        super().__init__()
        self.cfg = cfg
        if isinstance(cfg.img_size, list):
            self.img_size: ImgSize = (int(cfg.img_size[0]), int(cfg.img_size[1]))
        else:
            self.img_size = tuple(cfg.img_size)

        probe_w, probe_h = self.img_size
        backbone = efficientnet_b0(weights=EfficientNet_B0_Weights.IMAGENET1K_V1)
        with torch.no_grad():
            probe = torch.zeros(1, 3, probe_h, probe_w)
            feat_dim = backbone.avgpool(backbone.features(probe)).numel()

        self.features = backbone.features
        if cfg.frozen_bacbone:
            for param in self.features.parameters():
                param.requires_grad = False
        self.avgpool = backbone.avgpool
        self.head = nn.Sequential(nn.Linear(int(feat_dim), 2), nn.Tanh())

        if cfg.init_ckpt:
            self._load_checkpoint(Path(cfg.init_ckpt))

    def _load_checkpoint(self, path: Path) -> None:
        ckpt = torch.load(path, map_location="cpu", weights_only=True)
        state = ckpt.get("model", ckpt.get("state_dict", ckpt))
        if any(key.startswith("net.") for key in state):
            state = {key.removeprefix("net."): value for key, value in state.items()}
        missing, unexpected = self.load_state_dict(state, strict=False)
        if missing:
            print(f"checkpoint missing keys: {missing}")
        if unexpected:
            print(f"checkpoint unexpected keys: {unexpected}")
        print(f"loaded checkpoint {path}")

    def forward(self, batch: list[Tensor] | Tensor) -> Tensor:
        images = batch if isinstance(batch, Tensor) else torch.stack(batch, dim=0)
        pooled = self.avgpool(self.features(images)).flatten(1)
        return self.head(pooled)

    def logits_to_predicted_objects(self, logits: Tensor) -> list[ControlCommandPredicted]:
        preds = logits.detach().cpu()
        if preds.ndim == 1:
            preds = preds.unsqueeze(0)
        return [
            ControlCommandPredicted(
                propagation=float(row[0].item()),
                turn=float(row[1].item()),
            )
            for row in preds
        ]
