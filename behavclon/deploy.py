from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import onnx
import torch
from torch import Tensor, nn

from behavclon.model import BehaviourCloneModel, ImgSize


@dataclass
class BehaviourCloneDeployCfg:
    model: BehaviourCloneModel
    ckpt: str
    output_path: str | None = None
    opset_version: int = 17


class _OnnxExportWrapper(nn.Module):
    def __init__(self, model: BehaviourCloneModel) -> None:
        super().__init__()
        self.model = model

    def forward(self, images: Tensor) -> Tensor:
        return self.model(images)


class BehaviourCloneDeploy:
    """Export a trained behaviour-clone checkpoint to ONNX for robot inference."""

    def __init__(self, cfg: BehaviourCloneDeployCfg) -> None:
        self.cfg = cfg
        self.model = cfg.model

    def export(self) -> Path:
        ckpt_path = Path(self.cfg.ckpt)
        if not ckpt_path.is_file():
            raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")

        state = self._load_state_dict(ckpt_path)
        self.model.load_state_dict(state, strict=True)
        self.model.eval()

        img_size = self.model.img_size
        img_w, img_h = img_size
        in_ch = 3
        dummy_input = torch.randn(1, in_ch, img_h, img_w)

        output_path = (
            Path(self.cfg.output_path)
            if self.cfg.output_path is not None
            else ckpt_path.with_suffix(".onnx")
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)

        wrapper = _OnnxExportWrapper(self.model)
        torch.onnx.export(
            wrapper,
            dummy_input,
            str(output_path),
            input_names=["input"],
            output_names=["control"],
            opset_version=self.cfg.opset_version,
        )
        self._attach_metadata(output_path, img_size)
        print(f"exported ONNX: {output_path}")
        return output_path

    @staticmethod
    def _load_state_dict(ckpt_path: Path) -> dict[str, Tensor]:
        ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=True)
        state = ckpt.get("model", ckpt.get("state_dict", ckpt))
        if any(key.startswith("net.") for key in state):
            state = {key.removeprefix("net."): value for key, value in state.items()}
        return state

    @staticmethod
    def _attach_metadata(output_path: Path, img_size: ImgSize) -> None:
        img_w, img_h = img_size
        model = onnx.load(str(output_path))
        del model.metadata_props[:]
        for key, value in {
            "head_tanh": "true",
            "target_space_atanh": "false",
            "img_width": str(img_w),
            "img_height": str(img_h),
        }.items():
            prop = model.metadata_props.add()
            prop.key = key
            prop.value = value
        onnx.save(model, str(output_path))
