from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from omegaconf import OmegaConf

from behavclon.dataset.loaders.behaviour_clone_dataset import ImgSize
from behavclon.models.stacked_frame_net import build_model


def _img_size_from_raw(raw) -> ImgSize:
    if isinstance(raw, int):
        return (int(raw), int(raw))
    return (int(raw[0]), int(raw[1]))


def _load_params_from_cfg(cfg_path: Path) -> dict:
    cfg = OmegaConf.to_container(OmegaConf.load(cfg_path), resolve=True)
    model_cfg = cfg["model"]
    num_past = int(model_cfg["num_past"])
    img_size = _img_size_from_raw(model_cfg["img_size"])
    return {
        "num_past": num_past,
        "img_size": img_size,
        "in_channels": 3 * (num_past + 1),
        "frozen_bacbone": bool(model_cfg.get("frozen_bacbone", False)),
    }


def _load_params_from_config_json(ckpt_path: Path) -> dict | None:
    config_path = ckpt_path.parent / "config.json"
    if not config_path.is_file():
        return None
    with open(config_path, encoding="utf-8") as f:
        cfg = json.load(f)
    model_cfg = cfg.get("model", {})
    if not model_cfg:
        return None
    num_past = int(model_cfg["num_past"])
    img_size = _img_size_from_raw(model_cfg["img_size"])
    return {
        "num_past": num_past,
        "img_size": img_size,
        "in_channels": 3 * (num_past + 1),
        "frozen_bacbone": bool(model_cfg.get("frozen_bacbone", False)),
    }


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Export behaviour clone model to ONNX.")
    p.add_argument("--ckpt", type=Path, required=True, help="Path to model checkpoint")
    p.add_argument("--cfg", type=Path, default=None, help="YAML config with model hyperparameters")
    p.add_argument("--output_path", type=Path, default=None, help="Path to output ONNX model")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    ckpt = torch.load(args.ckpt, map_location="cpu", weights_only=True)
    state = ckpt.get("model", ckpt.get("state_dict", ckpt))
    if any(key.startswith("net.") for key in state):
        state = {key.removeprefix("net."): value for key, value in state.items()}

    if args.cfg is not None:
        params = _load_params_from_cfg(args.cfg)
    elif "in_channels" in ckpt:
        params = {
            "num_past": int(ckpt.get("num_past", 3)),
            "img_size": _img_size_from_raw(ckpt.get("img_size", (128, 128))),
            "in_channels": int(ckpt["in_channels"]),
            "frozen_bacbone": bool(ckpt.get("frozen_bacbone", False)),
        }
    elif (params := _load_params_from_config_json(args.ckpt)) is not None:
        pass
    else:
        raise SystemExit(
            "Checkpoint has no architecture metadata. Pass --cfg or train with the new pipeline "
            "(saves config.json next to checkpoints)."
        )

    img_size: ImgSize = params["img_size"]
    in_ch = params["in_channels"]
    model = build_model(
        in_channels=in_ch,
        probe_size=img_size,
        frozen_bacbone=params["frozen_bacbone"],
    )
    model.load_state_dict(state, strict=True)
    model.eval()

    img_w, img_h = img_size
    dummy_input = torch.randn((1, in_ch, img_h, img_w))
    output_path = args.output_path or args.ckpt.with_suffix(".onnx")
    torch.onnx.export(model, dummy_input, str(output_path))
    print(f"exported ONNX: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
