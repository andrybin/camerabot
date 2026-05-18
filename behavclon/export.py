import argparse
import torch
import torch.nn as nn
from pathlib import Path
from behavclon.train import build_model


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Export behaviour clone model to ONNX.")
    p.add_argument("--ckpt", type=Path, required=True, help="Path to model checkpoint")
    p.add_argument("--output_path", type=Path, default=None, help="Path to output ONNX model")
    args = p.parse_args()

    ckpt = torch.load(args.ckpt, map_location="cpu", weights_only=True)
    state = ckpt.get("state_dict", ckpt)
    in_ch = int(ckpt.get("in_channels", 3 * (ckpt.get("num_past", 3) + 1)))
    img_size = int(ckpt.get("img_size", 128))
    model = build_model(in_channels=in_ch, probe_size=img_size)
    model.load_state_dict(state, strict=True)
    model.eval()

    dummy_input = torch.randn((1, in_ch, img_size, img_size))
    torch.onnx.export(model, dummy_input, str(args.output_path or args.ckpt.with_suffix(".onnx")))
