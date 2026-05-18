#!/usr/bin/env python3
"""Behaviour-cloning train loop: stacked RGB frames -> each R,G,B plane through shared frozen MobileNetV3-small -> concat pooled feats -> cmd in [-1,1]."""

from __future__ import annotations

import argparse
import io
import os
import re
import zipfile
from pathlib import Path

import torch
import torch.nn as nn
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms
import torchvision.transforms.functional as TF
from torchvision.models import mobilenet_v3_small

try:
    from torchvision.models import MobileNet_V3_Small_Weights
except ImportError:  # torchvision < 0.13
    MobileNet_V3_Small_Weights = None  # type: ignore[misc, assignment]

# Main JPEG from behaviour_recorder: {stamp_ns}_{lx_pct}_{wz_pct}.jpg
_LABEL_RE = re.compile(r"^(\d+)_(-?\d+)_(-?\d+)\.jpg$")
# Zip member: {stamp_ns}.jpg or {stamp_ns}_{n}.jpg
_ZIP_MEMBER_RE = re.compile(r"^(\d+)(?:_(\d+))?\.jpg$")


def _zip_member_sort_key(name: str) -> tuple[int, int]:
    base = Path(name).name
    m = _ZIP_MEMBER_RE.match(base)
    if not m:
        return (0, 0)
    ts = int(m.group(1))
    suf = int(m.group(2) or 0)
    return (ts, suf)


def _load_pil_rgb(data: bytes) -> Image.Image:
    im = Image.open(io.BytesIO(data)).convert("RGB")
    return im


def _build_past_tensors(
    past_rgbs: list[Image.Image],
    num_past: int,
    current: Image.Image,
    tfm: transforms.Compose,
) -> list[torch.Tensor]:
    """Return list of num_past+1 tensors (3,H,W), oldest past first, then current."""
    cur_t = tfm(current)
    past_chw = [tfm(p) for p in past_rgbs]
    tail = past_chw[-num_past:] if past_chw else []
    out = list(tail)
    while len(out) < num_past:
        oldest = out[0] if out else cur_t
        out.insert(0, oldest.clone())
    out.append(cur_t)
    return out


class BehaviourCloneDataset(Dataset):
    def __init__(self, data_dir: Path, num_past: int, img_size: int):
        self.data_dir = Path(data_dir)
        self.num_past = num_past
        self.paths = sorted(self.data_dir.glob("*.jpg"))
        self.tfm = transforms.Compose(
            [
                transforms.Resize((img_size, img_size)),
                transforms.ToTensor(),
            ]
        )

    def __len__(self) -> int:
        return len(self.paths)

    def __getitem__(self, idx: int) -> tuple[torch.Tensor, torch.Tensor]:
        path = self.paths[idx]
        m = _LABEL_RE.match(path.name)
        if not m:
            raise ValueError(f"Bad filename (expected stamp_lx_wz.jpg): {path.name}")
        stamp_ns = m.group(1)
        lx = float(m.group(2)) / 100.0
        wz = float(m.group(3)) / 100.0
        y = torch.tensor([lx, wz], dtype=torch.float32).clamp(-1.0, 1.0)

        current = Image.open(path).convert("RGB")

        zip_path = self.data_dir / f"{stamp_ns}.zip"
        past_rgbs: list[Image.Image] = []
        if zip_path.is_file():
            with zipfile.ZipFile(zip_path, "r") as zf:
                names = sorted(
                    [n for n in zf.namelist() if n.endswith(".jpg")],
                    key=_zip_member_sort_key,
                )
                for n in names:
                    past_rgbs.append(_load_pil_rgb(zf.read(n)))

        stacked = _build_past_tensors(past_rgbs, self.num_past, current, self.tfm)
        x = torch.cat(stacked, dim=0)
        return x, y


class Augmentations:
    """Horizontal flip on stacked (B,C,H,W); per-sample: if flipped, negate label (mirror cmd)."""

    def __init__(self, p: float = 0.5):
        self.p = p

    def __call__(self, x: torch.Tensor, y: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        b = x.shape[0]
        mask = torch.rand(b, device=x.device, dtype=torch.float32) < self.p
        if not bool(mask.any()):
            return x, y
        m4 = mask[:, None, None, None]
        x = torch.where(m4, TF.hflip(x), x)
        y[:, 1] = torch.where(mask, -y[:, 1], y[:, 1])
        return x, y

class StackedFrameBehaviourNet(nn.Module):
    """Shared MobileNetV3-small features per pseudo-RGB, concat maps, 1x1 fuse conv, global pool, linear head."""

    def __init__(self, in_channels: int, probe_size: int = 128):
        super().__init__()
        if in_channels % 3 != 0:
            raise ValueError(f"in_channels ({in_channels}) must be a multiple of 3 (stacked RGB).")
        self.num_frames = in_channels // 3

        backbone = mobilenet_v3_small(pretrained=False)
        # for p in backbone.features.parameters():
        #     p.requires_grad = False

        with torch.no_grad():
            dummy = torch.zeros(1, 3, probe_size, probe_size)
            feat = backbone.avgpool(backbone.features(dummy))
            self._fd = int(feat.numel())

        self.features = backbone.features
        self.avgpool = backbone.avgpool
        concat_c = self.num_frames * self._fd
        self.fuse = nn.Conv2d(concat_c, self._fd, kernel_size=1, bias=True)
        self.head = nn.Sequential(nn.Linear(self._fd, 2), nn.Tanh())

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Stacked RGB frames -> concat feature maps -> 1x1 fuse -> pool -> head."""
        b, _, h, w = x.shape
        t = self.num_frames
        xf = x.view(b, t, 3, h, w)
        feats = []
        for ti in range(t):
            rgb = xf[:, ti, :, :, :]
            feats.append(self.features(rgb))
        z = torch.cat(feats, dim=1)
        z = self.fuse(z)
        z = self.avgpool(z).flatten(1)
        return self.head(z)


def build_model(in_channels: int, probe_size: int = 128) -> nn.Module:
    return StackedFrameBehaviourNet(in_channels, probe_size=probe_size)


def main() -> None:
    from tqdm import tqdm

    p = argparse.ArgumentParser(description="Train behaviour clone on behaviour_recorder data.")
    p.add_argument("--data_dir", type=Path, required=True, help="Directory with *.jpg and optional *.zip")
    p.add_argument("--epochs", type=int, default=5)
    p.add_argument("--init_ckpt", type=Path, default=None, help="Optional path to initial checkpoint")
    p.add_argument("--batch_size", type=int, default=32)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--num_past", type=int, default=3, help="Past frames from zip (plus current).")
    p.add_argument("--img_size", type=int, default=128)
    p.add_argument("--device", type=str, default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--save", type=Path, default=None, help="Optional path to save state_dict")
    args = p.parse_args()

    ds = BehaviourCloneDataset(args.data_dir, num_past=args.num_past, img_size=args.img_size)
    if len(ds) == 0:
        raise SystemExit(f"No *.jpg in {args.data_dir}")

    in_ch = 3 * (args.num_past + 1)
    model = build_model(in_ch, probe_size=args.img_size)
    if args.init_ckpt is not None:
        missing_keys, unexpected_keys = model.load_state_dict(torch.load(args.init_ckpt, map_location=args.device)["state_dict"], strict=False)
        if missing_keys:
            print(f"missing keys: {missing_keys}")
        if unexpected_keys:
            print(f"unexpected keys: {unexpected_keys}")
        print(f"loaded {args.init_ckpt}")
    model = model.to(args.device)

    loader = DataLoader(ds, batch_size=args.batch_size, shuffle=True, num_workers=0)
    trainable = [p for p in model.parameters() if p.requires_grad]
    opt = torch.optim.Adam(trainable, lr=args.lr)
    loss_fn = nn.MSELoss()
    augment = Augmentations()

    for epoch in range(args.epochs):
        model.train()
        total = 0.0
        n_batches = 0
        for xb, yb in tqdm(loader, desc=f"epoch {epoch+1}/{args.epochs}", total=len(loader)):
            xb = xb.to(args.device)
            yb = yb.to(args.device)
            xb, yb = augment(xb, yb)
            opt.zero_grad()
            pred = model(xb)
            loss = loss_fn(pred, yb)
            loss.backward()
            opt.step()
            total += float(loss.item())
            n_batches += 1
        print(f"epoch {epoch+1}/{args.epochs} mean_loss {total / max(n_batches, 1):.4f}")

        if args.save is not None:
            args.save.parent.mkdir(parents=True, exist_ok=True)
            torch.save(
                {
                    "state_dict": model.state_dict(),
                    "num_past": args.num_past,
                    "img_size": args.img_size,
                    "in_channels": in_ch,
                    "head_tanh": True,
                    "use_rgb_stem": False,
                    "per_channel_backbone": True,
                },
                args.save,
            )
            print(f"saved {args.save}")

if __name__ == "__main__":
    main()
