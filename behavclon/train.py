#!/usr/bin/env python3
"""Train behaviour cloning: stacked RGB frames -> MobileNetV3 -> (lx, wz) in [-1, 1]."""

from __future__ import annotations

import argparse
import io
import re
import zipfile
from pathlib import Path

import torch
import torch.nn as nn
from PIL import Image
from torch.utils.data import DataLoader, Dataset, random_split
from torch.utils.tensorboard import SummaryWriter
from torchvision import transforms
import torchvision.transforms.functional as TF
from torchvision.models import mobilenet_v3_small

# Labelled frame: {stamp_ns}_{lx_pct}_{wz_pct}.jpg
_LABEL_RE = re.compile(r"^(\d+)_(-?\d+)_(-?\d+)\.jpg$")
# Past frames in zip: {stamp_ns}.jpg or {stamp_ns}_{n}.jpg
_ZIP_MEMBER_RE = re.compile(r"^(\d+)(?:_(\d+))?\.jpg$")


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------


def _zip_member_sort_key(name: str) -> tuple[int, int]:
    base = Path(name).name
    match = _ZIP_MEMBER_RE.match(base)
    if not match:
        return (0, 0)
    timestamp = int(match.group(1))
    suffix = int(match.group(2) or 0)
    return (timestamp, suffix)


def _load_pil_rgb(jpeg_bytes: bytes) -> Image.Image:
    return Image.open(io.BytesIO(jpeg_bytes)).convert("RGB")


def _linear_speed_from_path(path: Path) -> float:
    """Normalized linear speed lx in [-1, 1] from filename (recorder percent / 100)."""
    match = _LABEL_RE.match(path.name)
    if not match:
        raise ValueError(f"Expected stamp_lx_wz.jpg, got: {path.name}")
    return float(match.group(2)) / 100.0


def _parse_label(path: Path) -> tuple[str, torch.Tensor]:
    """Return stamp_ns and target [lx, wz] in [-1, 1] from filename."""
    match = _LABEL_RE.match(path.name)
    if not match:
        raise ValueError(f"Expected stamp_lx_wz.jpg, got: {path.name}")
    stamp_ns = match.group(1)
    lx = _linear_speed_from_path(path)
    wz = float(match.group(3)) / 100.0
    target = torch.tensor([lx, wz], dtype=torch.float32).clamp(-1.0, 1.0)
    return stamp_ns, target


def _load_past_frames(data_dir: Path, stamp_ns: str) -> list[Image.Image]:
    zip_path = data_dir / f"{stamp_ns}.zip"
    if not zip_path.is_file():
        return []
    past: list[Image.Image] = []
    with zipfile.ZipFile(zip_path, "r") as archive:
        names = sorted(
            (n for n in archive.namelist() if n.endswith(".jpg")),
            key=_zip_member_sort_key,
        )
        for name in names:
            past.append(_load_pil_rgb(archive.read(name)))
    return past


def _stack_frame_tensors(
    past_images: list[Image.Image],
    num_past: int,
    current: Image.Image,
    transform: transforms.Compose,
) -> torch.Tensor:
    """Build (num_past + 1) RGB tensors, pad missing past with oldest, concat channels."""
    current_t = transform(current)
    past_tensors = [transform(img) for img in past_images]
    frames = list(past_tensors[-num_past:] if past_tensors else [])
    while len(frames) < num_past:
        pad = frames[0] if frames else current_t
        frames.insert(0, pad.clone())
    frames.append(current_t)
    return torch.cat(frames, dim=0)


class BehaviourCloneDataset(Dataset):
    """One sample = stacked past+current frames and commanded (lx, wz)."""

    def __init__(
        self,
        data_dir: Path,
        num_past: int,
        img_size: int,
        max_linear_speed: float | None = None,
    ):
        self.data_dir = Path(data_dir)
        self.num_past = num_past
        all_paths = sorted(self.data_dir.glob("*.jpg"))
        if max_linear_speed is not None:
            self.frame_paths = [
                p
                for p in all_paths
                if abs(_linear_speed_from_path(p)) <= max_linear_speed
            ]
            dropped = len(all_paths) - len(self.frame_paths)
            if dropped:
                print(
                    f"filtered {dropped}/{len(all_paths)} samples "
                    f"with |lx| > {max_linear_speed}"
                )
        else:
            self.frame_paths = all_paths
        self.transform = transforms.Compose(
            [
                transforms.Resize((img_size, img_size)),
                transforms.ToTensor(),
            ]
        )

    def __len__(self) -> int:
        return len(self.frame_paths)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor]:
        path = self.frame_paths[index]
        stamp_ns, target = _parse_label(path)
        current = Image.open(path).convert("RGB")
        past = _load_past_frames(self.data_dir, stamp_ns)
        inputs = _stack_frame_tensors(past, self.num_past, current, self.transform)
        return inputs, target


class HorizontalFlipAugment:
    """Random hflip; negate angular velocity (wz) when mirrored."""

    def __init__(self, probability: float = 0.5):
        self.probability = probability

    def __call__(
        self, images: torch.Tensor, targets: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        batch_size = images.shape[0]
        flip = torch.rand(batch_size, device=images.device) < self.probability
        if not flip.any():
            return images, targets
        flip_mask = flip[:, None, None, None]
        images = torch.where(flip_mask, TF.hflip(images), images)
        targets[:, 1] = torch.where(flip, -targets[:, 1], targets[:, 1])
        return images, targets


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


class StackedFrameBehaviourNet(nn.Module):
    """One MobileNetV3 backbone per frame; fuse feature maps; predict (lx, wz)."""

    def __init__(self, in_channels: int, probe_size: int = 128):
        super().__init__()
        if in_channels % 3 != 0:
            raise ValueError(f"in_channels must be multiple of 3, got {in_channels}")
        self.num_frames = in_channels // 3

        backbone = mobilenet_v3_small(weights=None)
        with torch.no_grad():
            probe = torch.zeros(1, 3, probe_size, probe_size)
            feat_dim = backbone.avgpool(backbone.features(probe)).numel()

        self.features = backbone.features
        self.avgpool = backbone.avgpool
        self.feat_dim = int(feat_dim)
        fused_in = self.num_frames * self.feat_dim
        self.fuse = nn.Conv2d(fused_in, self.feat_dim, kernel_size=1, bias=True)
        self.head = nn.Sequential(nn.Linear(self.feat_dim, 2), nn.Tanh())

    def forward(self, stacked: torch.Tensor) -> torch.Tensor:
        batch, _, height, width = stacked.shape
        frames = stacked.view(batch, self.num_frames, 3, height, width)
        per_frame_feats = [self.features(frames[:, i]) for i in range(self.num_frames)]
        fused = self.fuse(torch.cat(per_frame_feats, dim=1))
        pooled = self.avgpool(fused).flatten(1)
        return self.head(pooled)


def build_model(in_channels: int, probe_size: int = 128) -> StackedFrameBehaviourNet:
    return StackedFrameBehaviourNet(in_channels, probe_size=probe_size)


# ---------------------------------------------------------------------------
# Training loop helpers
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train behaviour clone on behaviour_recorder data.",
    )
    parser.add_argument("--data_dir", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=5)
    parser.add_argument("--batch_size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--weight_decay", type=float, default=1e-3)
    parser.add_argument("--num_past", type=int, default=3)
    parser.add_argument("--img_size", type=int, default=128)
    parser.add_argument("--max_linear_speed", type=float, default=None)
    parser.add_argument("--device", type=str, default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--init_ckpt", type=Path, default=None)
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument("--val_frac", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--logdir",
        type=Path,
        default=None,
        help="TensorBoard directory (default: runs/<data_dir name>).",
    )
    return parser.parse_args()


def split_dataset(
    dataset: Dataset,
    val_fraction: float,
    seed: int,
) -> tuple[Dataset, Dataset | None, int, int]:
    total = len(dataset)
    if total <= 1:
        return dataset, None, total, 0

    n_val = max(1, int(total * val_fraction))
    n_train = total - n_val
    generator = torch.Generator().manual_seed(seed)
    train_set, val_set = random_split(dataset, [n_train, n_val], generator=generator)
    return train_set, val_set, n_train, n_val


def load_checkpoint(model: nn.Module, path: Path, device: str) -> None:
    state = torch.load(path, map_location=device)["state_dict"]
    missing, unexpected = model.load_state_dict(state, strict=False)
    if missing:
        print(f"checkpoint missing keys: {missing}")
    if unexpected:
        print(f"checkpoint unexpected keys: {unexpected}")
    print(f"loaded checkpoint {path}")


def save_checkpoint(
    path: Path,
    model: nn.Module,
    num_past: int,
    img_size: int,
    in_channels: int,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "state_dict": model.state_dict(),
            "num_past": num_past,
            "img_size": img_size,
            "in_channels": in_channels,
            "head_tanh": True,
            "use_rgb_stem": False,
            "per_channel_backbone": True,
        },
        path,
    )
    print(f"saved checkpoint {path}")


@torch.no_grad()
def validation_loss(
    model: nn.Module,
    loader: DataLoader,
    loss_fn: nn.Module,
    device: str,
) -> float:
    model.eval()
    total_loss = 0.0
    for images, targets in loader:
        images = images.to(device)
        targets = targets.to(device)
        predictions = model(images)
        total_loss += loss_fn(predictions, targets).item()
    return total_loss / max(len(loader), 1)


def train_one_epoch(
    model: nn.Module,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer,
    loss_fn: nn.Module,
    augment: HorizontalFlipAugment,
    device: str,
    writer: SummaryWriter,
    epoch: int,
) -> float:
    model.train()
    running_loss = 0.0
    global_step = epoch * len(loader)

    from tqdm import tqdm

    for images, targets in tqdm(loader, desc=f"epoch {epoch + 1}", total=len(loader)):
        images = images.to(device)
        targets = targets.to(device)
        images, targets = augment(images, targets)

        optimizer.zero_grad()
        predictions = model(images)
        loss = loss_fn(predictions, targets)
        loss.backward()
        optimizer.step()

        batch_loss = loss.item()
        running_loss += batch_loss
        writer.add_scalar("loss/train_batch", batch_loss, global_step)
        global_step += 1

    return running_loss / max(len(loader), 1)


def log_epoch(
    epoch: int,
    num_epochs: int,
    train_loss: float,
    val_loss: float | None,
    writer: SummaryWriter,
) -> None:
    # add_scalars groups loss/train and loss/val on one TensorBoard chart
    epoch_losses = {"train": train_loss}
    if val_loss is not None:
        epoch_losses["val"] = val_loss
    writer.add_scalars("loss", epoch_losses, epoch)

    if val_loss is not None:
        print(
            f"epoch {epoch + 1}/{num_epochs}  "
            f"train_loss={train_loss:.4f}  val_loss={val_loss:.4f}"
        )
    else:
        print(f"epoch {epoch + 1}/{num_epochs}  train_loss={train_loss:.4f}")


def main() -> None:
    args = parse_args()
    logdir = args.logdir or Path("runs") / args.data_dir.name

    dataset = BehaviourCloneDataset(
        args.data_dir,
        args.num_past,
        args.img_size,
        max_linear_speed=args.max_linear_speed,
    )
    if len(dataset) == 0:
        raise SystemExit(f"No *.jpg files in {args.data_dir}")

    train_set, val_set, n_train, n_val = split_dataset(dataset, args.val_frac, args.seed)
    print(f"dataset: {n_train} train, {n_val} val")

    in_channels = 3 * (args.num_past + 1)
    model = StackedFrameBehaviourNet(in_channels, probe_size=args.img_size).to(args.device)
    if args.init_ckpt is not None:
        load_checkpoint(model, args.init_ckpt, args.device)

    train_loader = DataLoader(
        train_set, batch_size=args.batch_size, shuffle=True, num_workers=0
    )
    val_loader = None
    if val_set is not None:
        val_loader = DataLoader(
            val_set, batch_size=args.batch_size, shuffle=False, num_workers=0
        )

    optimizer = torch.optim.Adam(
        (p for p in model.parameters() if p.requires_grad),
        lr=args.lr,
        weight_decay=args.weight_decay,
    )
    print(f"optimizer: Adam lr={args.lr} weight_decay={args.weight_decay}")
    loss_fn = nn.MSELoss()
    augment = HorizontalFlipAugment()

    logdir.mkdir(parents=True, exist_ok=True)
    writer = SummaryWriter(log_dir=str(logdir))
    print(f"tensorboard: {logdir}")

    try:
        for epoch in range(args.epochs):
            train_loss = train_one_epoch(
                model,
                train_loader,
                optimizer,
                loss_fn,
                augment,
                args.device,
                writer,
                epoch,
            )
            val_loss = None
            if val_loader is not None:
                val_loss = validation_loss(model, val_loader, loss_fn, args.device)
            log_epoch(epoch, args.epochs, train_loss, val_loss, writer)

            if args.save is not None:
                save_checkpoint(args.save, model, args.num_past, args.img_size, in_channels)
    finally:
        writer.close()


if __name__ == "__main__":
    main()
