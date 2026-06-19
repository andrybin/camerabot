import torch
import torch.nn.functional as F
from typing import Protocol


class Metric(Protocol):
    def __call__(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor: ...


class BaseMetric(Metric):
    def __str__(self) -> str:
        return self.__class__.__name__

class AngularDirectionAccuracy(BaseMetric):
    def __init__(self, threshold: float = 0.0):
        self.threshold = threshold

    @staticmethod
    def _direction_sign(values: torch.Tensor, threshold: float) -> torch.Tensor:
        return torch.where(
            values > threshold,
            values.new_tensor(1.0),
            torch.where(values < -threshold, values.new_tensor(-1.0), values.new_tensor(0.0)),
        )

    def __call__(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        preds = self._direction_sign(logits[:, 1], self.threshold)
        targets = self._direction_sign(targets[:, 1], self.threshold)
        return (preds == targets).float().mean()

class BinaryIOU(BaseMetric):
    def __init__(self, threshold: float = 0.5):
        self.threshold = threshold

    def __call__(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        preds = (torch.sigmoid(logits) >= self.threshold).float()
        targets = (targets >= self.threshold).float()
        intersection = (preds * targets).sum()
        union = ((preds + targets) > 0).float().sum()
        return (intersection + 1e-6) / (union + 1e-6)

class BinaryDice(BaseMetric):
    def __init__(self, threshold: float = 0.5):
        self.threshold = threshold

    def __call__(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        preds = (torch.sigmoid(logits) >= self.threshold).float()
        targets = (targets >= self.threshold).float()
        intersection = (preds * targets).sum()
        total = preds.sum() + targets.sum()
        return (2 * intersection + 1e-6) / (total + 1e-6)


class BinaryPrecision(BaseMetric):
    def __init__(self, threshold: float = 0.5):
        self.threshold = threshold

    def __call__(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        preds = (torch.sigmoid(logits) >= self.threshold).float()
        targets = (targets >= self.threshold).float()
        tp = (preds * targets).sum()
        fp = (preds * (1 - targets)).sum()
        return tp / (tp + fp + 1e-6)


class BinaryRecall(BaseMetric):
    def __init__(self, threshold: float = 0.5):
        self.threshold = threshold

    def __call__(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        preds = (torch.sigmoid(logits) >= self.threshold).float()
        targets = (targets >= self.threshold).float()
        tp = (preds * targets).sum()
        fn = ((1 - preds) * targets).sum()
        return tp / (tp + fn + 1e-6)


class MulticlassIOU(BaseMetric):
    def __init__(self, num_classes: int | None = None, ignore_index: int | None = -100, eps: float = 1e-6):
        self.num_classes = num_classes
        self.ignore_index = ignore_index
        self.eps = eps

    def __call__(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        """Macro-averaged mean IoU over classes that have at least one pixel in predict.

        logits: (N, C, …) class scores. targets: (N, …) integer labels, or (N, C, …) one-hot / soft
        labels (class taken as argmax along C).
        """
        if logits.dim() < 2:
            raise ValueError("multiclass_iou expects logits shaped (N, C, …).")
        c = self.num_classes if self.num_classes is not None else logits.shape[1]
        preds = logits.argmax(dim=1)
        if targets.shape == logits.shape:
            tgt = targets.argmax(dim=1)
        elif targets.dim() == logits.dim() - 1:
            tgt = targets.long()
        else:
            raise ValueError(
                "targets must be (N, …) integer labels or (N, C, …) matching logits; "
                f"got shape {tuple(targets.shape)}."
            )
        if self.ignore_index is not None:
            mask = tgt != self.ignore_index
        else:
            mask = torch.ones_like(tgt, dtype=torch.bool)

        preds_flat = preds[mask]
        tgt_flat = tgt[mask]
        if tgt_flat.numel() == 0:
            return logits.new_tensor(0.0)

        preds_oh = F.one_hot(preds_flat, c).to(dtype=logits.dtype)
        tgt_oh = F.one_hot(tgt_flat, c).to(dtype=logits.dtype)
        inter = (preds_oh * tgt_oh).sum(0)
        union = torch.maximum(preds_oh, tgt_oh).sum(0)
        iou = (inter + self.eps) / (union + self.eps)
        valid = union > 0
        if not bool(valid.any()):
            return logits.new_tensor(0.0)
        return iou[valid].mean()


class MulticlassF1(BaseMetric):
    def __init__(self, num_classes: int | None = None, ignore_index: int | None = -100, eps: float = 1e-6):
        self.num_classes = num_classes
        self.ignore_index = ignore_index
        self.eps = eps

    def _macro_f1(self, preds: torch.Tensor, tgt: torch.Tensor, num_classes: int) -> torch.Tensor:
        f1_scores: list[torch.Tensor] = []
        for cls in range(num_classes):
            pred_pos = preds == cls
            tgt_pos = tgt == cls
            tp = (pred_pos & tgt_pos).sum().to(dtype=torch.float32)
            fp = (pred_pos & ~tgt_pos).sum().to(dtype=torch.float32)
            fn = (~pred_pos & tgt_pos).sum().to(dtype=torch.float32)
            if tgt_pos.sum() == 0:
                continue
            precision = tp / (tp + fp + self.eps)
            recall = tp / (tp + fn + self.eps)
            f1 = (2 * precision * recall) / (precision + recall + self.eps)
            f1_scores.append(f1)
        if not f1_scores:
            return preds.new_tensor(0.0)
        return torch.stack(f1_scores).mean()

    def _vector_f1(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        preds = logits.round().long()
        tgt = targets.long()
        if self.ignore_index is not None:
            mask = tgt != self.ignore_index
            preds = preds[mask]
            tgt = tgt[mask]
        if tgt.numel() == 0:
            return logits.new_tensor(0.0)
        num_classes = self.num_classes
        if num_classes is None:
            num_classes = int(max(preds.max(), tgt.max()).item()) + 1
        return self._macro_f1(preds.reshape(-1), tgt.reshape(-1), num_classes)

    def __call__(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        """Macro-averaged F1 over classes present in targets.

        Supports:
        - (N, C, …) logits with (N, …) integer labels or matching one-hot targets
        - (N, D) per-field class indices (e.g. behaviour-clone control vectors)
        """
        if logits.dim() < 2:
            raise ValueError("multiclass_f1 expects logits shaped (N, C, …) or (N, D).")

        if logits.dim() == 2 and targets.dim() == 2 and logits.shape == targets.shape:
            return self._vector_f1(logits, targets)

        if logits.dim() == 2 and targets.dim() == 1:
            preds = logits.argmax(dim=1)
            tgt = targets.long()
        elif logits.dim() >= 3:
            preds = logits.argmax(dim=1)
            if targets.shape == logits.shape:
                tgt = targets.argmax(dim=1)
            elif targets.dim() == logits.dim() - 1:
                tgt = targets.long()
            else:
                raise ValueError(
                    "targets must be (N, …) integer labels or (N, C, …) matching logits; "
                    f"got shape {tuple(targets.shape)}."
                )
            preds = preds.reshape(-1)
            tgt = tgt.reshape(-1)
        else:
            raise ValueError(
                f"unsupported logits/targets shapes: {tuple(logits.shape)} / {tuple(targets.shape)}"
            )

        if self.ignore_index is not None:
            mask = tgt != self.ignore_index
            preds = preds[mask]
            tgt = tgt[mask]
        if tgt.numel() == 0:
            return logits.new_tensor(0.0)

        num_classes = self.num_classes
        if num_classes is None:
            if logits.dim() >= 3:
                num_classes = logits.shape[1]
            else:
                num_classes = int(max(preds.max(), tgt.max()).item()) + 1
        return self._macro_f1(preds, tgt, num_classes)