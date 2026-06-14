from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from time import time
from typing import Any

import torch
from torch import nn, Tensor
from torch.optim import Optimizer
from torch.nn.parallel import DistributedDataParallel as DDP
from torch.utils.data import DataLoader
from torch.utils.data.distributed import DistributedSampler
from mlengine.dataset.loaders import DatasetLoader

from mlengine.training.debug_sampling import DebugSampleManager
from mlengine.training.losses import Loss, WeightsRegularization
from mlengine.training.metrics import Metric
from mlengine.training.monitoring import EpochStats, Monitor
from mlengine.training.distributed_utils import (
    get_local_rank,
    get_world_size,
    is_distributed,
    is_main_process,
    merge_gathered_sample_dicts,
)
from mlengine.common.logging import setup_logger

logger = setup_logger("train_val_pipeline")

@dataclass
class TrainValPipelineCfg:
    train_dataset: DatasetLoader
    val_dataset: DatasetLoader
    model: nn.Module
    optimizer: Optimizer | Any
    loss: Loss
    weights_regularization: WeightsRegularization
    monitor: Monitor
    output_dir: str
    metrics: dict[str, Metric]
    device: str = "auto"
    epochs: int = 1
    val_interval: int = 1
    batch_size: int = 1
    num_workers: int = 0
    pin_memory: bool | None = None
    shuffle: bool = True
    amp: bool = False
    cuda_memory_cleanup_between_epochs: bool = True
    debug_sample_manager: DebugSampleManager | None = None
    ddp_find_unused_parameters: bool = False
    ddp_linear_lr_scaling: bool = True


def _release_cupy_pool_memory() -> None:
    try:
        import cupy as cp  # type: ignore[import-not-found]
    except ImportError:
        return
    try:
        cp.get_default_memory_pool().free_all_blocks()
    except Exception:
        return
    try:
        cp.get_default_pinned_memory_pool().free_all_blocks()
    except Exception:
        pass


def resolve_device(device: str) -> torch.device:
    if is_distributed() and torch.cuda.is_available():
        return torch.device("cuda", get_local_rank())
    if device == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(device)


class TrainValPipeline:
    def __init__(self, cfg: TrainValPipelineCfg):
        self.cfg = cfg
        self._distributed = is_distributed()
        self._local_rank = get_local_rank()
        self.device = resolve_device(cfg.device)
        self.model = cfg.model.to(self.device)
        if self._distributed:
            if self.device.type != "cuda":
                raise RuntimeError("DDP training expects CUDA devices.")
            self.model = DDP(
                self.model,
                device_ids=[self._local_rank],
                output_device=self._local_rank,
                find_unused_parameters=cfg.ddp_find_unused_parameters,
            )
        self.criterion = cfg.loss.to(self.device)
        self.weights_reg = None
        if cfg.weights_regularization is not None:
            self.weights_reg = cfg.weights_regularization
            self.weights_reg.set_params(self._unwrap_model().parameters())
            self.weights_reg = self.weights_reg.to(self.device)
        self.optimizer = self._build_optimizer(cfg.optimizer)
        self.metrics = self.cfg.metrics
        self.output_dir = Path(cfg.output_dir)
        self.best_metric = -1.0
        self._use_amp = bool(cfg.amp) and self.device.type == "cuda"
        self._scaler = torch.amp.GradScaler("cuda", enabled=self._use_amp)
        self.monitor = cfg.monitor
        self.debug_sample_manager = cfg.debug_sample_manager
        self._train_sampler: DistributedSampler | None = None

    def _unwrap_model(self) -> nn.Module:
        return self.model.module if isinstance(self.model, DDP) else self.model

    def is_validation_epoch(self, epoch: int) -> bool:
        return epoch % self.cfg.val_interval == 0

    def _to_device(self, tensor: Tensor) -> Tensor:
        return tensor.to(self.device, dtype=torch.float32, non_blocking=True)

    @staticmethod
    def _collate_batch(batch):
        sample_ids, inputs, targets = zip(*batch)
        return list(sample_ids), list(inputs), list(targets)

    def train(self) -> None:
        train_dataset, self._train_sampler = self._build_dataloader(
            self.cfg.train_dataset, shuffle=self.cfg.shuffle, train=True
        )
        val_dataset, _ = self._build_dataloader(self.cfg.val_dataset, shuffle=False, train=False)

        for epoch in range(1, self.cfg.epochs + 1):
            is_val_epoch = self.is_validation_epoch(epoch)
            if self._train_sampler is not None:
                self._train_sampler.set_epoch(epoch)
            if self.debug_sample_manager is not None:
                self.debug_sample_manager.clear()

            epoch_stats = EpochStats(epoch, 
                                     learning_rate=self.optimizer.param_groups[0]["lr"], 
                                     time_start=time())
            epoch_stats.set_train(*self._run_epoch(train_dataset, train=True))
            if is_val_epoch:
                epoch_stats.set_val(*self._run_epoch(val_dataset, train=False))
                if epoch_stats.mean_val_metric > self.best_metric:
                    self.best_metric = epoch_stats.mean_val_metric
                    self._save_checkpoint("best.pth", epoch, epoch_stats)
                self._save_checkpoint("last.pth", epoch, epoch_stats)

                if self.debug_sample_manager is not None:
                    train_samples, val_samples = self.debug_sample_manager.debug_samples()
                    epoch_stats.train_debug_samples.update(merge_gathered_sample_dicts(train_samples))
                    epoch_stats.val_debug_samples.update(merge_gathered_sample_dicts(val_samples))

            if self.device.type == "cuda" and self.cfg.cuda_memory_cleanup_between_epochs:
                torch.cuda.synchronize()
                _release_cupy_pool_memory()
                torch.cuda.empty_cache()

            epoch_stats.set_duration(time(), is_val_epoch)
            
            if is_main_process():
                self.monitor.log_epoch_stats(epoch_stats)
                epoch_stats.print()



    def _build_dataloader(self, 
                          dataset: DatasetLoader, 
                          shuffle: bool, 
                          train: bool) -> tuple[DataLoader, DistributedSampler | None]:
        """Build the dataloader for the given dataset."""
        sampler: DistributedSampler | None = None
        if self._distributed:
            sampler = DistributedSampler(
                dataset,
                shuffle=shuffle,
                drop_last=train,
            )
            shuffle = False
        loader = DataLoader(
            dataset,
            batch_size=self.cfg.batch_size,
            shuffle=shuffle,
            sampler=sampler,
            num_workers=self.cfg.num_workers,
            pin_memory=self.cfg.pin_memory if self.cfg.pin_memory is not None else self.device.type == "cuda",
            collate_fn=self._collate_batch,
        )
        return loader, sampler

    def _build_optimizer(self, optimizer: Optimizer | Any) -> Optimizer:
        opt: Optimizer = optimizer.build(self.model.parameters())
        if self._distributed and self.cfg.ddp_linear_lr_scaling:
            ws = get_world_size() 
            for group in opt.param_groups:
                group["lr"] = float(group["lr"]) * float(ws)
            if is_main_process():
                logger.info("DDP linear LR scaling applied: lr *= world_size (%s).", ws)

        return opt

    def _reduce_epoch_totals(self, 
                             loss_sum: float, 
                             sample_count: int, 
                             metric_sums: dict[str, float]) -> tuple[float, dict[str, float]]:
        """Reduce the epoch totals across all ranks."""
        if not self._distributed:
            denom = max(1, sample_count)
            return loss_sum / denom, {k: v / denom for k, v in metric_sums.items()}

        # Construct helper array of ddp loss/metrics parts
        keys = list(metric_sums.keys())
        t = torch.zeros(2 + len(keys), dtype=torch.float64, device=self.device)
        t[0] = loss_sum
        t[1] = float(sample_count)
        for i, k in enumerate(keys):
            t[2 + i] = metric_sums[k]
        # Every rank replaces its t with the element-wise sum across all ranks. 
        # After this, all ranks agree on the global totals.
        torch.distributed.all_reduce(t, op=torch.distributed.ReduceOp.SUM)
        # Calculate the average values for each metric and the loss
        denom = max(1.0, t[1].item())
        out_loss = t[0].item() / denom
        out_metrics = {keys[i]: t[2 + i].item() / denom for i in range(len(keys))}
        return out_loss, out_metrics

    def _update_epoch_metrics(self, 
                              metrics: dict[str, float], 
                              logits: torch.Tensor, 
                              targets: torch.Tensor, 
                              batch_size: int) -> None:
        """Update the epoch metrics with the given logits and targets."""
        for name, metric in self.metrics.items():
            metrics[name] += metric(logits, targets) * batch_size

    def _run_epoch(self, dataloader: DataLoader, train: bool) -> tuple[float, dict[str, float]]:
        self.model.train(train)

        total_samples = 0
        total_loss = 0.0
        total_metrics = {name: 0.0 for name in self.metrics.keys()}

        context = torch.enable_grad() if train else torch.no_grad()
        with context:
            for ids, inputs, targets in dataloader:
                inputs = [self._to_device(input) for input in inputs]
                targets_tensor = torch.stack(
                    [self._to_device(torch.from_numpy(targ_in_frame.to_numpy())) for 
                    targ_in_batch in targets for 
                    targ_in_frame in targ_in_batch])
           
                amp_ctx = torch.amp.autocast("cuda") if self._use_amp else nullcontext()
                with amp_ctx:
                    logits = self.model(inputs)
                    loss = self.criterion(logits, targets_tensor)
                    if train and self.weights_reg is not None:
                        loss = loss + self.weights_reg()

                if train:
                    self.optimizer.zero_grad(set_to_none=True)
                    if self._use_amp:
                        self._scaler.scale(loss).backward()
                        self._scaler.step(self.optimizer)
                        self._scaler.update()
                    else:
                        loss.backward()
                        self.optimizer.step()

                batch_size = len(inputs)
                total_samples += batch_size
                total_loss += loss.item() * batch_size
                self._update_epoch_metrics(total_metrics, logits, targets_tensor, batch_size)

                if self.debug_sample_manager is None:
                    continue
                for batch_items in zip(ids, inputs, logits, targets):
                    sample_id, sample_inputs, sample_logits, sample_targets = batch_items
                    if self.debug_sample_manager.should_capture(sample_id, train):
                        model = self._unwrap_model()
                        self.debug_sample_manager.vis_and_save_predicts(
                            sample_id=str(sample_id),
                            data=sample_inputs.cpu().numpy(),
                            predictions=model.logits_to_predicted_objects(sample_logits),
                            markups=sample_targets,
                            train=train,
                        )

        epoch_loss, epoch_metrics = self._reduce_epoch_totals(total_loss, total_samples, total_metrics)
        return epoch_loss, epoch_metrics

    def _save_checkpoint(self, filename: str, epoch: int, stats: EpochStats) -> None:
        if is_main_process():
            self.output_dir.mkdir(parents=True, exist_ok=True)
            ckpt_path = self.output_dir / filename
            torch.save(
                {
                    "epoch": epoch,
                    "model": self._unwrap_model().state_dict(),
                    "optimizer": self.optimizer.state_dict(),
                    "best_metric": self.best_metric,
                    "train_metrics": stats.train_metrics,
                    "val_metrics": stats.val_metrics,
                },
                ckpt_path,
            )
            self.monitor.upload_checkpoint(ckpt_path, epoch=epoch, kind=Path(filename).stem)
