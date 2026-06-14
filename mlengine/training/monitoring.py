from collections.abc import Mapping
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

from matplotlib import pyplot as plt
import pandas as pd
from PIL.Image import Image
from typing import Protocol

from mlengine.training.distributed_utils import is_main_process

# Above this many seconds, log epoch duration in minutes (separate ClearML title).
_EPOCH_DURATION_LOG_MINUTES_THRESHOLD_S = 300.0


class Monitor(Protocol):
    def log_scalars(self, values: dict[str, float], title: str, iteration: int) -> None:
        ...
    def report_plot(self, plt: plt.Figure, title: str, series: str, iteration: int) -> None:
        ...
    def report_image(self, image: Image, title: str, series: str, iteration: int) -> None:
        ...
    def report_text(self, text: str) -> None:
        ...
    def report_table(self, df: pd.DataFrame, name: str, iteration: int) -> None:
        ...
    def log_metrics(self, metrics: dict[str, float], epoch: int, stage: str) -> None:
        ...
    def log_average(self, metrics: dict[str, float]) -> None:
        ...
    def print_metrics(self, metrics: dict[str, float]) -> None:
        ...


@dataclass
class ClearMLMonitorCfg:
    project_type: str
    task_type: str
    subtask_type: str
    username: str
    jira_task_id: str
    task_labels: dict = field(default_factory=dict)
    experiment_description: str = ""
    clearml_log: bool = False
    n_debug_samples: int = 10


@dataclass
class EpochStats:
    epoch: int
    learning_rate: float | None = None
    time_start: float | None = None
    train_debug_samples: dict[str, Image] = field(default_factory=dict)
    val_debug_samples: dict[str, Image] = field(default_factory=dict)
    train_loss: float | None = None
    val_loss: float | None = None
    train_metrics: dict[str, float] | None = None
    val_metrics: dict[str, float] | None = None
    train_duration: float | None = None
    train_with_val_duration: float | None = None

    def set_duration(self, time_end: float, is_val_epoch: bool) -> float:
        if self.time_start is None:
            return None
        duration = time_end - self.time_start
        if is_val_epoch:
            self.train_with_val_duration = duration
        else:
            self.train_duration = duration
        return duration

    def set_train(self, loss: float, metrics: dict[str, float]) -> None:
        self.train_loss = loss
        self.train_metrics = metrics

    def set_val(self, loss: float, metrics: dict[str, float]) -> None:
        self.val_loss = loss
        self.val_metrics = metrics

    def mean_loss(self) -> float:
        return (self.train_loss + self.val_loss) / 2

    @property
    def mean_val_metric(self) -> float:
        return sum(self.val_metrics.values()) / len(self.val_metrics)

    def print(self) -> None:
        print_line = (f"epoch={self.epoch:03d} "
                      f"train_loss={self.train_loss:.4f} "
                      f"{' '.join(f'{name}={value:.4f}' for name, value in self.train_metrics.items())} ")
        if self.val_metrics is not None:
            print_line += (f"val_loss={self.val_loss:.4f} "
                           f"{' '.join(f'{name}={value:.4f}' for name, value in self.val_metrics.items())}")
        print(print_line)


@dataclass
class TensorBoardMonitorCfg:
    logdir: str
    output_dir: str = ""


class TensorBoardMonitor:
    def __init__(self, cfg: TensorBoardMonitorCfg):
        from torch.utils.tensorboard import SummaryWriter

        self.cfg = cfg
        self.writer = SummaryWriter(log_dir=cfg.logdir)

    def connect_experiment_config(self, configuration: Mapping[str, Any]) -> None:
        pass

    def log_epoch_stats(self, epoch_stats: EpochStats) -> None:
        epoch = epoch_stats.epoch
        if epoch_stats.learning_rate is not None:
            self.writer.add_scalar("learning_rate", epoch_stats.learning_rate, epoch)
        if epoch_stats.train_loss is not None:
            self.writer.add_scalar("loss/train", epoch_stats.train_loss, epoch)
        if epoch_stats.val_loss is not None:
            self.writer.add_scalar("loss/val", epoch_stats.val_loss, epoch)
        if epoch_stats.train_metrics:
            for name, value in epoch_stats.train_metrics.items():
                self.writer.add_scalar(f"metrics/train/{name}", value, epoch)
        if epoch_stats.val_metrics:
            for name, value in epoch_stats.val_metrics.items():
                self.writer.add_scalar(f"metrics/val/{name}", value, epoch)

    def upload_checkpoint(self, checkpoint_path: str | Path, *, epoch: int, kind: str) -> None:
        pass

    def close(self) -> None:
        self.writer.close()


class ClearMLMonitor:
    def __init__(self, cfg: ClearMLMonitorCfg):
        self.task_labels = cfg.task_labels
        task_name = (
            f"{datetime.now().strftime('%y%m%d')}/{cfg.jira_task_id}/{cfg.experiment_description}"
        )
        from clearml import Task
        self.task = None
        self.clearml_logger = None
        if cfg.clearml_log and is_main_process():

            self.task = Task.init(
                project_name=self.compose_project_name(
                    project_type=cfg.project_type,
                    task_type=cfg.task_type,
                    subtask_type=cfg.subtask_type,
                ),
                task_name=task_name,
                tags=[cfg.task_type, cfg.username],
            )
            self.clearml_logger = self.task.get_logger().set_default_debug_sample_history(
                cfg.n_debug_samples
            )
        # First non-null epoch duration picks ClearML unit (s vs min) for the whole run.
        self._epoch_duration_unit: str | None = None

    @staticmethod
    def compose_project_name(project_type: str,
                             task_type: str,
                             subtask_type: str) -> str:
        return f"{project_type.capitalize()}: {task_type}: {subtask_type.capitalize()}"

    def connect_experiment_config(self, configuration: Mapping[str, Any]) -> None:
        """Store resolved experiment YAML (as a dict) in the task Configuration section."""
        if not self.task:
            return
        self.task.connect_configuration(dict(configuration), name="resolved_experiment")

    def log_scalars(self, values: dict[str, Any], title: str, iteration: int) -> None:
        if self.task:
            for series_name, value in values.items():
                self.task.get_logger().report_scalar(title=title,
                                                     series=series_name,
                                                     value=float(value),
                                                     iteration=iteration)

    def report_plot(self, plt: plt.Figure, title: str, series: str, iteration: int) -> None:
        if self.task:
            self.task.get_logger().report_matplotlib_figure(
                title=title, series=series, iteration=iteration, figure=plt, report_image=True)

    def upload_artifact(self, name: str, path2artifact: str) -> None:
        if self.task:
            self.task.upload_artifact(name=name, artifact_object=path2artifact)

    def upload_checkpoint(self, checkpoint_path: str | Path, *, epoch: int, kind: str) -> None:
        """Upload a saved .pth file as a ClearML artifact (same name overwrites previous upload)."""
        if not self.task:
            return
        path = Path(checkpoint_path)
        self.task.upload_artifact(
            name=f"checkpoint.{kind}",
            artifact_object=path.as_posix(),
            metadata={"epoch": epoch, "filename": path.name},
        )

    def report_plotly(self, plt: plt.Figure, title: str, series: str, iteration: int) -> None:
        if self.task:
            self.task.get_logger().report_plotly(
                title=title, series=series, iteration=iteration, figure=plt
            )

    def report_image(self, image: Image, title: str, series: str, iteration: int) -> None:
        if self.task:
            self.task.get_logger().report_image(title=title,
                                                series=series,
                                                iteration=iteration,
                                                image=image)

    def report_media(self, media: str, title: str, series: str, iteration: int) -> None:
        if self.task:
            self.task.get_logger().report_media(title=title,
                                                series=series,
                                                iteration=iteration,
                                                local_path=media)

    def report_text(self, text):
        if self.task:
            self.task.get_logger().report_text(text)

    def report_table(self, df, name, iteration):
        if self.task:
            self.task.get_logger().report_table(
                "Final metrics",
                name,
                iteration=iteration,
                table_plot=df
            )

    def duration_sec_to_min(self, duration: float) -> float:
        if self._epoch_duration_unit is None:
            if duration >= _EPOCH_DURATION_LOG_MINUTES_THRESHOLD_S:
                self._epoch_duration_unit = "min"
            else:
                self._epoch_duration_unit = "s"
        return duration / (60.0 if self._epoch_duration_unit == "min" else 1.0)

    def log_epoch_stats(self, epoch_stats: EpochStats):
        if not self.task:
            return

        # Log learning rate
        lr_values: dict[str, float] = {}
        if epoch_stats.learning_rate is not None:
            lr_values["learning_rate"] = epoch_stats.learning_rate
        self.log_scalars(lr_values, title="Learning Rate", iteration=epoch_stats.epoch)

        # Log epoch duration (unit fixed after first logged duration — no s/min switching per epoch)
        duration_values: dict[str, float] = {}
        if epoch_stats.train_duration is not None:
            duration_values["train_duration"] = self.duration_sec_to_min(epoch_stats.train_duration)
        if epoch_stats.train_with_val_duration is not None:
            duration_values["train_with_val_duration"] = self.duration_sec_to_min(epoch_stats.train_with_val_duration)
        self.log_scalars(values=duration_values, 
                         title=f"Epoch Duration ({self._epoch_duration_unit})", 
                         iteration=epoch_stats.epoch)

        # Log loss
        loss_values: dict[str, float] = {}
        if epoch_stats.train_loss is not None:
            loss_values["train_loss"] = epoch_stats.train_loss
        if epoch_stats.val_loss is not None:
            loss_values["val_loss"] = epoch_stats.val_loss
        self.log_scalars(loss_values, title="Loss", iteration=epoch_stats.epoch)

        # Log metrics
        metric_values: dict[str, float] = {}
        if epoch_stats.train_metrics:
            metric_values.update({f"train/{name}": float(v) for name, v in epoch_stats.train_metrics.items()})
        if epoch_stats.val_metrics:
            metric_values.update({f"val/{name}": float(v) for name, v in epoch_stats.val_metrics.items()})
        self.log_scalars(metric_values, title="Metrics", iteration=epoch_stats.epoch)

        # Log debug images
        if is_main_process():
            for sample_id, image in sorted(epoch_stats.train_debug_samples.items()):
                self.report_image(
                    image,
                    title="Train Debug Samples",
                    series=str(sample_id),
                    iteration=epoch_stats.epoch,
                )
            for sample_id, image in sorted(epoch_stats.val_debug_samples.items()):
                self.report_image(
                    image,
                    title="Val Debug Samples",
                    series=str(sample_id),
                    iteration=epoch_stats.epoch,
                )

