from dataclasses import dataclass
from random import Random

from typing import Any
from PIL.Image import Image

from mlengine.common.logging import setup_logger
from mlengine.common.type import Markup
from mlengine.common.objects import FrameMarkupObject, FramePredictionObject
from mlengine.visualization import FrameVisualizer
from mlengine.training.distributed_utils import broadcast_object_from_rank0, get_local_rank, is_main_process

logger = setup_logger("training: DebugSampleManager")


@dataclass
class DebugSampleManagerCfg:
    """Allowlists come from ``train_markup_path`` / ``val_markup_path`` (PCD stems).

    Candidate IDs are **all** stems in each markup (same files as loaders for best overlap).

    ``max_*_samples``: how many IDs rank 0 samples at random from candidates, broadcasts to all
    ranks, and caps distinct debug samples per epoch; ``None`` = use all candidates.
    """

    frame_visualizer: FrameVisualizer
    train_markup_path: str
    val_markup_path: str    
    max_train_samples: int | None = None
    max_val_samples: int | None = None
    auto_random_seed: int = 0


class DebugSampleManager:
    def __init__(self, cfg: DebugSampleManagerCfg):
        self.cfg = cfg
        self.random_seed = cfg.auto_random_seed
        self.frame_visualizer = cfg.frame_visualizer
        self._train_debug_samples: dict[str, Image] = {}
        self._val_debug_samples: dict[str, Image] = {}
        self._random_allowlists_done = False
        self._train_candidates: list[str] = []
        self._val_candidates: list[str] = []
        train_m = Markup.from_file(str(cfg.train_markup_path))
        val_m = Markup.from_file(str(cfg.val_markup_path))
        self._train_candidates = train_m.sample_stems()
        self._val_candidates = val_m.sample_stems()
        self._train_id_allow: frozenset[str] = frozenset()
        self._val_id_allow: frozenset[str] = frozenset()

    def _random_k_from_candidates(self, candidates: list[str], k: int | None) -> list[str]:
        if not candidates:
            return []
        take = k if k is not None else len(candidates)
        return Random(self.random_seed).sample(candidates, take)

    def _ensure_allowlists(self) -> None:
        if self._random_allowlists_done:
            return
        if is_main_process():
            train_ids = self._random_k_from_candidates(
                self._train_candidates,
                self.cfg.max_train_samples,
            )
            val_ids = self._random_k_from_candidates(
                self._val_candidates,
                self.cfg.max_val_samples,
            )
            train_val_ids: tuple[list[str], list[str]] | None = (train_ids, val_ids)
        else:
            train_val_ids = None
        train_ids, val_ids = broadcast_object_from_rank0(train_val_ids)
        self._train_id_allow = frozenset(map(str, train_ids))
        self._val_id_allow = frozenset(map(str, val_ids))
        self._random_allowlists_done = True
        logger.debug(f"Rank {get_local_rank()}: Synced Train IDs: {train_ids}, Val IDs: {val_ids}")

    def clear(self) -> None:
        self._ensure_allowlists()
        self._train_debug_samples.clear()
        self._val_debug_samples.clear()

    def should_capture(self, sample_id: str | int, train: bool) -> bool:
        self._ensure_allowlists()
        sid = str(sample_id)
        return sid in (self._train_id_allow if train else self._val_id_allow)

    def vis_and_save_predicts(self,
                              sample_id: str | int,
                              data: Any,
                              predictions: list[FramePredictionObject],
                              markups: list[FrameMarkupObject],
                              train: bool) -> None:
        sid = str(sample_id)
        if not self.should_capture(sid, train):
            return

        rgb_image = self.frame_visualizer.visualize_objects(data, predictions, markups)
        if rgb_image is None:
            return
        if train:
            self._train_debug_samples[sid] = rgb_image
        else:
            self._val_debug_samples[sid] = rgb_image

    def debug_samples(self) -> tuple[dict[str, Image], dict[str, Image]]:
        """Per-rank ``{sample_id: RGB PIL image}`` for this epoch; merge under DDP before ClearML."""
        return dict(self._train_debug_samples), dict(self._val_debug_samples)
