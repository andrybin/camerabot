from random import Random
from dataclasses import dataclass, field
from typing import Dict
from pathlib import Path

from mlengine.common.type import Markup, Scene
from mlengine.common.logging import setup_logger

logger = setup_logger("dataset:TrainValSplitter")
fractions_logger = setup_logger("dataset:TrainFractionsSplitter")

@dataclass
class TrainValSplitterCfg:
    data_root: str = ""
    scene_split: Dict[str, float] = field(
        default_factory=lambda: {"train": 0.8, "val": 0.2}
    )
    random_seed: int = 0
    train_markup_filename: str | None = None
    val_markup_filename: str | None = None

@dataclass
class TrainFractionsSplitterCfg:
    data_root: str = ""
    markup_filename: str = "markup_train.json"
    train_fractions: list[float] = field(default_factory=list)
    sort_by_id: bool = True
    shuffle: bool = False
    random_seed: int = 0


class TrainValSplitter:
    def __init__(self, cfg: TrainValSplitterCfg):
        self.random_seed = cfg.random_seed
        self.scene_split = self.normalize_split(cfg.scene_split)
        self.data_root = Path(cfg.data_root)
        self.train_markup_filename = cfg.train_markup_filename
        self.val_markup_filename = cfg.val_markup_filename

    def split_markup(self, markup: Markup) -> tuple[Markup, Markup]:
        scene_map = {str(scene.scene_id): scene for scene in markup}
        scene_ids = sorted(scene_map)
        Random(self.random_seed).shuffle(scene_ids)

        split_names = list(self.scene_split)
        counts = {name: int(len(scene_ids) * self.scene_split[name]) for name in split_names}
        assigned = sum(counts.values())
        remainders = sorted(
            ((len(scene_ids) * self.scene_split[name]) - counts[name], name)
            for name in split_names
        )
        while assigned < len(scene_ids):
            _, split_name = remainders.pop()
            counts[split_name] += 1
            assigned += 1

        positive_splits = [name for name in split_names if self.scene_split[name] > 0]
        if len(scene_ids) >= len(positive_splits):
            for split_name in positive_splits:
                if counts[split_name] > 0:
                    continue
                donor = max(positive_splits, key=lambda name: counts[name])
                if counts[donor] > 1:
                    counts[donor] -= 1
                    counts[split_name] += 1

        split_scene_ids = {}
        start = 0
        for split_name in split_names:
            end = start + counts[split_name]
            split_scene_ids[split_name] = scene_ids[start:end]
            start = end

        train_markup = Markup(scene_map[scene_id] for scene_id in split_scene_ids["train"])
        val_markup = Markup(scene_map[scene_id] for scene_id in split_scene_ids["val"])
        return train_markup, val_markup

    def split_and_save(self, path_or_markup: str | Markup) -> None:
        if isinstance(path_or_markup, str):
            markup = Markup.from_file(path_or_markup)
        else:
            markup = path_or_markup

        train_markup, val_markup = self.split_markup(markup)
        if self.train_markup_filename is not None:
            train_markup.save(self.data_root / self.train_markup_filename)
            logger.info(f"Saved {len(train_markup)} scenes to {self.train_markup_filename}")
        if self.val_markup_filename is not None:
            val_markup.save(self.data_root / self.val_markup_filename)
            logger.info(f"Saved {len(val_markup)} scenes to {self.val_markup_filename}")


    @staticmethod
    def normalize_split(scene_split: Dict[str, float]) -> Dict[str, float]:
        expected_keys = {"train", "val"}
        actual_keys = set(scene_split)
        if actual_keys != expected_keys:
            raise ValueError(
                f"scene_split must contain exactly {sorted(expected_keys)}, got {sorted(actual_keys)}"
            )
        total = sum(scene_split.values())
        if total <= 0:
            raise ValueError("scene_split values must sum to a positive number")
        return {name: value / total for name, value in scene_split.items()}


class TrainFractionsSplitter:
    def __init__(self, cfg: TrainFractionsSplitterCfg):
        self.data_root = Path(cfg.data_root)
        self.markup_filename = Path(cfg.markup_filename)
        self.train_fractions = cfg.train_fractions
        self.sort_by_id = cfg.sort_by_id
        self.shuffle = cfg.shuffle
        self.random_seed = cfg.random_seed

    def _ordered_scenes(self, markup: Markup) -> list[Scene]:
        scenes = list(markup)
        if self.sort_by_id:
            scenes.sort(key=lambda scene: str(scene.scene_id))
        if self.shuffle:
            Random(self.random_seed).shuffle(scenes)
        return scenes

    def _subset_markup(self, scenes: list[Scene], fraction: float) -> Markup:
        if not 0 < fraction <= 1.0:
            raise ValueError(f"fraction must be in (0, 1], got {fraction}")

        n_total = len(scenes)
        n_keep = max(1, round(n_total * fraction))
        return Markup(scenes[:n_keep])

    def _output_path(self, fraction: float) -> Path:
        frac_label = format(fraction, "g")

        return (self.data_root / self.markup_filename.with_stem(f"{self.markup_filename.stem}__frac_{frac_label}"))

    def split_and_save(self, path_or_markup: str | Markup) -> None:
        if not self.train_fractions:
            raise ValueError("train_fractions must be a non-empty list of fractions in (0, 1]")

        if isinstance(path_or_markup, Markup):
            markup = path_or_markup
        else:
            markup = Markup.from_file(path_or_markup)
            

        scenes = self._ordered_scenes(markup)
        if not scenes:
            raise ValueError("markup has no scenes to split")
        n_total = len(scenes)

        for fraction in self.train_fractions:
            subset = self._subset_markup(scenes, fraction)
            out_path = self._output_path(fraction)
            subset.save(out_path)
            fractions_logger.info(
                f"Saved {len(subset)}/{n_total} scenes ({fraction:.2%}) to {out_path.name}"
            )
