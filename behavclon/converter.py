from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image
from tqdm import tqdm

from behavclon.control_codec import parse_control_code
from behavclon.common import ControlCommandMarkup
from mlengine.dataset.serializers import ImageFileSerializer
from mlengine.common.type import Frame, Markup, Scene

# Labelled frame: {stamp_ns}_{control_code}.jpg
_LABEL_RE = re.compile(r"^(\d+)_(\w+)\.jpg$")


@dataclass
class BehaviourRecorderConverterCfg:
    source_dir: str
    output_dir: str
    markup_filename: str = "markup.json"
    image_serializer: ImageFileSerializer | None = None
    target_label: str = "control"


class BehaviourRecorderConverter:
    """Convert raw behaviour_recorder tree into images/ + markup.json (current frame only)."""

    def __init__(self, cfg: BehaviourRecorderConverterCfg):
        self.cfg = cfg
        self.source_dir = Path(cfg.source_dir)
        self.output_dir = Path(cfg.output_dir)
        self.markup_path = self.output_dir / cfg.markup_filename
        if cfg.image_serializer is None:
            raise ValueError("image_serializer is required")
        self.image_serializer = cfg.image_serializer
        self._used_image_names: set[str] = set()

    def prepare(self) -> Markup:
        if not self.source_dir.is_dir():
            raise FileNotFoundError(f"Source directory not found: {self.source_dir}")

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.image_serializer.init_source()
        scenes: dict[str, list[Frame]] = {}

        frame_paths = sorted(self.source_dir.rglob("*.jpg"))
        for path in tqdm(frame_paths, desc="Converting behaviour records"):
            parsed = self._parse_labelled_frame(path)
            if parsed is None:
                continue
            stamp_ns, propagation, turn = parsed

            scene_id = self._scene_id(path)
            image_filename = self._save_image(path, stamp_ns, scene_id)
            frame = Frame(
                image_filename=image_filename,
                objects=[
                    ControlCommandMarkup(
                        propagation=propagation,
                        turn=turn,
                    )
                ],
            )
            scenes.setdefault(scene_id, []).append(frame)

        if not scenes:
            raise ValueError(f"No labelled *.jpg frames found under {self.source_dir}")

        markup = Markup(
            scenes=[Scene(scene_id=scene_id, frames=frames) for scene_id, frames in sorted(scenes.items())]
        )
        markup.save(self.markup_path)
        return markup

    def _parse_labelled_frame(self, path: Path) -> tuple[str, str, str] | None:
        match = _LABEL_RE.match(path.name)
        if not match:
            return None
        stamp_ns, control_code = match.group(1), match.group(2)
        try:
            propagation, turn = parse_control_code(control_code)
        except ValueError:
            return None
        return stamp_ns, propagation, turn

    def _scene_id(self, path: Path) -> str:
        rel_parent = path.parent.relative_to(self.source_dir)
        if str(rel_parent) == ".":
            return "default"
        return str(rel_parent).replace("\\", "/")

    def _save_image(self, source_path: Path, stamp_ns: str, scene_id: str) -> str:
        image_filename = f"{stamp_ns}.jpg"
        if image_filename in self._used_image_names:
            safe_scene = scene_id.replace("/", "_")
            image_filename = f"{safe_scene}_{stamp_ns}.jpg"
        if image_filename in self._used_image_names:
            raise ValueError(f"Duplicate image name for stamp {stamp_ns} in scene {scene_id}")

        image = np.array(Image.open(source_path).convert(self.image_serializer.cfg.mode))
        saved_name = self.image_serializer(image_filename).save(image)
        self._used_image_names.add(saved_name)
        return saved_name
