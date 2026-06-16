from __future__ import annotations

import json
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Protocol, runtime_checkable

import numpy as np


class FrameMarkupObject(ABC):
    @abstractmethod
    def to_dict(self) -> Dict[str, Any]: ...

    @abstractmethod
    def to_numpy(self) -> np.ndarray: ...

    @classmethod
    @abstractmethod
    def from_dict(cls, data: Dict[str, Any]) -> FrameMarkupObject: ...


class FramePredictionObject(ABC):
    @abstractmethod
    def to_numpy(self) -> np.ndarray: ...


@dataclass
class Frame:
    image_filename: str | None = None
    pcd_filename: str | None = None
    objects: list[FrameMarkupObject] = field(default_factory=list)

    @classmethod
    def object_from_dict(cls, data: Dict[str, Any]) -> FrameMarkupObject:
        from mlengine.common.objects import OBJECTS_TYPE_MAPPING
        return OBJECTS_TYPE_MAPPING[data.get("type", "cuboid")].from_dict(data)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> Frame:
        data = dict(data)
        data["objects"] = [cls.object_from_dict(obj) for obj in data.get("objects", [])]
        return cls(
            image_filename=data.get("image_filename"),
            pcd_filename=data.get("pcd_filename"),
            objects=data["objects"],
        )

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {"objects": [obj.to_dict() for obj in self.objects]}
        if self.image_filename is not None:
            result["image_filename"] = self.image_filename
        if self.pcd_filename is not None:
            result["pcd_filename"] = self.pcd_filename
        return result


@dataclass(eq=False)
class Scene:
    scene_id: str = ""
    frames: list[Frame] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Dict[str, Any], scene_id: str = "") -> Scene:
        return cls(
            scene_id=scene_id,
            frames=[Frame.from_dict(frame) for frame in data["frames"]],
        )

    def to_dict(self) -> Dict[str, Any]:
        return {"frames": [frame.to_dict() for frame in self.frames]}


class Markup:
    def __init__(self, scenes: list[Scene]) -> None:
        self.scenes = scenes

    def __iter__(self):
        return iter(self.scenes)

    def __len__(self) -> int:
        return len(self.scenes)

    def to_json(self) -> Dict[str, Dict[str, Any]]:
        return {scene.scene_id: scene.to_dict() for scene in self.scenes}

    def save(self, filename: str | Path) -> None:
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(self.to_json(), f, ensure_ascii=False, indent=2)

    @classmethod
    def from_json(cls, json_data: Dict[str, Dict[str, Any]]) -> Markup:
        scenes = [
            Scene.from_dict(scene_data, scene_id=scene_id)
            for scene_id, scene_data in json_data.items()
        ]
        return cls(scenes)

    @classmethod
    def from_file(cls, filename: str | Path) -> Markup:
        with open(filename, encoding="utf-8") as f:
            return cls.from_json(json.load(f))

    def flatten_frames(self) -> list[Frame]:
        return [frame for scene in self for frame in scene.frames]

    def sample_stems(self) -> list[str]:
        stems = []
        for frame in self.flatten_frames():
            if frame.pcd_filename is None:
                continue
            stems.append(str(Path(frame.pcd_filename).stem))
        return stems


@runtime_checkable
class Targets(Protocol):
    def to_numpy(self) -> np.ndarray: ...