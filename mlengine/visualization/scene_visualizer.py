from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Protocol

import cv2
import numpy as np
from PIL import Image
from tqdm import tqdm

from mlengine.common.logging import setup_logger
from mlengine.common.type import Frame, Markup, Scene

logger = setup_logger("visualization: SceneVisualizer")


@dataclass
class SceneVisualizerCfg:
    output_dir: str | None = None
    frame_visualizer: FrameVisualizer | None = None
    fps: int = 10
    max_scenes: int | None = None
    concat_scenes: bool = True
    output_format: str = "mp4"


class FrameVisualizer(Protocol):
    def visualize_frame(self, frame: Frame) -> Image.Image: ...


@dataclass
class SceneFrame:
    timestamp: int
    image: Image.Image


@dataclass
class FrameSequence:
    frames: list[SceneFrame]

    def sorted(self) -> "FrameSequence":
        self.frames = sorted(self.frames, key=lambda x: x.timestamp)
        return FrameSequence(frames=self.frames)

    def append(self, frame: SceneFrame) -> None:
        self.frames.append(frame)

    def __iter__(self) -> Iterator[SceneFrame]:
        return iter(self.frames)

    def __len__(self) -> int:
        return len(self.frames)

    def get_frame_video_shapes(self) -> tuple[tuple[int, int], tuple[int, int]]:
        if not self.frames:
            raise ValueError("FrameSequence.get_frame_video_shapes requires at least one frame")
        first_image = np.asarray(self.frames[0].image, dtype=np.uint8)
        frame_h, frame_w = first_image.shape[:2]
        video_h = frame_h + 48
        video_w = frame_w
        return (frame_h, frame_w), (video_h, video_w)

class SceneVisualizer:
    def __init__(self, cfg: SceneVisualizerCfg):
        self.cfg = cfg
        self.output_dir = Path(cfg.output_dir)
        self.output_format = cfg.output_format.lower()
        self.frame_visualizer = cfg.frame_visualizer
        self.concat_scenes = cfg.concat_scenes
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.max_scenes = cfg.max_scenes

    def convert_scene_to_frame_sequence(self, scene: Scene) -> FrameSequence:
        frame_sequence = FrameSequence(frames=[])
        for frame in tqdm(scene.frames,
                                   total=len(scene.frames), 
                                   desc=f"Converting scene {scene.scene_id} to frame sequence"):
            frame_sequence.append(SceneFrame(timestamp=int(frame.timestamp), 
                                             image=self.frame_visualizer.visualize_frame(frame)))
        return frame_sequence

    def visualize_markup(self, markup: Markup, output_stem: str | Path) -> list[Path]:
        if self.max_scenes is None:
            self.max_scenes = len(markup)
        output_stem = Path(output_stem)
        if self.concat_scenes:
            combined_frames = [
                sf
                for scene in list(markup)[:self.max_scenes]
                for sf in self.convert_scene_to_frame_sequence(scene)
            ]
            frame_sequence = FrameSequence(frames=combined_frames).sorted()
            output_path = self.output_dir / output_stem.with_suffix(f".{self.output_format}")
            if frame_sequence.frames and self._write_bev_video(frame_sequence, output_path):
                logger.info(f"Saved concatenated visualization to {output_path}")
            return [output_path]

        output_paths = []
        for scene in list(markup)[:self.max_scenes]:
            frame_sequence = self.convert_scene_to_frame_sequence(scene).sorted()
            output_path = (
                self.output_dir
                / output_stem.with_stem(f"scene_{scene.scene_id}").with_suffix(f".{self.output_format}")
            )
            if self._write_bev_video(frame_sequence, output_path):
                logger.info(f"Saved scene visualization to {output_path}")
            output_paths.append(output_path)
        return output_paths

    def _render_canvas(self,
                       scene_frame: SceneFrame,
                       frame_shape: tuple[int, int],
                       video_shape: tuple[int, int],
                       output_name: str) -> Image.Image:
        frame_h, frame_w = frame_shape
        video_h, video_w = video_shape

        image = np.asarray(scene_frame.image, dtype=np.uint8)
        if image.shape[:2] != (frame_h, frame_w):
            raise ValueError(
                f"Inconsistent image shape for video {output_name!r}: "
                f"expected {(frame_h, frame_w)}, got {image.shape[:2]}"
            )

        canvas = Image.new("RGB", (video_w, video_h), color=(0, 0, 0))
        canvas.paste(Image.fromarray(image, mode="RGB"), (0, 0))

        # draw = ImageDraw.Draw(canvas)
        # timestamp = int(sample["timestamp"])
        # frame_index = int(sample.get("frame_index", -1))
        # current_scene_id = str(sample.get("scene_id", ""))
        # overlay = f"scene={current_scene_id} frame={frame_index} ts={timestamp}"
        # draw.text((10, video_h - 28), overlay, fill=(0, 255, 0))
        return canvas

    def _write_bev_video(self, frame_sequence: FrameSequence, output_path: Path) -> bool:
        logger.info(f"Writing {output_path.name!r} in {self.output_format} format")
        if self.output_format == "gif":
            return self._write_bev_gif(frame_sequence, output_path)
        return self._write_bev_mp4(frame_sequence, output_path)

    def _write_bev_mp4(self, frame_sequence: FrameSequence, output_path: Path) -> bool:
        output_path = Path(output_path)
        if not frame_sequence.frames:
            logger.warning("Empty frame sequence; skipping video write")
            return False
        sorted_frames = frame_sequence.sorted()
        frame_shape, video_shape = frame_sequence.get_frame_video_shapes()
        video_h, video_w = video_shape

        output_path.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(
            str(output_path), fourcc, float(self.cfg.fps), (video_w, video_h)
        )
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open video writer for {output_path}")

        try:
            for scene_frame in tqdm(sorted_frames, desc=f"Writing {output_path.name!r}"):
                canvas = self._render_canvas(
                    scene_frame, frame_shape, video_shape, output_path.name
                )
                writer.write(np.asarray(canvas, dtype=np.uint8)[:, :, ::-1])
        finally:
            writer.release()

        return True

    def _write_bev_gif(self, frame_sequence: FrameSequence, output_path: Path) -> bool:
        output_path = Path(output_path)
        if not frame_sequence.frames:
            logger.warning("Empty frame sequence; skipping video write")
            return False
        sorted_frames = frame_sequence.sorted()
        frame_shape, video_shape = frame_sequence.get_frame_video_shapes()

        frames: list[Image.Image] = []
        for scene_frame in tqdm(sorted_frames, desc=f"Writing {output_path.name!r}"):
            canvas = self._render_canvas(scene_frame, frame_shape, video_shape, output_path.name)
            frames.append(
                canvas.quantize(
                    colors=256,
                    method=Image.Quantize.MEDIANCUT,
                    dither=Image.Dither.NONE,
                )
            )

        output_path.parent.mkdir(parents=True, exist_ok=True)
        frames[0].save(
            output_path,
            save_all=True,
            append_images=frames[1:],
            duration=max(1, int(round(1000 / self.cfg.fps))),
            loop=0,
            optimize=False,
            disposal=2,
        )
        return True