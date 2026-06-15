from pathlib import Path
from typing import Protocol


class ModelONNXExporter(Protocol):
    def export(self) -> Path:
        ...