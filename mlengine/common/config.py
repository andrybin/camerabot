import importlib
from pathlib import Path
from typing import Any, Mapping, Sequence

from omegaconf import OmegaConf

class_registry = {}

def parse_config(
    config_path: Path,
    overrides: Sequence[str] | None = None,
) -> dict[str, Any]:
    cfg = OmegaConf.load(config_path)
    if overrides:
        cfg = OmegaConf.merge(cfg, OmegaConf.from_dotlist(list(overrides)))
    cfg = OmegaConf.to_container(cfg, resolve=True)
    return cfg


def init_from_config(cfg: dict[str, Any]) -> dict[str, Any]:
    for key, value in cfg.items():
        if isinstance(value, str):
            pass
        elif isinstance(value, Mapping):
            cfg[key] = init_from_config(value)
    cfg = initialize_class(cfg)
    return cfg


def initialize_class(class_or_cfg: str | Mapping[str, Any]):
    """Initialize a class from an import path or a config mapping.

    Args:
        class_or_cfg: Either a dotted import path like
            ``"dataset.spray_segmentation.SpraySegmentationLoader"``
            or a mapping like
            ``{"type": "dataset.serialization.QDSLidarCodec", "scale": 100.0}``.
        **overrides: Extra keyword arguments passed to the constructor.

    Returns:
        Instantiated class object.
    """
    if isinstance(class_or_cfg, Mapping):
        if "type" not in class_or_cfg:
            return class_or_cfg
        class_path = str(class_or_cfg["type"])
        if "params" in class_or_cfg:
            params = class_or_cfg["params"]
            if not isinstance(params, Mapping):
                raise TypeError(
                    f"Config params for {class_path!r} must be a mapping, "
                    f"got {type(params)}."
                )
            init_kwargs = params
        else:
            init_kwargs = {
                key: value for key, value in class_or_cfg.items() if key != "type"
            }
    else:
        raise TypeError(
            f"initialize_class() expects a mapping, got {type(class_or_cfg)}"
        )

    if class_path in class_registry:
        class_ = class_registry.get(class_path, None)
        class_name = class_path
        module = importlib.import_module(class_.__module__)
    else:
        module_name, class_name = class_path.rsplit(".", 1)
        module = importlib.import_module(module_name)
        class_ = getattr(module, class_name, None)
    
    if class_ is None:
        raise ValueError(f"Class {class_path} of module {module_name} not found. \
        Make sure the class is imported in the __init__.py file.")

    # Try to find class config 
    cfg_class_name = f"{class_name}Cfg"
    cfg_class = getattr(module, cfg_class_name, None)
    if cfg_class is not None:
        cfg = cfg_class(**init_kwargs)
        return class_(cfg)
    return class_(**init_kwargs)