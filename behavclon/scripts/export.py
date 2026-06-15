from __future__ import annotations

import argparse
from pathlib import Path

from mlengine.common.config import init_from_config, parse_config


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Export behaviour clone checkpoint to ONNX from YAML config.",
    )
    parser.add_argument(
        "--cfg",
        required=True,
        help="Path to YAML config, for example behavclon/configs/common.yaml",
    )
    return parser.parse_known_args()


def main() -> int:
    args, overrides = parse_args()
    config = parse_config(Path(args.cfg), overrides=overrides)
    deployer = init_from_config(config["export"])
    deployer.export()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
