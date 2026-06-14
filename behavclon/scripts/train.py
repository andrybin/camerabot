from __future__ import annotations

import argparse
import copy
from pathlib import Path
from mlengine.common.config import init_from_config, parse_config


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Train behaviour clone model from YAML config.",
    )
    parser.add_argument(
        "--cfg",
        required=True,
        help="Path to YAML config, for example behavclon/configs/behaviour_clone.yaml",
    )
    return parser.parse_known_args()


def main() -> int:
    from mlengine.training.distributed_utils import (
        cleanup_distributed,
        init_distributed_from_env,
        is_main_process,
    )

    init_distributed_from_env()
    pipeline = None
    try:
        args, overrides = parse_args()
        config = parse_config(Path(args.cfg), overrides=overrides)
        config_snapshot = copy.deepcopy(config)
        pipeline = init_from_config(config["train_pipeline"])
        if is_main_process():
            pipeline.monitor.connect_experiment_config(config_snapshot)
        pipeline.train()
    finally:
        cleanup_distributed()
        if (
            pipeline is not None
            and is_main_process()
            and hasattr(pipeline.monitor, "close")
        ):
            pipeline.monitor.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
