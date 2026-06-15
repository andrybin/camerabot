import argparse
from pathlib import Path

from mlengine.common.config import init_from_config, parse_config


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Convert behaviour_recorder data to images/ + markup.json from YAML config.",
    )
    parser.add_argument(
        "--cfg",
        required=True,
        help="Path to YAML config, for example behavclon/configs/behaviour_convert.yaml",
    )
    return parser.parse_known_args()


def main() -> int:
    args, overrides = parse_args()
    cfg = parse_config(Path(args.cfg), overrides=overrides)
    converter = init_from_config(cfg["converter"])
    markup_updated = converter.prepare()
    init_from_config(cfg["dataset_train_val_splitter"]).split_and_save(markup_updated)


if __name__ == "__main__":
    raise SystemExit(main())
