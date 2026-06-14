import sys
from loguru import logger


def setup_logger(name: str) -> logger:
    """
    Setup the logger.
    """
    log = logger.bind(name=name)
    log.remove()
    log.add(
        sys.stdout,
        level="INFO",
        format=(
            "<green>{time:YYYY-MM-DD HH:mm:ss}</green> | "
            "<level>{level: <8}</level> | "
            "<cyan>{extra[name]}</cyan> | "
            "<level>{message}</level>"
        ),
    )
    return log
