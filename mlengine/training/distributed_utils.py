from __future__ import annotations

import os
from typing import Any

import torch


_local_rank: int = 0


def _dist_ready() -> bool:
    return torch.distributed.is_available() and torch.distributed.is_initialized()


def init_distributed_from_env() -> tuple[int, int]:
    """Initialize the default process group when ``WORLD_SIZE`` > 1 (e.g. ``torchrun``).

    Sets the current CUDA device to ``LOCAL_RANK`` when CUDA is available.

    Returns:
        ``(local_rank, world_size)``
    """
    global _local_rank
    world_sz = int(os.environ.get("WORLD_SIZE", "1"))
    if world_sz <= 1:
        _local_rank = 0
        return 0, 1
    if _dist_ready():
        _local_rank = int(os.environ.get("LOCAL_RANK", "0"))
        return _local_rank, torch.distributed.get_world_size()
    backend = "nccl" if torch.cuda.is_available() else "gloo"
    torch.distributed.init_process_group(backend=backend, init_method="env://")
    _local_rank = int(os.environ["LOCAL_RANK"])
    if torch.cuda.is_available():
        torch.cuda.set_device(_local_rank)
    return _local_rank, torch.distributed.get_world_size()


def cleanup_distributed() -> None:
    if not _dist_ready():
        return
    torch.distributed.destroy_process_group()


def is_distributed() -> bool:
    return _dist_ready()


def get_local_rank() -> int:
    return _local_rank if is_distributed() else 0


def get_rank() -> int:
    return torch.distributed.get_rank() if is_distributed() else 0


def get_world_size() -> int:
    return torch.distributed.get_world_size() if is_distributed() else 1


def is_main_process() -> bool:
    return get_rank() == 0


def broadcast_object_from_rank0(obj: Any) -> Any:
    """
    Pickle-broadcast a single object from rank 0 to all ranks.
    On rank 0, pass the value to send; on other ranks, ``obj`` is ignored (may be ``None``).
    """
    if not is_distributed():
        return obj
    obj_list = [obj if is_main_process() else None]
    torch.distributed.broadcast_object_list(obj_list, src=0)
    return obj_list[0]


def merge_gathered_sample_dicts(local: dict[str, Any]) -> dict[str, Any]:
    """
    Under DDP, merge per-rank ``{sample_id: Any}`` on rank 0 (same keys: last wins).
    Non-main ranks get ``{}`` so downstream logging runs once.
    """
    if not is_distributed():
        return dict(local)

    gathered: list[dict[str, Any] | None] = [None for _ in range(get_world_size())]
    torch.distributed.all_gather_object(gathered, dict(local))
    if not is_main_process():
        return {}
    merged: dict[str, Any] = {}
    for part in gathered:
        if part:
            merged.update(part)
    return dict(sorted(merged.items(), key=lambda kv: kv[0]))
