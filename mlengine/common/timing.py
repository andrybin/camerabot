from __future__ import annotations

import time
from collections import defaultdict
from contextlib import AbstractContextManager, contextmanager, nullcontext
from typing import TYPE_CHECKING, Iterator

# TODO get logger from host 
if TYPE_CHECKING:
    from loguru import Logger


def _timing_table_lines(
    totals: dict[str, float],
    counts: dict[str, int],
) -> list[str]:
    rows: list[tuple[str, float, float, int]] = []
    for name in sorted(totals.keys()):
        total_s = totals[name]
        n = max(counts.get(name, 0), 1)
        rows.append((name, total_s * 1000.0, (total_s / n) * 1000.0, int(counts.get(name, n))))

    w_block = max(len("block"), *(len(r[0]) for r in rows)) if rows else len("block")
    header = f"{'block':<{w_block}}  {'total_ms':>10}  {'mean_ms':>10}  {'n':>6}"
    sep = f"{'-' * w_block}  {'-' * 10}  {'-' * 10}  {'-' * 6}"
    body = [
        f"{name:<{w_block}}  {total_ms:>10.2f}  {mean_ms:>10.2f}  {n:>6d}"
        for name, total_ms, mean_ms, n in rows
    ]
    return ["[profile] timing summary", header, sep, *body]


def _maybe_cuda_sync() -> None:
    import torch

    if torch.cuda.is_available():
        torch.cuda.synchronize()


class Profiler:
    """
    Scoped timing for scripts: optional per-scope log line, always aggregates for a final summary.
    Use log_each=True for one-off sections; keep False inside tight loops to avoid log spam.
    """

    def __init__(self, enabled: bool, logger: Logger) -> None:
        self._enabled = enabled
        self._logger = logger
        self._totals: dict[str, float] = defaultdict(float)
        self._counts: dict[str, int] = defaultdict(int)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @contextmanager
    def scope(
        self,
        name: str,
        *,
        log_each: bool = False,
        cuda_sync: bool = False,
    ) -> Iterator[None]:
        if not self._enabled:
            yield
            return
        t0 = time.perf_counter()
        try:
            yield
        finally:
            if cuda_sync:
                _maybe_cuda_sync()
            dt = time.perf_counter() - t0
            self._totals[name] += dt
            self._counts[name] += 1
            if log_each:
                self._logger.info(f"[profile] {name}: {dt * 1000:.2f} ms")

    def summarize(self) -> None:
        if not self._enabled or not self._totals:
            return
        lines = _timing_table_lines(dict(self._totals), dict(self._counts))
        print("\n".join(lines), flush=True)


def profiler_scope(
    profiler: Profiler | None,
    name: str,
    *,
    log_each: bool = False,
    cuda_sync: bool = False,
) -> AbstractContextManager[None]:
    """Context manager that no-ops when profiler is None or disabled."""
    if profiler is None or not profiler.enabled:
        return nullcontext()
    return profiler.scope(name, log_each=log_each, cuda_sync=cuda_sync)
