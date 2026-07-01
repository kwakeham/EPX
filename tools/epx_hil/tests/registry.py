"""Test-suite registry: ``@hil_test(name, help)`` registers a callable ``fn(ctx)``.

``ctx`` is an :class:`~epx_hil.context.Context` (link, session, gear_table, args,
log, ...). Suites log moves via ``epx_hil.moves`` and non-move pass/fail via
``ctx.session.add_check(...)`` so both feed the run summary and exit code.
"""

from __future__ import annotations

from typing import Callable

_REGISTRY: dict[str, tuple[Callable, str]] = {}


def hil_test(name: str, help: str = ""):
    def deco(fn: Callable) -> Callable:
        _REGISTRY[name] = (fn, help)
        return fn
    return deco


def names() -> list[str]:
    return list(_REGISTRY)


def list_tests() -> list[tuple[str, str]]:
    return [(n, h) for n, (f, h) in _REGISTRY.items()]


def run(name: str, ctx):
    fn, _ = _REGISTRY[name]
    return fn(ctx)
