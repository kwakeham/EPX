"""Shared run context handed to test-battery suites (keeps cli<->tests decoupled)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .link import SerialLink
from .logging_ import LogSession
from .models import GearTable


@dataclass
class Context:
    link: SerialLink
    session: LogSession
    gear_table: GearTable | None
    args: object
    log: Callable[[str], None]
    settle_timeout: float = 12.0
    tele_div: int = 2
