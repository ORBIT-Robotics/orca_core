#!/usr/bin/env python3
"""Tension all selected HELIOS ORCA hand roles concurrently."""

from __future__ import annotations

import sys
from pathlib import Path

try:
    from orca_core.scripts import hand_tension
except ModuleNotFoundError:
    REPO_ROOT = Path(__file__).resolve().parents[3]
    ORCA_CORE_ROOT = REPO_ROOT / "orca_core"
    if str(ORCA_CORE_ROOT) not in sys.path:
        sys.path.insert(0, str(ORCA_CORE_ROOT))
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from orca_core.scripts import hand_tension


def main(argv: list[str] | None = None) -> int:
    return hand_tension.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
