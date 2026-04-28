"""CLI for ORCA hand calibration."""

from __future__ import annotations

import argparse

try:
    from orca_core.scripts._role_cli import (
        add_role_argument,
        create_hand,
        print_role_summary,
        resolve_role,
    )
except ModuleNotFoundError:
    from _role_cli import add_role_argument, create_hand, print_role_summary, resolve_role


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate ORCA hand.")
    add_role_argument(parser)
    args = parser.parse_args()

    try:
        role = resolve_role(args.role)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1
    print_role_summary(role)
    hand = create_hand(role)
    success, message = hand.connect()
    print((success, message))
    if not success:
        return 1

    hand.calibrate()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
