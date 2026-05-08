"""CLI for moving ORCA hand to neutral position."""

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
    parser = argparse.ArgumentParser(description="Move ORCA hand to neutral position.")
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
    if not success:
        print(f"Failed to connect: {message}")
        return 1

    try:
        hand.enable_torque()
        hand.set_neutral_position()
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        try:
            hand.disable_torque()
        finally:
            hand.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
