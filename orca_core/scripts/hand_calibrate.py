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
    parser.add_argument(
        "--force-wrist",
        action="store_true",
        help="Recalibrate the wrist even if it is already marked as calibrated.",
    )
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

    try:
        print("Starting calibration...")
        hand.calibrate(force_wrist=args.force_wrist)
        print("Calibration complete.")
        return 0
    finally:
        hand.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
