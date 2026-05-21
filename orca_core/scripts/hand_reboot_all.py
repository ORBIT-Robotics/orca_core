#!/usr/bin/env python3
"""Reboot selected HELIOS ORCA hand motors without starting calibration."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    from orca_core.scripts._role_cli import print_role_summary
    from orca_core.scripts.hand_calibrate_all import (
        DEFAULT_ROLES,
        _reboot_dynamixel_roles,
        _role_motor_ids,
        _role_motor_type,
        _validate_roles,
    )
except ModuleNotFoundError:
    REPO_ROOT = Path(__file__).resolve().parents[3]
    ORCA_CORE_ROOT = REPO_ROOT / "orca_core"
    if str(ORCA_CORE_ROOT) not in sys.path:
        sys.path.insert(0, str(ORCA_CORE_ROOT))
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from orca_core.scripts._role_cli import print_role_summary
    from orca_core.scripts.hand_calibrate_all import (
        DEFAULT_ROLES,
        _reboot_dynamixel_roles,
        _role_motor_ids,
        _role_motor_type,
        _validate_roles,
    )


def _print_reboot_plan(specs) -> None:
    print("Reboot plan:")
    for spec in specs:
        motor_type = _role_motor_type(spec)
        motor_ids = _role_motor_ids(spec)
        if motor_type == "dynamixel":
            action = "reboot and verify Hardware Error Status(70)"
        else:
            action = "skip; current shared calibration logic only reboots Dynamixel roles"
        print(f"  {spec.role}: motor_type={motor_type}, motor_ids={motor_ids}, action={action}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Reboot selected HELIOS ORCA hand motors using the same safe "
            "Dynamixel reboot path as hand_calibrate_all.py --reboot."
        )
    )
    parser.add_argument(
        "--role",
        action="append",
        dest="roles",
        help=(
            "Role to reboot. May be repeated. "
            f"Default: {', '.join(DEFAULT_ROLES)}."
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve roles and print the reboot plan without connecting to hardware.",
    )
    args = parser.parse_args(argv)

    role_names = args.roles or list(DEFAULT_ROLES)
    try:
        specs = _validate_roles(role_names)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print("Resolved hand roles:")
    for spec in specs:
        print_role_summary(spec)
    _print_reboot_plan(specs)

    if args.dry_run:
        return 0

    return 0 if _reboot_dynamixel_roles(specs) else 1


if __name__ == "__main__":
    raise SystemExit(main())
