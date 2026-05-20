#!/usr/bin/env python3
"""Preflight enabled ORCA Dynamixel hand roles and reboot bad motor IDs."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    from openteach.helpers.orca_hand_roles import load_robot_orca_hardware_roles
    from orca_core.scripts._dynamixel_preflight import (
        _role_motor_ids,
        _role_motor_type,
        preflight_dynamixel_roles,
    )
    from orca_core.scripts._role_cli import print_role_summary, resolve_role
except ModuleNotFoundError:
    REPO_ROOT = Path(__file__).resolve().parents[3]
    ORCA_CORE_ROOT = REPO_ROOT / "orca_core"
    if str(ORCA_CORE_ROOT) not in sys.path:
        sys.path.insert(0, str(ORCA_CORE_ROOT))
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from openteach.helpers.orca_hand_roles import load_robot_orca_hardware_roles
    from orca_core.scripts._dynamixel_preflight import (
        _role_motor_ids,
        _role_motor_type,
        preflight_dynamixel_roles,
    )
    from orca_core.scripts._role_cli import print_role_summary, resolve_role


REPO_ROOT = Path(__file__).resolve().parents[3]


def _resolve_specs(robot: str, roles: list[str] | None):
    if roles:
        return [resolve_role(role) for role in roles]
    return load_robot_orca_hardware_roles(
        robot,
        repo_root=REPO_ROOT,
        validate_paths=True,
        require_calibration=False,
        require_ports=True,
    )


def _print_plan(specs) -> None:
    print("Dynamixel preflight plan:")
    for spec in specs:
        motor_type = _role_motor_type(spec)
        if motor_type == "dynamixel":
            action = "check Hardware Error Status(70); reboot only bad/nonresponsive IDs"
            motor_ids = _role_motor_ids(spec)
        else:
            action = "skip; motor_type is not dynamixel"
            motor_ids = []
        print(
            f"  {spec.role}: motor_type={motor_type}, port={spec.port}, "
            f"baudrate={spec.baudrate}, motor_ids={motor_ids}, action={action}"
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Check all enabled ORCA Dynamixel roles before hardware startup. "
            "If a required motor reports nonzero Hardware Error Status(70) or "
            "does not return a valid status packet, reboot only that bad motor ID "
            "and verify it clears before continuing."
        )
    )
    parser.add_argument(
        "--robot",
        choices=("ikarus", "helios"),
        default="helios",
        help="Robot config whose enabled ORCA hardware roles should be checked. Default: helios.",
    )
    parser.add_argument(
        "--role",
        action="append",
        dest="roles",
        help="Explicit ORCA hand role to check. May be repeated. Defaults to enabled roles for --robot.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve roles and print the preflight plan without connecting to hardware.",
    )
    args = parser.parse_args(argv)

    try:
        specs = _resolve_specs(args.robot, args.roles)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print("Resolved hand roles:")
    for spec in specs:
        print_role_summary(spec)
    _print_plan(specs)

    if args.dry_run:
        return 0

    return 0 if preflight_dynamixel_roles(specs) else 1


if __name__ == "__main__":
    raise SystemExit(main())
