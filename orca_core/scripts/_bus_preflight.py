"""Shared ORCA hand serial-bus preflight helpers."""

from __future__ import annotations

import sys
from typing import Iterable

from orca_core.scripts._dynamixel_preflight import (
    _role_motor_ids,
    _role_motor_type,
    preflight_dynamixel_role,
)
from orca_core.scripts.feetech_check import check_motors, print_results


def preflight_feetech_role(spec) -> bool:
    """Read-check every configured Feetech motor before opening the runtime."""
    motor_type = _role_motor_type(spec)
    if motor_type != "feetech":
        print(f"[{spec.role}] skipping Feetech preflight for motor_type={motor_type}")
        return True

    motor_ids = _role_motor_ids(spec)
    print(
        f"[{spec.role}] Feetech reachability check on IDs {motor_ids} "
        f"at {spec.port} baudrate={spec.baudrate}:"
    )
    try:
        results = check_motors(spec.port, int(spec.baudrate), motor_ids)
    except Exception as exc:
        print(
            f"[{spec.role}] ERROR: Feetech transport check failed: "
            f"{type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return False

    print_results(results)
    failed_ids = [row.motor_id for row in results if not row.ok]
    if failed_ids:
        print(
            f"[{spec.role}] ERROR: Missing or unreadable Feetech motor IDs: {failed_ids}",
            file=sys.stderr,
        )
        return False

    print(f"[{spec.role}] Feetech reachability preflight clean.")
    return True


def preflight_motor_role(spec) -> bool:
    """Run the appropriate preflight for a configured ORCA hand role."""
    motor_type = _role_motor_type(spec)
    if motor_type == "dynamixel":
        return preflight_dynamixel_role(spec)
    if motor_type == "feetech":
        return preflight_feetech_role(spec)

    print(
        f"[{spec.role}] ERROR: Unsupported ORCA motor_type={motor_type!r}.",
        file=sys.stderr,
    )
    return False


def preflight_motor_roles(specs: Iterable[object]) -> bool:
    """Preflight every selected ORCA hand role."""
    ok = True
    selected_any = False
    for spec in specs:
        selected_any = True
        if not preflight_motor_role(spec):
            ok = False
    if not selected_any:
        print("No ORCA hand roles selected for bus preflight.")
    return ok
