#!/usr/bin/env python3
"""Reboot active HELIOS head Dynamixel motors."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    from helios_core.utils.head_config import (
        DEFAULT_HEAD_ROLE,
        load_head_config,
        parse_disabled_motor_axes,
        parse_motor_ids,
    )
    from orca_core.scripts._dynamixel_preflight import (
        ADDR_TORQUE_ENABLE,
        DYNAMIXEL_REBOOT_SETTLE_SEC,
        _format_packet_status,
        _read_hardware_error_statuses,
    )
except ModuleNotFoundError:
    REPO_ROOT = Path(__file__).resolve().parents[3]
    ORCA_CORE_ROOT = REPO_ROOT / "orca_core"
    if str(ORCA_CORE_ROOT) not in sys.path:
        sys.path.insert(0, str(ORCA_CORE_ROOT))
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from helios_core.utils.head_config import (
        DEFAULT_HEAD_ROLE,
        load_head_config,
        parse_disabled_motor_axes,
        parse_motor_ids,
    )
    from orca_core.scripts._dynamixel_preflight import (
        ADDR_TORQUE_ENABLE,
        DYNAMIXEL_REBOOT_SETTLE_SEC,
        _format_packet_status,
        _read_hardware_error_statuses,
    )


HEAD_AXES = ("yaw", "pitch", "roll")


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reboot active HELIOS head Dynamixel motors.")
    parser.add_argument(
        "--config",
        default=None,
        help="Path to HELIOS head config YAML. Overrides --role when set.",
    )
    parser.add_argument(
        "--role",
        default=DEFAULT_HEAD_ROLE,
        help="HELIOS head role from configs/helios.yaml.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve config and print the reboot plan without connecting to hardware.",
    )
    return parser.parse_args(argv)


def _active_axis_ids(config: dict) -> list[tuple[str, int]]:
    motor_ids = parse_motor_ids(config)
    disabled_axes = set(parse_disabled_motor_axes(config))
    return [
        (axis, int(motor_ids.as_dict[axis]))
        for axis in HEAD_AXES
        if axis not in disabled_axes
    ]


def _print_plan(config: dict, cfg_path: Path, role: str) -> list[int]:
    hw_cfg = dict(config.get("hardware", {}))
    disabled_axes = parse_disabled_motor_axes(config)
    active_axis_ids = _active_axis_ids(config)
    active_ids = [motor_id for _axis, motor_id in active_axis_ids]
    print("HELIOS head reboot plan:")
    print(f"  role={role}")
    print(f"  config={cfg_path}")
    print(f"  port={hw_cfg.get('port', '')}")
    print(f"  baudrate={hw_cfg.get('baudrate', 3000000)}")
    print(f"  disabled_axes={disabled_axes}")
    print(f"  active_ids={active_ids}")
    for axis, motor_id in active_axis_ids:
        print(f"  {axis}: reboot and verify Hardware Error Status(70) on id={motor_id}")
    return active_ids


def _require_transport(config: dict) -> tuple[str, int]:
    hw_cfg = dict(config.get("hardware", {}))
    port = str(hw_cfg.get("port", "") or "").strip()
    if not port:
        raise ValueError("hardware.port is required for HELIOS head reboot.")
    baudrate = int(hw_cfg.get("baudrate", 3000000))
    return port, baudrate


def _reboot_active_ids(config: dict, role: str, active_ids: list[int]) -> bool:
    try:
        import dynamixel_sdk
    except ModuleNotFoundError:
        print(
            "ERROR: dynamixel_sdk is missing. Run this from the vision conda env.",
            file=sys.stderr,
        )
        return False

    if not active_ids:
        print(f"[{role}] ERROR: no active HELIOS head motor IDs selected.", file=sys.stderr)
        return False

    port, baudrate = _require_transport(config)
    port_handler = dynamixel_sdk.PortHandler(port)
    packet_handler = dynamixel_sdk.PacketHandler(2.0)
    packet_handler.dxl = dynamixel_sdk

    if not port_handler.openPort():
        print(f"[{role}] ERROR: failed to open port {port}", file=sys.stderr)
        return False

    try:
        if not port_handler.setBaudRate(baudrate):
            print(f"[{role}] ERROR: failed to set baudrate {baudrate}", file=sys.stderr)
            return False

        print(f"[{role}] hardware status before reboot:")
        _read_hardware_error_statuses(packet_handler, port_handler, active_ids)

        print(f"[{role}] disabling torque")
        for motor_id in active_ids:
            comm_result, dxl_error = packet_handler.write1ByteTxRx(
                port_handler,
                int(motor_id),
                ADDR_TORQUE_ENABLE,
                0,
            )
            packet_status = _format_packet_status(packet_handler, comm_result, dxl_error)
            print(f"    id={int(motor_id):02d} torque_disable={packet_status}")

        time.sleep(0.2)

        print(f"[{role}] rebooting motors")
        for motor_id in active_ids:
            comm_result, dxl_error = packet_handler.reboot(port_handler, int(motor_id))
            packet_status = _format_packet_status(packet_handler, comm_result, dxl_error)
            print(f"    id={int(motor_id):02d} reboot={packet_status}")
            time.sleep(0.25)

        time.sleep(DYNAMIXEL_REBOOT_SETTLE_SEC)

        print(f"[{role}] hardware status after reboot:")
        after_statuses = _read_hardware_error_statuses(packet_handler, port_handler, active_ids)
        failed_statuses = [status for status in after_statuses if not status.ok]
        if failed_statuses:
            print(
                f"[{role}] ERROR: HELIOS head reboot did not clear all selected motors.",
                file=sys.stderr,
            )
            for status in failed_statuses:
                print(f"[{role}] ERROR: {status.summary()}", file=sys.stderr)
            return False

        print(f"[{role}] HELIOS head reboot verified clean.")
        return True
    finally:
        port_handler.closePort()


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        config, cfg_path = load_head_config(args.config, role=args.role)
        active_ids = _print_plan(config, cfg_path, args.role)
        if args.dry_run:
            return 0
        return 0 if _reboot_active_ids(config, args.role, active_ids) else 1
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
