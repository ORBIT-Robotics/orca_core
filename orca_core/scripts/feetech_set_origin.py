"""Set Feetech present positions to a chosen raw origin using EEPROM offsets."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import Iterable

try:
    from hardware.feetech import (
        COMM_SUCCESS,
        PortHandler,
        SMS_STS_TORQUE_ENABLE,
        sms_sts,
    )
except ModuleNotFoundError:
    ORCA_CORE_ROOT = Path(__file__).resolve().parents[2]
    if str(ORCA_CORE_ROOT) not in sys.path:
        sys.path.insert(0, str(ORCA_CORE_ROOT))
    from hardware.feetech import (
        COMM_SUCCESS,
        PortHandler,
        SMS_STS_TORQUE_ENABLE,
        sms_sts,
    )

from orca_core.scripts.feetech_check import parse_motor_ids, resolve_role_defaults


DEFAULT_ROLE = "helios_lower_right_feetech"
DEFAULT_BAUDRATE = 1_000_000
DEFAULT_TARGET_RAW = 2048
DEFAULT_MOTOR_IDS = tuple(range(1, 18))


def validate_target_raw(value: int) -> int:
    value = int(value)
    if value < 0 or value > 4095:
        raise ValueError("Target raw position must be in Feetech servo range 0..4095.")
    return value


def _comm_message(packet: sms_sts, result: int, error: int) -> str:
    result_text = packet.getTxRxResult(result) if result != COMM_SUCCESS else "ok"
    error_text = packet.getRxPacketError(error) if error else ""
    return "; ".join(part for part in (result_text, error_text) if part)


def _read_position(packet: sms_sts, motor_id: int) -> tuple[int | None, str]:
    position, result, error = packet.ReadPos(motor_id)
    if result != COMM_SUCCESS or error != 0:
        return None, _comm_message(packet, result, error)
    return int(position), "ok"


def set_origin(
    port: str,
    baudrate: int,
    motor_ids: Iterable[int],
    target_raw: int,
    apply: bool,
) -> bool:
    motor_ids = tuple(motor_ids)
    target_raw = validate_target_raw(target_raw)

    port_handler = PortHandler(port)
    port_handler.baudrate = int(baudrate)
    if not port_handler.openPort():
        raise OSError(f"Failed to open Feetech port {port} at baudrate {baudrate}.")

    all_ok = True
    try:
        packet = sms_sts(port_handler)
        print(f"Feetech origin target: raw {target_raw}")
        if not apply:
            print("DRY RUN: pass --apply to write persistent EEPROM offsets.")

        print("ID  BEFORE  AFTER   STATUS")
        print("--  ------  ------  ------")
        for motor_id in motor_ids:
            before, before_msg = _read_position(packet, motor_id)
            if before is None:
                all_ok = False
                print(f"{motor_id:>2}  {'-':>6}  {'-':>6}  read failed: {before_msg}")
                continue

            if not apply:
                print(f"{motor_id:>2}  {before:>6}  {'dry':>6}  would set current position to {target_raw}")
                continue

            packet.write1ByteTxRx(motor_id, SMS_STS_TORQUE_ENABLE, 0)
            time.sleep(0.02)
            result, error = packet.reOfsCal(motor_id, target_raw)
            time.sleep(0.05)
            after, after_msg = _read_position(packet, motor_id)

            ok = result == COMM_SUCCESS and error == 0 and after is not None
            all_ok = all_ok and ok
            status = _comm_message(packet, result, error)
            if after_msg != "ok":
                status = f"{status}; read after: {after_msg}"
            print(
                f"{motor_id:>2}  {before:>6}  {str(after) if after is not None else '-':>6}  {status}"
            )

        return all_ok
    finally:
        port_handler.closePort()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Set selected Feetech motors so their current physical position "
            "reports as a chosen raw encoder value. This writes persistent offsets."
        )
    )
    parser.add_argument(
        "--role",
        default=DEFAULT_ROLE,
        help=f"ORCA hardware role to read port/baudrate from. Default: {DEFAULT_ROLE}",
    )
    parser.add_argument("--port", default="", help="Serial port override.")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=0,
        help=f"Serial baudrate override. Default from role or {DEFAULT_BAUDRATE}.",
    )
    motor_group = parser.add_mutually_exclusive_group(required=True)
    motor_group.add_argument(
        "--ids",
        help="Comma-separated motor IDs/ranges to update, for example 13 or 1-17.",
    )
    motor_group.add_argument(
        "--all",
        action="store_true",
        help="Update all 17 Feetech hand motors.",
    )
    parser.add_argument(
        "--target-raw",
        type=validate_target_raw,
        default=DEFAULT_TARGET_RAW,
        help=f"Raw position to assign to the current physical position. Default: {DEFAULT_TARGET_RAW}.",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Actually write persistent Feetech offsets. Without this, only prints a dry run.",
    )
    args = parser.parse_args()

    try:
        role_port, role_baudrate = resolve_role_defaults(args.role)
        motor_ids = DEFAULT_MOTOR_IDS if args.all else parse_motor_ids(args.ids)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 2

    port = args.port.strip() or role_port
    baudrate = args.baudrate or role_baudrate or DEFAULT_BAUDRATE
    if not port:
        print("ERROR: No Feetech serial port configured. Pass --port or use a valid --role.")
        return 2

    print(f"Using Feetech motors {motor_ids} on {port} at {baudrate} baud...")
    try:
        ok = set_origin(port, baudrate, motor_ids, args.target_raw, args.apply)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
