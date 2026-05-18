"""Check that all expected Feetech motors are reachable on the ORCA hand bus."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Iterable

try:
    from hardware.feetech import (
        COMM_SUCCESS,
        PortHandler,
        SMS_STS_PRESENT_CURRENT_L,
        SMS_STS_PRESENT_TEMPERATURE,
        sms_sts,
    )
except ModuleNotFoundError:
    ORCA_CORE_ROOT = Path(__file__).resolve().parents[2]
    if str(ORCA_CORE_ROOT) not in sys.path:
        sys.path.insert(0, str(ORCA_CORE_ROOT))
    from hardware.feetech import (
        COMM_SUCCESS,
        PortHandler,
        SMS_STS_PRESENT_CURRENT_L,
        SMS_STS_PRESENT_TEMPERATURE,
        sms_sts,
    )


DEFAULT_ROLE = "helios_upper_right_feetech"
DEFAULT_MOTOR_IDS = tuple(range(1, 18))
DEFAULT_BAUDRATE = 1_000_000
CURRENT_MA_PER_UNIT = 6.5


@dataclass(frozen=True)
class MotorCheckResult:
    motor_id: int
    ok: bool
    model: int | None
    position: int | None
    speed: int | None
    current_ma: float | None
    temperature_c: float | None
    result: int
    error: int
    message: str


def parse_motor_ids(raw: str) -> tuple[int, ...]:
    """Parse comma-separated motor IDs and inclusive ranges like ``1-17``."""
    ids: list[int] = []
    for part in str(raw).split(","):
        token = part.strip()
        if not token:
            continue
        if "-" in token:
            start_raw, end_raw = token.split("-", 1)
            start = int(start_raw)
            end = int(end_raw)
            if end < start:
                raise ValueError(f"Invalid descending motor ID range: {token}")
            ids.extend(range(start, end + 1))
        else:
            ids.append(int(token))

    deduped = tuple(dict.fromkeys(ids))
    if not deduped:
        raise ValueError("At least one motor ID is required.")
    if any(motor_id < 0 or motor_id > 252 for motor_id in deduped):
        raise ValueError("Motor IDs must be in the Feetech range 0..252.")
    return deduped


def resolve_role_defaults(role: str) -> tuple[str | None, int | None]:
    if not role:
        return None, None

    repo_root = Path(__file__).resolve().parents[3]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

    try:
        from openteach.helpers.orca_hand_roles import load_orca_hand_role
    except ModuleNotFoundError:
        return None, None

    spec = load_orca_hand_role(
        role,
        repo_root=repo_root,
        validate_paths=True,
        require_enabled=False,
        require_port=False,
    )
    return spec.port, spec.baudrate


def _format_comm_message(packet: sms_sts, result: int, error: int) -> str:
    result_text = packet.getTxRxResult(result) if result != COMM_SUCCESS else "ok"
    error_text = packet.getRxPacketError(error) if error else ""
    return "; ".join(part for part in (result_text, error_text) if part)


def check_motor(packet: sms_sts, motor_id: int) -> MotorCheckResult:
    model, result, error = packet.ping(motor_id)
    if result != COMM_SUCCESS or error != 0:
        return MotorCheckResult(
            motor_id=motor_id,
            ok=False,
            model=None,
            position=None,
            speed=None,
            current_ma=None,
            temperature_c=None,
            result=result,
            error=error,
            message=_format_comm_message(packet, result, error),
        )

    position, speed, pos_result, pos_error = packet.ReadPosSpeed(motor_id)
    current_raw, cur_result, cur_error = packet.read2ByteTxRx(
        motor_id, SMS_STS_PRESENT_CURRENT_L
    )
    temp_raw, temp_result, temp_error = packet.read1ByteTxRx(
        motor_id, SMS_STS_PRESENT_TEMPERATURE
    )

    result_values = (pos_result, cur_result, temp_result)
    error_values = (pos_error, cur_error, temp_error)
    ok = all(value == COMM_SUCCESS for value in result_values) and all(
        value == 0 for value in error_values
    )
    result_code = next((value for value in result_values if value != COMM_SUCCESS), COMM_SUCCESS)
    error_code = next((value for value in error_values if value != 0), 0)

    current_signed = packet.scs_tohost(current_raw, 15)
    return MotorCheckResult(
        motor_id=motor_id,
        ok=ok,
        model=model,
        position=position if pos_result == COMM_SUCCESS and pos_error == 0 else None,
        speed=speed if pos_result == COMM_SUCCESS and pos_error == 0 else None,
        current_ma=(
            current_signed * CURRENT_MA_PER_UNIT
            if cur_result == COMM_SUCCESS and cur_error == 0
            else None
        ),
        temperature_c=(
            float(temp_raw)
            if temp_result == COMM_SUCCESS and temp_error == 0
            else None
        ),
        result=result_code,
        error=error_code,
        message=_format_comm_message(packet, result_code, error_code),
    )


def check_motors(port: str, baudrate: int, motor_ids: Iterable[int]) -> list[MotorCheckResult]:
    port_handler = PortHandler(port)
    port_handler.baudrate = int(baudrate)
    if not port_handler.openPort():
        raise OSError(f"Failed to open Feetech port {port} at baudrate {baudrate}.")

    try:
        packet = sms_sts(port_handler)
        return [check_motor(packet, motor_id) for motor_id in motor_ids]
    finally:
        port_handler.closePort()


def print_results(results: Iterable[MotorCheckResult]) -> None:
    rows = list(results)
    print("ID  STATUS   MODEL  POS_RAW  SPEED_RAW  CURRENT_MA  TEMP_C  MESSAGE")
    print("--  -------  -----  -------  ---------  ----------  ------  -------")
    for row in rows:
        status = "OK" if row.ok else "MISSING"
        model = str(row.model) if row.model is not None else "-"
        position = str(row.position) if row.position is not None else "-"
        speed = str(row.speed) if row.speed is not None else "-"
        current = f"{row.current_ma:.1f}" if row.current_ma is not None else "-"
        temp = f"{row.temperature_c:.0f}" if row.temperature_c is not None else "-"
        print(
            f"{row.motor_id:>2}  {status:<7}  {model:>5}  {position:>7}  "
            f"{speed:>9}  {current:>10}  {temp:>6}  {row.message}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check that all 17 Feetech ORCA hand motors respond."
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
    parser.add_argument(
        "--ids",
        default="1-17",
        help="Comma-separated motor IDs/ranges to check. Default: 1-17.",
    )
    args = parser.parse_args()

    try:
        role_port, role_baudrate = resolve_role_defaults(args.role)
        motor_ids = parse_motor_ids(args.ids)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 2

    port = args.port.strip() or role_port
    baudrate = args.baudrate or role_baudrate or DEFAULT_BAUDRATE
    if not port:
        print("ERROR: No Feetech serial port configured. Pass --port or use a valid --role.")
        return 2

    print(f"Checking Feetech motors {motor_ids} on {port} at {baudrate} baud...")
    try:
        results = check_motors(port, baudrate, motor_ids)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1

    print_results(results)
    missing = [row.motor_id for row in results if not row.ok]
    if missing:
        print(f"FAILED: Missing or unreadable motors: {missing}")
        return 1

    expected_count = len(DEFAULT_MOTOR_IDS) if motor_ids == DEFAULT_MOTOR_IDS else len(motor_ids)
    print(f"OK: {expected_count} Feetech motors responded.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
