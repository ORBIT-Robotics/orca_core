"""Scan common Feetech baudrates and report responding ORCA hand motors."""

from __future__ import annotations

import argparse
from dataclasses import dataclass

from orca_core.scripts.feetech_check import (
    DEFAULT_MOTOR_IDS,
    DEFAULT_BAUDRATE,
    check_motors,
    parse_motor_ids,
    resolve_role_defaults,
)


DEFAULT_ROLE = "helios_upper_left_feetech"
DEFAULT_BAUDRATES = (
    1_000_000,
    500_000,
    250_000,
    128_000,
    115_200,
    76_800,
    57_600,
    38_400,
)


@dataclass(frozen=True)
class BaudScanResult:
    baudrate: int
    responding_ids: tuple[int, ...]
    missing_ids: tuple[int, ...]
    error: str = ""

    @property
    def ok_count(self) -> int:
        return len(self.responding_ids)


def parse_baudrates(raw: str) -> tuple[int, ...]:
    baudrates: list[int] = []
    for part in str(raw).split(","):
        token = part.strip()
        if not token:
            continue
        baudrate = int(token)
        if baudrate <= 0:
            raise ValueError("Baudrates must be positive integers.")
        baudrates.append(baudrate)

    deduped = tuple(dict.fromkeys(baudrates))
    if not deduped:
        raise ValueError("At least one baudrate is required.")
    return deduped


def scan_baudrates(
    port: str,
    baudrates: tuple[int, ...],
    motor_ids: tuple[int, ...],
) -> list[BaudScanResult]:
    results: list[BaudScanResult] = []
    for baudrate in baudrates:
        try:
            motor_results = check_motors(port, baudrate, motor_ids)
        except Exception as exc:
            results.append(
                BaudScanResult(
                    baudrate=baudrate,
                    responding_ids=(),
                    missing_ids=motor_ids,
                    error=str(exc),
                )
            )
            continue

        responding = tuple(row.motor_id for row in motor_results if row.ok)
        missing = tuple(row.motor_id for row in motor_results if not row.ok)
        results.append(
            BaudScanResult(
                baudrate=baudrate,
                responding_ids=responding,
                missing_ids=missing,
            )
        )
    return results


def print_scan_results(results: list[BaudScanResult]) -> None:
    print("BAUDRATE  OK_COUNT  RESPONDING_IDS  MESSAGE")
    print("--------  --------  --------------  -------")
    for result in results:
        ids = ",".join(str(motor_id) for motor_id in result.responding_ids) or "-"
        message = result.error or ("ok" if result.responding_ids else "no motors responded")
        print(f"{result.baudrate:>8}  {result.ok_count:>8}  {ids:<14}  {message}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Scan common Feetech baudrates for responding ORCA hand motors."
    )
    parser.add_argument(
        "--role",
        default=DEFAULT_ROLE,
        help=f"ORCA hardware role to read port from. Default: {DEFAULT_ROLE}",
    )
    parser.add_argument("--port", default="", help="Serial port override.")
    parser.add_argument(
        "--baudrates",
        default=",".join(str(value) for value in DEFAULT_BAUDRATES),
        help="Comma-separated baudrates to scan.",
    )
    parser.add_argument(
        "--ids",
        default="1-17",
        help="Comma-separated motor IDs/ranges to scan. Default: 1-17.",
    )
    parser.add_argument(
        "--stop-on-full-match",
        action="store_true",
        help="Stop scanning once all requested motor IDs respond at one baudrate.",
    )
    args = parser.parse_args()

    try:
        role_port, _ = resolve_role_defaults(args.role)
        motor_ids = parse_motor_ids(args.ids)
        baudrates = parse_baudrates(args.baudrates)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 2

    port = args.port.strip() or role_port
    if not port:
        print("ERROR: No Feetech serial port configured. Pass --port or use a valid --role.")
        return 2

    print(f"Scanning Feetech motors {motor_ids} on {port}...")
    results: list[BaudScanResult] = []
    for result in scan_baudrates(port, baudrates, motor_ids):
        results.append(result)
        if args.stop_on_full_match and result.responding_ids == motor_ids:
            break

    print_scan_results(results)
    best = max(results, key=lambda item: item.ok_count, default=None)
    if best is None or best.ok_count == 0:
        print("FAILED: No motors responded at any scanned baudrate.")
        return 1

    if best.responding_ids == motor_ids:
        print(f"OK: all requested motors responded at {best.baudrate} baud.")
        return 0

    print(
        f"PARTIAL: best baudrate is {best.baudrate}, "
        f"{best.ok_count}/{len(motor_ids)} motors responded."
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
