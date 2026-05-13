#!/usr/bin/env python3
"""Calibrate all HELIOS ORCA hands concurrently."""

from __future__ import annotations

import argparse
import os
from collections import Counter
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    from orca_core.scripts._role_cli import print_role_summary, resolve_role
    from orca_core.utils.yaml_io import read_yaml
except ModuleNotFoundError:
    REPO_ROOT = Path(__file__).resolve().parents[3]
    ORCA_CORE_ROOT = REPO_ROOT / "orca_core"
    if str(ORCA_CORE_ROOT) not in sys.path:
        sys.path.insert(0, str(ORCA_CORE_ROOT))
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from orca_core.scripts._role_cli import print_role_summary, resolve_role
    from orca_core.utils.yaml_io import read_yaml


DEFAULT_ROLES = (
    "helios_upper_left_feetech",
    "helios_upper_right_feetech",
    "helios_lower_left",
    "helios_lower_right",
)

ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
DYNAMIXEL_REBOOT_SETTLE_SEC = 1.5


def _default_log_dir() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return Path.home() / "orca_hand_calibration_logs" / timestamp


def _build_child_command(role: str, force_wrist: bool) -> list[str]:
    script_path = Path(__file__).resolve().with_name("hand_calibrate.py")
    command = [sys.executable, str(script_path), "--role", role]
    if force_wrist:
        command.append("--force-wrist")
    return command


def _load_model_config(spec) -> dict:
    config = read_yaml(str(spec.model_path / "config.yaml"))
    if not isinstance(config, dict):
        raise ValueError(f"{spec.role} model config must be a mapping.")
    return config


def _role_motor_type(spec) -> str:
    config = _load_model_config(spec)
    return str(config.get("motor_type", "dynamixel")).strip().lower()


def _role_motor_ids(spec) -> list[int]:
    config = _load_model_config(spec)
    motor_ids = config.get("motor_ids")
    if not isinstance(motor_ids, list) or not motor_ids:
        raise ValueError(
            f"{spec.role} model config must define a non-empty motor_ids list."
        )
    return [int(motor_id) for motor_id in motor_ids]


def _format_packet_status(packet_handler, comm_result: int, dxl_error: int) -> str:
    parts = []
    if comm_result != packet_handler.dxl.COMM_SUCCESS:
        parts.append(packet_handler.getTxRxResult(comm_result))
    if dxl_error:
        parts.append(packet_handler.getRxPacketError(dxl_error))
    return "; ".join(parts) if parts else "ok"


def _read_hardware_errors(packet_handler, port_handler, motor_ids: list[int]) -> dict[int, int | None]:
    statuses: dict[int, int | None] = {}
    for motor_id in motor_ids:
        value, comm_result, dxl_error = packet_handler.read1ByteTxRx(
            port_handler,
            motor_id,
            ADDR_HARDWARE_ERROR_STATUS,
        )
        if comm_result != packet_handler.dxl.COMM_SUCCESS:
            packet_status = _format_packet_status(packet_handler, comm_result, dxl_error)
            print(f"    id={motor_id:02d} hw=???? status={packet_status}")
            statuses[motor_id] = None
            continue
        print(f"    id={motor_id:02d} hw=0x{int(value):02x}")
        statuses[motor_id] = int(value)
    return statuses


def _reboot_dynamixel_role(spec) -> bool:
    try:
        import dynamixel_sdk
    except ModuleNotFoundError:
        print(
            "ERROR: dynamixel_sdk is missing. Run this from the control conda env.",
            file=sys.stderr,
        )
        return False

    motor_ids = _role_motor_ids(spec)
    print(f"[{spec.role}] rebooting Dynamixel IDs {motor_ids}")
    port_handler = dynamixel_sdk.PortHandler(spec.port)
    packet_handler = dynamixel_sdk.PacketHandler(2.0)
    packet_handler.dxl = dynamixel_sdk

    if not port_handler.openPort():
        print(f"[{spec.role}] ERROR: failed to open port {spec.port}", file=sys.stderr)
        return False

    try:
        if not port_handler.setBaudRate(spec.baudrate):
            print(
                f"[{spec.role}] ERROR: failed to set baudrate {spec.baudrate}",
                file=sys.stderr,
            )
            return False

        print(f"[{spec.role}] hardware status before reboot:")
        _read_hardware_errors(packet_handler, port_handler, motor_ids)

        print(f"[{spec.role}] disabling torque")
        for motor_id in motor_ids:
            comm_result, dxl_error = packet_handler.write1ByteTxRx(
                port_handler,
                motor_id,
                ADDR_TORQUE_ENABLE,
                0,
            )
            packet_status = _format_packet_status(packet_handler, comm_result, dxl_error)
            print(f"    id={motor_id:02d} torque_disable={packet_status}")

        time.sleep(0.2)

        print(f"[{spec.role}] rebooting motors")
        for motor_id in motor_ids:
            comm_result, dxl_error = packet_handler.reboot(port_handler, motor_id)
            packet_status = _format_packet_status(packet_handler, comm_result, dxl_error)
            print(f"    id={motor_id:02d} reboot={packet_status}")
            time.sleep(0.25)

        time.sleep(DYNAMIXEL_REBOOT_SETTLE_SEC)

        print(f"[{spec.role}] hardware status after reboot:")
        after_statuses = _read_hardware_errors(packet_handler, port_handler, motor_ids)
        failed_ids = [
            motor_id
            for motor_id, status in after_statuses.items()
            if status is None or status != 0
        ]
        if failed_ids:
            print(
                f"[{spec.role}] ERROR: hardware status still not clear for IDs {failed_ids}.",
                file=sys.stderr,
            )
            return False

        print(f"[{spec.role}] Dynamixel reboot verified clean.")
        return True
    finally:
        port_handler.closePort()


def _reboot_dynamixel_roles(specs) -> bool:
    dynamixel_specs = []
    for spec in specs:
        motor_type = _role_motor_type(spec)
        if motor_type == "dynamixel":
            dynamixel_specs.append(spec)
        else:
            print(f"[{spec.role}] skipping reboot for motor_type={motor_type}")

    if not dynamixel_specs:
        print("No Dynamixel roles selected for reboot.")
        return True

    ok = True
    for spec in dynamixel_specs:
        if not _reboot_dynamixel_role(spec):
            ok = False
    return ok


def _stream_child_output(
    role: str,
    process: subprocess.Popen[str],
    log_file,
    verbose_console: bool,
) -> None:
    assert process.stdout is not None
    suppressed_counts: Counter[str] = Counter()

    def flush_suppressed() -> None:
        if not suppressed_counts:
            return
        details = ", ".join(
            f"{label}={count}" for label, count in sorted(suppressed_counts.items())
        )
        print(
            f"[{role}] suppressed repeated low-level Dynamixel logs: {details}",
            flush=True,
        )
        suppressed_counts.clear()

    for line in process.stdout:
        text = line.rstrip("\n")
        log_file.write(line)
        log_file.flush()
        if not verbose_console:
            category = _low_level_dynamixel_log_category(text)
            if category is not None:
                suppressed_counts[category] += 1
                continue
        flush_suppressed()
        print(f"[{role}] {text}", flush=True)

    flush_suppressed()


def _low_level_dynamixel_log_category(text: str) -> str | None:
    if text.startswith("ERROR:root:> write_byte:"):
        if "Hardware error occurred" in text:
            return "hardware_error_packets"
        if "There is no status packet" in text:
            return "no_status_packets"
        return "write_byte_errors"
    if text.startswith("ERROR:root:Could not set torque "):
        return "torque_retry_summaries"
    return None


def _terminate_processes(processes: dict[str, subprocess.Popen[str]]) -> None:
    for process in processes.values():
        if process.poll() is None:
            process.terminate()

    deadline = time.monotonic() + 5.0
    for process in processes.values():
        remaining = max(0.0, deadline - time.monotonic())
        try:
            process.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            process.kill()


def _validate_roles(role_names: list[str]):
    specs = []
    for role_name in role_names:
        spec = resolve_role(role_name)
        specs.append(spec)

    ports: dict[str, str] = {}
    for spec in specs:
        normalized_port = spec.port.strip()
        if normalized_port in ports:
            raise ValueError(
                f"Roles {ports[normalized_port]} and {spec.role} share port {normalized_port!r}."
            )
        ports[normalized_port] = spec.role

    return specs


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Calibrate all HELIOS ORCA hand roles concurrently."
    )
    parser.add_argument(
        "--role",
        action="append",
        dest="roles",
        help=(
            "Role to calibrate. May be repeated. "
            f"Default: {', '.join(DEFAULT_ROLES)}."
        ),
    )
    parser.add_argument(
        "--force-wrist",
        action="store_true",
        help="Pass --force-wrist to each hand calibration process.",
    )
    parser.add_argument(
        "--reboot",
        action="store_true",
        help=(
            "Before calibration, reboot all configured motors for every selected "
            "Dynamixel role and verify Hardware Error Status(70) is clear."
        ),
    )
    parser.add_argument(
        "--stagger-sec",
        type=float,
        default=1.0,
        help="Delay between launching each role. Default: 1.0.",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=None,
        help="Directory for per-role logs. Default: ~/orca_hand_calibration_logs/<timestamp>.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve roles and print commands without connecting to hardware.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the interactive confirmation prompt.",
    )
    parser.add_argument(
        "--verbose-child-logs",
        action="store_true",
        help="Print every child process log line to the terminal. Full logs are always written to files.",
    )
    args = parser.parse_args()

    if args.stagger_sec < 0:
        print("ERROR: --stagger-sec must be >= 0.", file=sys.stderr)
        return 2

    role_names = args.roles or list(DEFAULT_ROLES)
    try:
        specs = _validate_roles(role_names)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print("Resolved hand roles:")
    for spec in specs:
        print_role_summary(spec)

    commands = {
        spec.role: _build_child_command(spec.role, args.force_wrist)
        for spec in specs
    }

    print("Calibration commands:")
    for role, command in commands.items():
        print(f"  {role}: {' '.join(command)}")
    print(
        "Note: Dynamixel hands currently try only the configured baud. "
        "Fallback baud probing is disabled in DynamixelClient.connect()."
    )
    if args.reboot:
        print("Dynamixel reboot preflight: enabled.")

    if args.dry_run:
        return 0

    if not args.yes:
        action = "reboot Dynamixel motors and calibrate" if args.reboot else "calibrate"
        confirmation = input(
            f"This will {action} all listed hands. "
            "Type CALIBRATE ALL to continue: "
        )
        if confirmation != "CALIBRATE ALL":
            print("Aborted.")
            return 130

    if args.reboot and not _reboot_dynamixel_roles(specs):
        print("ERROR: Dynamixel reboot preflight failed. Aborting calibration.", file=sys.stderr)
        return 1

    log_dir = args.log_dir or _default_log_dir()
    log_dir.mkdir(parents=True, exist_ok=True)
    print(f"Writing logs to: {log_dir}")
    if args.verbose_child_logs:
        print("Console child logs: verbose/raw")
    else:
        print(
            "Console child logs: filtered. Full unfiltered child logs are still written "
            "to the per-role log files."
        )

    processes: dict[str, subprocess.Popen[str]] = {}
    log_files = {}
    threads: list[threading.Thread] = []

    interrupted = False
    previous_sigint = signal.getsignal(signal.SIGINT)

    def _handle_sigint(signum, frame):
        nonlocal interrupted
        interrupted = True
        print("\nReceived interrupt. Stopping calibration processes...", flush=True)
        _terminate_processes(processes)

    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        env = os.environ.copy()
        for idx, spec in enumerate(specs):
            if idx and args.stagger_sec:
                time.sleep(args.stagger_sec)

            log_path = log_dir / f"{spec.role}.log"
            log_file = open(log_path, "w", encoding="utf-8")
            log_files[spec.role] = log_file
            process = subprocess.Popen(
                commands[spec.role],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env,
            )
            processes[spec.role] = process
            thread = threading.Thread(
                target=_stream_child_output,
                args=(spec.role, process, log_file, args.verbose_child_logs),
                daemon=True,
            )
            thread.start()
            threads.append(thread)
            print(f"Started {spec.role} as PID {process.pid}", flush=True)

        exit_codes = {role: process.wait() for role, process in processes.items()}
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        for thread in threads:
            thread.join(timeout=1.0)
        for log_file in log_files.values():
            log_file.close()

    print("Calibration exit codes:")
    for role, returncode in exit_codes.items():
        print(f"  {role}: {returncode}")

    if interrupted:
        return 130
    return 0 if all(returncode == 0 for returncode in exit_codes.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
