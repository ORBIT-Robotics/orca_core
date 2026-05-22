#!/usr/bin/env python3
"""Calibrate all HELIOS ORCA hands sequentially by default."""

from __future__ import annotations

import argparse
import os
from collections import Counter
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    from orca_core.scripts._bus_preflight import preflight_motor_role, preflight_motor_roles
    from orca_core.scripts._dynamixel_preflight import (
        _reboot_dynamixel_roles,
        _role_motor_ids,
        _role_motor_type,
    )
    from orca_core.scripts._role_cli import print_role_summary, resolve_role
except ModuleNotFoundError:
    REPO_ROOT = Path(__file__).resolve().parents[3]
    ORCA_CORE_ROOT = REPO_ROOT / "orca_core"
    if str(ORCA_CORE_ROOT) not in sys.path:
        sys.path.insert(0, str(ORCA_CORE_ROOT))
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from orca_core.scripts._bus_preflight import preflight_motor_role, preflight_motor_roles
    from orca_core.scripts._dynamixel_preflight import (
        _reboot_dynamixel_roles,
        _role_motor_ids,
        _role_motor_type,
    )
    from orca_core.scripts._role_cli import print_role_summary, resolve_role


DEFAULT_ROLES = (
    "helios_lower_left",
    "helios_lower_right",
    "helios_upper_left_feetech",
    "helios_upper_right_feetech",
)
TIMEOUT_EXIT_CODE = 124


def _default_log_dir() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return Path.home() / "orca_hand_calibration_logs" / timestamp


def _build_child_command(role: str, force_wrist: bool) -> list[str]:
    script_path = Path(__file__).resolve().with_name("hand_calibrate.py")
    command = [sys.executable, str(script_path), "--role", role]
    if force_wrist:
        command.append("--force-wrist")
    return command


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


def _terminate_child(role: str, process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    print(f"[{role}] terminating calibration process pid={process.pid}", flush=True)
    process.terminate()
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        print(f"[{role}] killing calibration process pid={process.pid}", flush=True)
        process.kill()
        process.wait(timeout=5.0)


def _terminate_processes(processes: dict[str, subprocess.Popen[str]]) -> None:
    for role, process in processes.items():
        _terminate_child(role, process)


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


def _run_child_process(
    spec,
    command: list[str],
    log_dir: Path,
    verbose_console: bool,
    timeout_sec: float,
    env: dict[str, str],
) -> int:
    log_path = log_dir / f"{spec.role}.log"
    with open(log_path, "w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )
        thread = threading.Thread(
            target=_stream_child_output,
            args=(spec.role, process, log_file, verbose_console),
            daemon=True,
        )
        thread.start()
        print(f"Started {spec.role} as PID {process.pid}; log={log_path}", flush=True)
        try:
            return process.wait(timeout=timeout_sec if timeout_sec > 0 else None)
        except subprocess.TimeoutExpired:
            print(
                f"[{spec.role}] ERROR: calibration timed out after {timeout_sec:.1f}s.",
                file=sys.stderr,
                flush=True,
            )
            _terminate_child(spec.role, process)
            return TIMEOUT_EXIT_CODE
        except KeyboardInterrupt:
            _terminate_child(spec.role, process)
            raise
        finally:
            thread.join(timeout=2.0)


def _run_parallel(
    specs,
    commands: dict[str, list[str]],
    log_dir: Path,
    verbose_console: bool,
    timeout_sec: float,
    stagger_sec: float,
    env: dict[str, str],
) -> tuple[dict[str, int], bool]:
    processes: dict[str, subprocess.Popen[str]] = {}
    log_files = {}
    threads: list[threading.Thread] = []
    start_times: dict[str, float] = {}
    exit_codes: dict[str, int] = {}
    interrupted = False

    try:
        for idx, spec in enumerate(specs):
            if idx and stagger_sec:
                time.sleep(stagger_sec)
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
            start_times[spec.role] = time.monotonic()
            thread = threading.Thread(
                target=_stream_child_output,
                args=(spec.role, process, log_file, verbose_console),
                daemon=True,
            )
            thread.start()
            threads.append(thread)
            print(f"Started {spec.role} as PID {process.pid}; log={log_path}", flush=True)

        while len(exit_codes) < len(processes):
            now = time.monotonic()
            for role, process in processes.items():
                if role in exit_codes:
                    continue
                returncode = process.poll()
                if returncode is not None:
                    exit_codes[role] = int(returncode)
                    continue
                if timeout_sec > 0 and now - start_times[role] >= timeout_sec:
                    print(
                        f"[{role}] ERROR: calibration timed out after {timeout_sec:.1f}s.",
                        file=sys.stderr,
                        flush=True,
                    )
                    _terminate_child(role, process)
                    exit_codes[role] = TIMEOUT_EXIT_CODE
            if len(exit_codes) < len(processes):
                time.sleep(0.2)
    except KeyboardInterrupt:
        interrupted = True
        print("\nReceived interrupt. Stopping calibration processes...", flush=True)
        _terminate_processes(processes)
        for role, process in processes.items():
            exit_codes.setdefault(role, process.poll() if process.poll() is not None else 130)
    finally:
        for thread in threads:
            thread.join(timeout=2.0)
        for log_file in log_files.values():
            log_file.close()

    return exit_codes, interrupted


def _print_exit_codes(exit_codes: dict[str, int]) -> None:
    print("Calibration exit codes:")
    for role, returncode in exit_codes.items():
        print(f"  {role}: {returncode}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Calibrate all HELIOS ORCA hand roles sequentially by default."
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
        "--parallel",
        action="store_true",
        help="Launch all role calibrations concurrently after a shared bus preflight.",
    )
    parser.add_argument(
        "--continue-on-failure",
        action="store_true",
        help="Continue with remaining roles after one role preflight or calibration fails.",
    )
    parser.add_argument(
        "--role-timeout-sec",
        type=float,
        default=600.0,
        help="Maximum seconds to allow each role calibration child process. Default: 600.",
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
        "--verbose-child-logs",
        action="store_true",
        help="Print every child process log line to the terminal. Full logs are always written to files.",
    )
    args = parser.parse_args(argv)

    if args.stagger_sec < 0:
        print("ERROR: --stagger-sec must be >= 0.", file=sys.stderr)
        return 2
    if args.role_timeout_sec < 0:
        print("ERROR: --role-timeout-sec must be >= 0.", file=sys.stderr)
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
        "Mode: parallel" if args.parallel else "Mode: sequential; one hand role owns its serial bus at a time."
    )
    print(
        "Note: Dynamixel hands currently try only the configured baud. "
        "Fallback baud probing is disabled in DynamixelClient.connect()."
    )
    if args.reboot:
        print("Dynamixel reboot preflight: enabled.")

    if args.dry_run:
        return 0

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

    env = os.environ.copy()
    if args.parallel:
        if not preflight_motor_roles(specs):
            print("ERROR: ORCA hand bus preflight failed. Aborting calibration.", file=sys.stderr)
            return 1
        exit_codes, interrupted = _run_parallel(
            specs,
            commands,
            log_dir,
            args.verbose_child_logs,
            args.role_timeout_sec,
            args.stagger_sec,
            env,
        )
        _print_exit_codes(exit_codes)
        if interrupted:
            return 130
        return 0 if all(returncode == 0 for returncode in exit_codes.values()) else 1

    exit_codes: dict[str, int] = {}
    interrupted = False
    try:
        for idx, spec in enumerate(specs):
            if idx and args.stagger_sec:
                time.sleep(args.stagger_sec)
            print(f"[{spec.role}] Running ORCA hand bus preflight before calibration.")
            if not preflight_motor_role(spec):
                print(
                    f"[{spec.role}] ERROR: bus preflight failed; calibration child not started.",
                    file=sys.stderr,
                )
                exit_codes[spec.role] = 1
                if not args.continue_on_failure:
                    break
                continue

            returncode = _run_child_process(
                spec,
                commands[spec.role],
                log_dir,
                args.verbose_child_logs,
                args.role_timeout_sec,
                env,
            )
            exit_codes[spec.role] = returncode
            if returncode != 0 and not args.continue_on_failure:
                break
    except KeyboardInterrupt:
        interrupted = True
        print("\nReceived interrupt. Calibration stopped.", flush=True)

    _print_exit_codes(exit_codes)
    if interrupted:
        return 130
    return 0 if exit_codes and all(returncode == 0 for returncode in exit_codes.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
