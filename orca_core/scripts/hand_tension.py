"""CLI for ORCA hand tensioning."""

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
    from orca_core.scripts._role_cli import (
        create_hand,
        print_role_summary,
        resolve_role,
    )
except ModuleNotFoundError:
    REPO_ROOT = Path(__file__).resolve().parents[3]
    ORCA_CORE_ROOT = REPO_ROOT / "orca_core"
    if str(ORCA_CORE_ROOT) not in sys.path:
        sys.path.insert(0, str(ORCA_CORE_ROOT))
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from orca_core.scripts._role_cli import create_hand, print_role_summary, resolve_role


DEFAULT_ROLES = (
    "helios_upper_left_feetech",
    "helios_upper_right_feetech",
    "helios_lower_left",
    "helios_lower_right",
)


def _default_log_dir() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return Path.home() / "orca_hand_tension_logs" / timestamp


def _build_child_command(role: str, move_motors: bool) -> list[str]:
    script_path = Path(__file__).resolve()
    command = [sys.executable, str(script_path), "--role", role]
    if move_motors:
        command.append("--move_motors")
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


def _interrupt_processes(processes: dict[str, subprocess.Popen[str]]) -> None:
    for process in processes.values():
        if process.poll() is None:
            process.send_signal(signal.SIGINT)

    deadline = time.monotonic() + 5.0
    for process in processes.values():
        if process.poll() is not None:
            continue
        remaining = max(0.0, deadline - time.monotonic())
        try:
            process.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            process.terminate()

    deadline = time.monotonic() + 2.0
    for process in processes.values():
        if process.poll() is not None:
            continue
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


def _run_single_role(spec, move_motors: bool) -> int:
    print_role_summary(spec)
    hand = create_hand(spec)
    success, message = hand.connect()
    print((success, message))
    if not success:
        return 1

    try:
        hand.enable_torque()
        hand.tension(move_motors)
        return 0
    except KeyboardInterrupt:
        print("\nTension interrupted.")
        return 130
    finally:
        hand.disconnect()


def _run_multi_role(args: argparse.Namespace, specs) -> int:
    print("Resolved hand roles:")
    for spec in specs:
        print_role_summary(spec)

    commands = {
        spec.role: _build_child_command(spec.role, args.move_motors)
        for spec in specs
    }

    print("Tension commands:")
    for role, command in commands.items():
        print(f"  {role}: {' '.join(command)}")

    if args.dry_run:
        return 0

    if not args.yes:
        confirmation = input(
            "This will lock all listed hands for manual tensioning. "
            "Type TENSION ALL to continue: "
        )
        if confirmation != "TENSION ALL":
            print("Aborted.")
            return 130

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
    exit_codes: dict[str, int] = {}

    interrupted = False
    previous_sigint = signal.getsignal(signal.SIGINT)

    def _handle_sigint(signum, frame):
        nonlocal interrupted
        interrupted = True
        print("\nReceived interrupt. Releasing tension processes...", flush=True)
        _interrupt_processes(processes)

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

        while len(exit_codes) < len(processes):
            for role, process in processes.items():
                if role in exit_codes:
                    continue
                returncode = process.poll()
                if returncode is None:
                    continue
                exit_codes[role] = returncode
                if returncode != 0 and not interrupted:
                    print(
                        f"{role} exited with {returncode}; other hands remain active. "
                        "Press Ctrl+C when you want to release them.",
                        file=sys.stderr,
                        flush=True,
                    )
            if len(exit_codes) < len(processes):
                time.sleep(0.1)
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        for role, process in processes.items():
            if role not in exit_codes and process.poll() is not None:
                exit_codes[role] = process.returncode
        for thread in threads:
            thread.join(timeout=1.0)
        for log_file in log_files.values():
            log_file.close()

    print("Tension exit codes:")
    for role, returncode in exit_codes.items():
        print(f"  {role}: {returncode}")

    if interrupted:
        return 130
    return 0 if all(returncode == 0 for returncode in exit_codes.values()) else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Run ORCA hand tension routine.")
    parser.add_argument(
        "--role",
        action="append",
        dest="roles",
        help=(
            "ORCA hand role from configs/<robot>.yaml. May be repeated. "
            f"Omit to tension all default HELIOS roles: {', '.join(DEFAULT_ROLES)}."
        ),
    )
    parser.add_argument(
        "--move_motors",
        action="store_true",
        help="Move motors during tension setup.",
    )
    parser.add_argument(
        "--stagger-sec",
        type=float,
        default=0.0,
        help="Delay between launching each role in multi-role mode. Default: 0.0.",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=None,
        help="Directory for per-role logs in multi-role mode. Default: ~/orca_hand_tension_logs/<timestamp>.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve roles and print commands without connecting to hardware.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the multi-role confirmation prompt.",
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

    if len(specs) == 1 and args.roles:
        if args.dry_run:
            print("Resolved hand role:")
            print_role_summary(specs[0])
            print("Tension command:")
            print(f"  {' '.join(_build_child_command(specs[0].role, args.move_motors))}")
            return 0
        return _run_single_role(specs[0], args.move_motors)

    return _run_multi_role(args, specs)


if __name__ == "__main__":
    raise SystemExit(main())
