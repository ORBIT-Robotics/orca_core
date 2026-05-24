#!/usr/bin/env python3
"""Move all selected HELIOS ORCA hand roles to neutral."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    from orca_core.scripts._role_cli import print_role_summary, resolve_role
except ModuleNotFoundError:
    REPO_ROOT = Path(__file__).resolve().parents[3]
    ORCA_CORE_ROOT = REPO_ROOT / "orca_core"
    if str(ORCA_CORE_ROOT) not in sys.path:
        sys.path.insert(0, str(ORCA_CORE_ROOT))
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from orca_core.scripts._role_cli import print_role_summary, resolve_role


DEFAULT_ROLES = (
    "helios_upper_left_feetech",
    "helios_upper_right_feetech",
    "helios_lower_left",
    "helios_lower_right",
)


def _default_log_dir() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return Path.home() / "orca_hand_neutral_logs" / timestamp


def _build_child_command(role: str, num_steps: int, step_size: float) -> list[str]:
    script_path = Path(__file__).resolve().with_name("hand_neutral.py")
    return [
        sys.executable,
        str(script_path),
        "--role",
        role,
        "--num-steps",
        str(num_steps),
        "--step-size",
        str(step_size),
    ]


def _stream_child_output(role: str, process: subprocess.Popen[str], log_file) -> None:
    assert process.stdout is not None
    for line in process.stdout:
        log_file.write(line)
        log_file.flush()
        print(f"[{role}] {line.rstrip()}", flush=True)


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
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass


def _wait_for_processes(
    processes: dict[str, subprocess.Popen[str]],
    start_times: dict[str, float],
    role_timeout_sec: float,
) -> tuple[dict[str, int], set[str]]:
    exit_codes: dict[str, int] = {}
    timed_out: set[str] = set()

    while len(exit_codes) < len(processes):
        now = time.monotonic()
        for role, process in processes.items():
            if role in exit_codes:
                continue

            returncode = process.poll()
            if returncode is not None:
                exit_codes[role] = returncode
                continue

            if now - start_times[role] >= role_timeout_sec:
                print(
                    f"[{role}] ERROR: neutral timed out after "
                    f"{role_timeout_sec:.1f}s; stopping process.",
                    file=sys.stderr,
                    flush=True,
                )
                timed_out.add(role)
                _interrupt_processes({role: process})
                returncode = process.poll()
                exit_codes[role] = returncode if returncode is not None else 124

        if len(exit_codes) < len(processes):
            time.sleep(0.1)

    return exit_codes, timed_out


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


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Move all selected HELIOS ORCA hand roles to neutral."
    )
    parser.add_argument(
        "--role",
        action="append",
        dest="roles",
        help=(
            "Role to move to neutral. May be repeated. "
            f"Default: {', '.join(DEFAULT_ROLES)}."
        ),
    )
    parser.add_argument(
        "--num-steps",
        type=int,
        default=25,
        help="Number of interpolation steps for each neutral move. Default: 25.",
    )
    parser.add_argument(
        "--step-size",
        type=float,
        default=0.001,
        help="Seconds between interpolation steps. Default: 0.001.",
    )
    parser.add_argument(
        "--stagger-sec",
        type=float,
        default=0.5,
        help="Delay between launching each role. Default: 0.5.",
    )
    parser.add_argument(
        "--role-timeout-sec",
        type=float,
        default=60.0,
        help="Maximum seconds to wait for each role after it starts. Default: 60.",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=None,
        help="Directory for per-role logs. Default: ~/orca_hand_neutral_logs/<timestamp>.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve roles and print commands without connecting to hardware.",
    )
    args = parser.parse_args(argv)

    if args.num_steps < 1:
        print("ERROR: --num-steps must be >= 1", file=sys.stderr)
        return 2
    if args.step_size < 0:
        print("ERROR: --step-size must be >= 0", file=sys.stderr)
        return 2
    if args.stagger_sec < 0:
        print("ERROR: --stagger-sec must be >= 0", file=sys.stderr)
        return 2
    if args.role_timeout_sec <= 0:
        print("ERROR: --role-timeout-sec must be > 0", file=sys.stderr)
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
        spec.role: _build_child_command(spec.role, args.num_steps, args.step_size)
        for spec in specs
    }

    print("Neutral commands:")
    for role, command in commands.items():
        print(f"  {role}: {' '.join(command)}")

    if args.dry_run:
        return 0

    log_dir = args.log_dir or _default_log_dir()
    log_dir.mkdir(parents=True, exist_ok=True)
    print(f"Writing logs to: {log_dir}")

    processes: dict[str, subprocess.Popen[str]] = {}
    start_times: dict[str, float] = {}
    log_files = {}
    threads: list[threading.Thread] = []
    interrupted = False
    timed_out: set[str] = set()
    previous_sigint = signal.getsignal(signal.SIGINT)

    def _handle_sigint(signum, frame):
        nonlocal interrupted
        interrupted = True
        print("\nReceived interrupt. Stopping neutral processes...", flush=True)
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
            start_times[spec.role] = time.monotonic()
            thread = threading.Thread(
                target=_stream_child_output,
                args=(spec.role, process, log_file),
                daemon=True,
            )
            thread.start()
            threads.append(thread)
            print(f"Started {spec.role} as PID {process.pid}", flush=True)

        exit_codes, timed_out = _wait_for_processes(
            processes,
            start_times,
            args.role_timeout_sec,
        )
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        for thread in threads:
            thread.join(timeout=1.0)
        for log_file in log_files.values():
            log_file.close()

    print("Neutral exit codes:")
    for role, returncode in exit_codes.items():
        print(f"  {role}: {returncode}")

    if interrupted:
        return 130
    if timed_out:
        return 1
    return 0 if all(returncode == 0 for returncode in exit_codes.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
