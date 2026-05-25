#!/usr/bin/env python3
"""Run HELIOS head tension/hold until interrupted."""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import sys
import threading

try:
    from helios_core.head_calibration import HeliosHeadCalibrator
    from helios_core.utils.head_config import (
        DEFAULT_HEAD_ROLE,
        load_head_config,
        parse_disabled_motor_axes,
    )
except ModuleNotFoundError:
    REPO_ROOT = Path(__file__).resolve().parents[3]
    ORCA_CORE_ROOT = REPO_ROOT / "orca_core"
    if str(ORCA_CORE_ROOT) not in sys.path:
        sys.path.insert(0, str(ORCA_CORE_ROOT))
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from helios_core.head_calibration import HeliosHeadCalibrator
    from helios_core.utils.head_config import (
        DEFAULT_HEAD_ROLE,
        load_head_config,
        parse_disabled_motor_axes,
    )


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run HELIOS head tension/hold until interrupted.")
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
        help="Resolve config and print the hold plan without connecting to hardware.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    stop_event = threading.Event()

    def _request_stop(_signum, _frame):
        stop_event.set()

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    try:
        config, cfg_path = load_head_config(args.config, role=args.role)
        print("[helios_head] Tension hold plan:")
        print(f"  role={args.role}")
        print(f"  config={cfg_path}")
        print(f"  disabled_axes={parse_disabled_motor_axes(config)}")
        if args.dry_run:
            return 0

        cal = HeliosHeadCalibrator(config_path=args.config, role=args.role)
        try:
            cal.connect()
            cal.tension_assist()
            print("[helios_head] Tension hold active. Press Ctrl+C or stop from supervisor to release.")
            while not stop_event.wait(0.2):
                pass
            return 130
        finally:
            cal.disconnect()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    raise SystemExit(main())
