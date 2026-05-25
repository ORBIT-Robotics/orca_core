#!/usr/bin/env python3
"""Move the HELIOS head to calibrated neutral."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import numpy as np

try:
    from hardware.head_hardware import HeliosHeadHardware
    from helios_core.head_model import HeliosHeadCalibrationModel
    from helios_core.utils.head_config import (
        DEFAULT_HEAD_ROLE,
        calibration_output_path,
        load_head_config,
        parse_disabled_motor_axes,
        parse_motor_ids,
        read_yaml,
    )
except ModuleNotFoundError:
    REPO_ROOT = Path(__file__).resolve().parents[3]
    ORCA_CORE_ROOT = REPO_ROOT / "orca_core"
    if str(ORCA_CORE_ROOT) not in sys.path:
        sys.path.insert(0, str(ORCA_CORE_ROOT))
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from hardware.head_hardware import HeliosHeadHardware
    from helios_core.head_model import HeliosHeadCalibrationModel
    from helios_core.utils.head_config import (
        DEFAULT_HEAD_ROLE,
        calibration_output_path,
        load_head_config,
        parse_disabled_motor_axes,
        parse_motor_ids,
        read_yaml,
    )


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move the HELIOS head to calibrated neutral.")
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
        "--settle-sec",
        type=float,
        default=0.5,
        help="Seconds to hold neutral before disconnecting. Default: 0.5.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve config/calibration and print target without connecting to hardware.",
    )
    return parser.parse_args(argv)


def _fmt(values) -> str:
    arr = np.asarray(values, dtype=float).reshape(-1)
    return "[" + ", ".join(f"{float(value):.5f}" for value in arr) + "]"


def _load_model(config: dict, cfg_path: Path) -> HeliosHeadCalibrationModel:
    motor_ids = parse_motor_ids(config)
    calib_path = calibration_output_path(config, cfg_path)
    calib_data = read_yaml(calib_path)
    if not calib_data:
        raise RuntimeError(f"HELIOS head calibration YAML missing or empty: {calib_path}")
    return HeliosHeadCalibrationModel.from_yaml_dict(calib_data, motor_ids=motor_ids)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.settle_sec < 0.0:
        print("ERROR: --settle-sec must be >= 0.", file=sys.stderr)
        return 2

    try:
        config, cfg_path = load_head_config(args.config, role=args.role)
        model = _load_model(config, cfg_path)
        target = model.virtual_to_motor_targets(np.zeros(3, dtype=float))
        disabled_axes = parse_disabled_motor_axes(config)

        print("[helios_head] Neutral plan:")
        print(f"  role={args.role}")
        print(f"  config={cfg_path}")
        print(f"  disabled_axes={disabled_axes}")
        print(f"  neutral_motor_rad={_fmt(target)}")

        if args.dry_run:
            return 0

        hardware = HeliosHeadHardware(config, parse_motor_ids(config))
        hardware.set_disabled_motor_positions(model.neutral_motors)
        try:
            hardware.connect()
            runtime_current = float(
                (config.get("hardware") or {}).get(
                    "runtime_current_limit_ma",
                    (config.get("hardware") or {}).get("startup_current_limit_ma", 260.0),
                )
            )
            hardware.set_current_limit(runtime_current)
            commanded = hardware.command_motor_positions(
                target,
                limits=model.motor_limits_for_hardware(),
                max_step_rad=0.0,
            )
            print(f"[helios_head] Commanded neutral motor target: {_fmt(commanded)}")
            time.sleep(args.settle_sec)
            state = hardware.read_state()
            print(f"[helios_head] Final motor position: {_fmt(state.positions_rad)}")
        finally:
            hardware.disconnect()
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
