#!/usr/bin/env python3
"""Quick HELIOS head motor and endpoint-calibration availability check."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

orca_core_root = Path(__file__).resolve().parents[2]
if str(orca_core_root) not in sys.path:
    sys.path.insert(0, str(orca_core_root))

from helios_core.utils.head_config import (  # noqa: E402
    DEFAULT_HEAD_ROLE,
    calibration_output_path,
    load_head_config,
    parse_motor_ids,
    read_yaml,
)
from helios_core.head_model import HeliosHeadCalibrationModel  # noqa: E402
from hardware.head_hardware import HeliosHeadHardware  # noqa: E402


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HELIOS head motor/calibration availability check")
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
    return parser.parse_args()


def _fmt_vec(values: np.ndarray, precision: int = 4) -> str:
    return "[" + ", ".join(f"{float(v):.{precision}f}" for v in values) + "]"


def main() -> int:
    args = _parse_args()
    config, cfg_path = load_head_config(args.config, role=args.role)
    motor_ids = parse_motor_ids(config)

    hardware = HeliosHeadHardware(config, motor_ids)
    disabled_axes = tuple(getattr(hardware, "disabled_motor_axes", ()))
    active_ids = tuple(
        motor_ids.as_dict[axis]
        for axis in ("yaw", "pitch", "roll")
        if axis not in disabled_axes
    )
    checks: list[tuple[str, bool, str]] = []

    try:
        try:
            hardware.connect()
            state = hardware.read_state()
            checks.append(
                (
                    "motors",
                    True,
                    (
                        f"connected on {hardware.port} @ {hardware.baudrate} with IDs {motor_ids.ordered}; "
                        f"active_ids={active_ids} disabled_axes={disabled_axes}; "
                        f"pos_rad={_fmt_vec(state.positions_rad)} "
                        f"vel_rad_s={_fmt_vec(state.velocities_rad_s)} "
                        f"cur_ma={_fmt_vec(state.currents_ma, precision=1)}"
                    ),
                )
            )
        except Exception as exc:
            checks.append(("motors", False, str(exc)))

        try:
            calib_path = calibration_output_path(config, cfg_path)
            calib_data = read_yaml(calib_path)
            model = HeliosHeadCalibrationModel.from_yaml_dict(calib_data, motor_ids=motor_ids)
            neutral_target = model.virtual_to_motor_targets(np.zeros(3, dtype=float))
            checks.append(
                (
                    "endpoint_calibration",
                    True,
                    (
                        f"loaded {calib_path}; "
                        f"neutral_motor_rad={_fmt_vec(neutral_target)} "
                        f"ratios={model.joint_to_motor_ratios}"
                    ),
                )
            )
        except Exception as exc:
            checks.append(("endpoint_calibration", False, str(exc)))
    finally:
        try:
            hardware.disconnect()
        except Exception:
            pass

    overall_ok = all(ok for _, ok, _ in checks)
    print("[helios_head_check] Summary")
    for name, ok, detail in checks:
        status = "OK" if ok else "FAIL"
        print(f" - {name}: {status} - {detail}")
    print(f"[helios_head_check] overall={'OK' if overall_ok else 'FAIL'}")
    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
