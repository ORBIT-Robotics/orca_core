#!/usr/bin/env python3
"""CLI for HELIOS head endpoint calibration."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

orca_core_root = Path(__file__).resolve().parents[2]
if str(orca_core_root) not in sys.path:
    sys.path.insert(0, str(orca_core_root))

from helios_core.head_calibration import HeliosHeadCalibrator
from helios_core.utils.head_config import DEFAULT_HEAD_ROLE, read_yaml, write_yaml, utc_now_iso


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HELIOS head endpoint calibration")
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
        "--stage",
        choices=["full", "tension", "limits", "neutral", "save"],
        default="full",
        help="Calibration stage to run",
    )
    return parser.parse_args()


def _pause(msg: str) -> None:
    input(f"{msg} Press Enter to continue...")


def _hold_until_interrupt(msg: str) -> None:
    try:
        input(f"{msg} Press Enter when finished or Ctrl+C to abort...")
    except KeyboardInterrupt:
        print("\n[helios_head] Exiting tension hold.")


def _save_partial(cal: HeliosHeadCalibrator) -> None:
    """Persist discovered limits/neutral before endpoint ratios are complete."""
    current = read_yaml(cal.calibration_path)
    payload = dict(current or {})
    payload["version"] = 2
    payload["updated_at"] = utc_now_iso()
    payload["calibrated"] = False
    payload["hardware"] = {"motor_ids": cal.motor_ids.as_dict}

    if cal.motor_limits:
        payload["motor_limits"] = {
            axis: [float(v[0]), float(v[1])] for axis, v in cal.motor_limits.items()
        }

    if cal.neutral_motors is not None:
        payload["neutral"] = {
            "motors": {
                axis: float(cal.neutral_motors[axis])
                for axis in ("yaw", "pitch", "roll")
            }
        }
    else:
        payload.pop("neutral", None)

    for old_key in (
        "gains",
        "joint_to_motor_ratios",
        "model",
        "signs",
        "validation",
        "virtual_limits_rad",
    ):
        payload.pop(old_key, None)

    write_yaml(cal.calibration_path, payload)
    print(f"[helios_head] Saved partial calibration data: {cal.calibration_path}")


def _load_partial_into_calibrator(cal: HeliosHeadCalibrator) -> None:
    data = read_yaml(cal.calibration_path)
    motor_limits = data.get("motor_limits", {})
    axis_order = ("yaw", "pitch", "roll")
    legacy_aliases = {"pitch": "upper_left", "roll": "upper_right"}

    def _axis_value(mapping: dict, axis: str):
        if axis in mapping:
            return mapping[axis]
        legacy_axis = legacy_aliases.get(axis)
        if legacy_axis is not None and legacy_axis in mapping:
            return mapping[legacy_axis]
        raise KeyError(axis)

    if all(axis in motor_limits or legacy_aliases.get(axis) in motor_limits for axis in axis_order):
        cal.motor_limits = {
            axis: (
                float(_axis_value(motor_limits, axis)[0]),
                float(_axis_value(motor_limits, axis)[1]),
            )
            for axis in axis_order
        }

    neutral = (data.get("neutral") or {}).get("motors", {})
    neutral_axes = axis_order
    if isinstance(neutral, dict) and all(axis in neutral or legacy_aliases.get(axis) in neutral for axis in neutral_axes):
        cal.neutral_motors = {
            axis: float(_axis_value(neutral, axis))
            for axis in neutral_axes
        }


def main() -> None:
    args = _parse_args()

    cal = HeliosHeadCalibrator(config_path=args.config, role=args.role)
    try:
        cal.connect()

        if args.stage == "tension":
            cal.tension_assist()
            _save_partial(cal)
            _hold_until_interrupt(
                "Head is holding current motor positions for manual inspection.",
            )
            return

        if args.stage == "limits":
            _pause(
                "Make sure the head is mechanically safe, centered, and away from end stops.",
            )
            cal.find_motor_limits()
            _save_partial(cal)
            return

        if args.stage == "neutral":
            _load_partial_into_calibrator(cal)
            cal.release_for_manual_neutral()
            _pause("Position the head at neutral.")
            cal.prepare_capture_neutral()
            cal.capture_neutral()
            _save_partial(cal)
            return

        if args.stage == "save":
            _load_partial_into_calibrator(cal)
            if not cal.motor_limits:
                raise RuntimeError("Missing motor limits in calibration YAML. Run --stage limits first.")
            if cal.neutral_motors is None:
                raise RuntimeError("Missing neutral in calibration YAML. Run --stage neutral first.")
            cal.save_results()
            return

        _pause(
            "Step 1/4: Ensure power is on, communication is healthy, and the head mount is mechanically safe.",
        )
        cal.tension_assist()

        _pause(
            "Step 2/4: Confirm the mount is mechanically safe and the head is centered.",
        )
        cal.find_motor_limits()

        cal.release_for_manual_neutral()
        _pause(
            "Step 3/4: Adjust the head freely to neutral, then continue to re-enable hold and capture.",
        )
        cal.prepare_capture_neutral()
        cal.capture_neutral()

        _pause("Step 4/4: Saving endpoint calibration.")
        cal.save_results()

    finally:
        cal.disconnect()


if __name__ == "__main__":
    main()
