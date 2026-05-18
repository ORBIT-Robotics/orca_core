"""HELIOS head endpoint mapping between virtual [yaw, pitch, roll] and motors."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from helios_core.utils.head_config import HeadMotorIDs


MOTOR_AXES = ("yaw", "pitch", "roll")
VIRTUAL_AXES = ("yaw", "pitch", "roll")
LEGACY_MOTOR_AXIS_ALIASES = {
    "pitch": "upper_left",
    "roll": "upper_right",
}


def _as_float(value, name: str) -> float:
    if value is None:
        raise ValueError(f"Missing calibration value for {name}")
    value_f = float(value)
    if not np.isfinite(value_f):
        raise ValueError(f"Non-finite calibration value for {name}: {value}")
    return value_f


def _parse_positive_float(value, name: str) -> float:
    value_f = _as_float(value, name)
    if value_f <= 0.0:
        raise ValueError(f"Expected positive calibration value for {name}, got {value_f}")
    return value_f


def _get_axis_value(mapping: Dict, axis: str, label: str):
    value = mapping.get(axis)
    if value is None:
        legacy_axis = LEGACY_MOTOR_AXIS_ALIASES.get(axis)
        if legacy_axis is not None:
            value = mapping.get(legacy_axis)
    if value is None:
        raise ValueError(f"Missing calibration value for {label}.{axis}")
    return value


def _validate_motor_limits(motor_limits: Dict[str, tuple[float, float]]) -> None:
    for axis in MOTOR_AXES:
        lo, hi = motor_limits[axis]
        if lo >= hi:
            raise ValueError(f"Expected min < max for motor_limits.{axis}, got {(lo, hi)}")


def _validate_neutral_inside_limits(
    motor_limits: Dict[str, tuple[float, float]],
    neutral_motors: Dict[str, float],
) -> None:
    violations = []
    for axis in MOTOR_AXES:
        lo, hi = motor_limits[axis]
        value = neutral_motors[axis]
        if value < lo or value > hi:
            violations.append(f"{axis}={value:.5f} outside [{lo:.5f}, {hi:.5f}]")
    if violations:
        raise ValueError("Neutral motors must be inside motor limits: " + "; ".join(violations))


def derive_endpoint_joint_to_motor_ratios(
    motor_limits: Dict[str, tuple[float, float]],
    neutral_motors: Dict[str, float],
    virtual_limits_rad: Dict[str, float],
    *,
    min_ratio: float = 1e-6,
) -> Dict[str, float]:
    """Derive direct motor-per-virtual-radian ratios from endpoint margins."""

    _validate_motor_limits(motor_limits)
    _validate_neutral_inside_limits(motor_limits, neutral_motors)

    limits = {
        axis: _parse_positive_float(virtual_limits_rad.get(axis), f"virtual_limits_rad.{axis}")
        for axis in VIRTUAL_AXES
    }

    ratios = {}
    for axis in VIRTUAL_AXES:
        lo, hi = motor_limits[axis]
        neutral = neutral_motors[axis]
        margin = min(neutral - lo, hi - neutral)
        ratios[axis] = margin / limits[axis]

    min_ratio_f = max(0.0, float(min_ratio))
    for axis, value in ratios.items():
        if not np.isfinite(value) or value <= min_ratio_f:
            raise ValueError(
                f"Degenerate endpoint ratio for {axis}: {value}. "
                "Check motor limits, neutral pose, and virtual limits."
            )
    return {axis: float(value) for axis, value in ratios.items()}


@dataclass
class HeliosHeadCalibrationModel:
    motor_ids: HeadMotorIDs
    motor_limits: Dict[str, tuple[float, float]]
    neutral_motors: Dict[str, float]
    joint_to_motor_ratios: Dict[str, float]
    signs: Dict[str, float]
    virtual_limits_rad: Dict[str, float]

    @classmethod
    def from_yaml_dict(cls, data: Dict, motor_ids: Optional[HeadMotorIDs] = None):
        if int(data.get("version", 0)) < 2:
            raise ValueError(
                "HELIOS head calibration schema version 2 is required. "
                "Run orca_core/helios_core/scripts/helios_head_calibrate.py to regenerate endpoint calibration."
            )
        if data.get("calibrated") is not True:
            raise ValueError("HELIOS head calibration is not marked calibrated.")

        hw = dict(data.get("hardware", {}))
        mid = dict(hw.get("motor_ids", {}))
        if motor_ids is None:
            motor_ids = HeadMotorIDs(
                yaw=int(_get_axis_value(mid, "yaw", "hardware.motor_ids")),
                pitch=int(_get_axis_value(mid, "pitch", "hardware.motor_ids")),
                roll=int(_get_axis_value(mid, "roll", "hardware.motor_ids")),
            )

        raw_limits = dict(data.get("motor_limits", {}))
        motor_limits = {}
        for axis in MOTOR_AXES:
            lim = _get_axis_value(raw_limits, axis, "motor_limits")
            if lim is None or len(lim) != 2:
                raise ValueError(f"Invalid motor_limits.{axis}: {lim}")
            motor_limits[axis] = (
                _as_float(lim[0], f"motor_limits.{axis}[0]"),
                _as_float(lim[1], f"motor_limits.{axis}[1]"),
            )
        _validate_motor_limits(motor_limits)

        neutral = dict((data.get("neutral") or {}).get("motors", {}))
        neutral_motors = {
            axis: _as_float(
                _get_axis_value(neutral, axis, "neutral.motors"),
                f"neutral.motors.{axis}",
            )
            for axis in MOTOR_AXES
        }
        _validate_neutral_inside_limits(motor_limits, neutral_motors)

        raw_virtual_limits = dict(data.get("virtual_limits_rad", {}))
        virtual_limits = {
            "yaw": _parse_positive_float(raw_virtual_limits.get("yaw", 0.7), "virtual_limits_rad.yaw"),
            "pitch": _parse_positive_float(raw_virtual_limits.get("pitch", 0.55), "virtual_limits_rad.pitch"),
            "roll": _parse_positive_float(raw_virtual_limits.get("roll", 0.55), "virtual_limits_rad.roll"),
        }

        raw_ratios = dict(data.get("joint_to_motor_ratios", {}))
        ratios = {
            axis: _parse_positive_float(raw_ratios.get(axis), f"joint_to_motor_ratios.{axis}")
            for axis in VIRTUAL_AXES
        }

        raw_signs = dict(data.get("signs", {}))
        signs = {
            "yaw_sign": _as_float(raw_signs.get("yaw_sign", 1.0), "signs.yaw_sign"),
            "pitch_sign": _as_float(raw_signs.get("pitch_sign", 1.0), "signs.pitch_sign"),
            "roll_sign": _as_float(raw_signs.get("roll_sign", 1.0), "signs.roll_sign"),
        }
        for key, value in signs.items():
            if np.isclose(value, 0.0):
                raise ValueError(f"Sign {key} cannot be zero.")

        return cls(
            motor_ids=motor_ids,
            motor_limits=motor_limits,
            neutral_motors=neutral_motors,
            joint_to_motor_ratios=ratios,
            signs=signs,
            virtual_limits_rad=virtual_limits,
        )

    def clip_virtual(self, virtual_cmd: np.ndarray) -> np.ndarray:
        cmd = np.asarray(virtual_cmd, dtype=float).reshape(3,)
        clipped = cmd.copy()
        for idx, axis in enumerate(VIRTUAL_AXES):
            lim = abs(self.virtual_limits_rad[axis])
            clipped[idx] = float(np.clip(clipped[idx], -lim, lim))
        return clipped

    def clip_motor_targets(self, motor_targets: np.ndarray, margin_rad: float = 0.0) -> np.ndarray:
        values = np.asarray(motor_targets, dtype=float).reshape(3,)
        clipped = values.copy()
        margin = max(0.0, float(margin_rad))
        for idx, axis in enumerate(MOTOR_AXES):
            lo, hi = self.motor_limits[axis]
            if lo + margin > hi - margin:
                raise ValueError(f"Motor limit margin {margin} leaves no valid range for {axis}.")
            clipped[idx] = float(np.clip(clipped[idx], lo + margin, hi - margin))
        return clipped

    def virtual_to_motor_targets(self, virtual_cmd: np.ndarray) -> np.ndarray:
        cmd = self.clip_virtual(virtual_cmd)
        targets = np.zeros(3, dtype=float)
        for idx, axis in enumerate(VIRTUAL_AXES):
            sign = self.signs[f"{axis}_sign"]
            ratio = self.joint_to_motor_ratios[axis]
            targets[idx] = self.neutral_motors[axis] + ratio * sign * cmd[idx]
        return targets

    def motor_to_virtual(self, motor_positions: np.ndarray) -> np.ndarray:
        motor = np.asarray(motor_positions, dtype=float).reshape(3,)
        virtual = np.zeros(3, dtype=float)
        for idx, axis in enumerate(VIRTUAL_AXES):
            ratio = self.joint_to_motor_ratios[axis]
            sign = self.signs[f"{axis}_sign"]
            motor_delta = motor[idx] - self.neutral_motors[axis]
            virtual[idx] = motor_delta / ratio / sign
        return self.clip_virtual(virtual)

    def motor_limits_for_hardware(self) -> Dict[str, tuple[float, float]]:
        return {
            "yaw": tuple(self.motor_limits["yaw"]),
            "pitch": tuple(self.motor_limits["pitch"]),
            "roll": tuple(self.motor_limits["roll"]),
        }
