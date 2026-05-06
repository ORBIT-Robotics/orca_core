"""HELIOS head endpoint mapping between virtual [yaw,pitch,roll] and motors."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from helios_core.utils.head_config import HeadMotorIDs


MOTOR_AXES = ("yaw", "upper_left", "upper_right")
VIRTUAL_AXES = ("yaw", "pitch", "roll")


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
    """Derive symmetric motor-per-virtual-radian ratios from endpoint margins."""

    _validate_motor_limits(motor_limits)
    _validate_neutral_inside_limits(motor_limits, neutral_motors)

    limits = {
        axis: _parse_positive_float(virtual_limits_rad.get(axis), f"virtual_limits_rad.{axis}")
        for axis in VIRTUAL_AXES
    }

    yaw_lo, yaw_hi = motor_limits["yaw"]
    ul_lo, ul_hi = motor_limits["upper_left"]
    ur_lo, ur_hi = motor_limits["upper_right"]
    yaw_n = neutral_motors["yaw"]
    ul_n = neutral_motors["upper_left"]
    ur_n = neutral_motors["upper_right"]

    yaw_margin = min(yaw_n - yaw_lo, yaw_hi - yaw_n)
    upper_common_positive = min(ul_hi - ul_n, ur_hi - ur_n)
    upper_common_negative = min(ul_n - ul_lo, ur_n - ur_lo)
    upper_diff_positive = min(ul_hi - ul_n, ur_n - ur_lo)
    upper_diff_negative = min(ul_n - ul_lo, ur_hi - ur_n)

    ratios = {
        "yaw": yaw_margin / limits["yaw"],
        "pitch": min(upper_common_positive, upper_common_negative) / limits["pitch"],
        "roll": min(upper_diff_positive, upper_diff_negative) / limits["roll"],
    }

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
                yaw=int(mid.get("yaw")),
                upper_left=int(mid.get("upper_left")),
                upper_right=int(mid.get("upper_right")),
            )

        raw_limits = dict(data.get("motor_limits", {}))
        motor_limits = {}
        for axis in MOTOR_AXES:
            lim = raw_limits.get(axis)
            if lim is None or len(lim) != 2:
                raise ValueError(f"Invalid motor_limits.{axis}: {lim}")
            motor_limits[axis] = (
                _as_float(lim[0], f"motor_limits.{axis}[0]"),
                _as_float(lim[1], f"motor_limits.{axis}[1]"),
            )
        _validate_motor_limits(motor_limits)

        neutral = dict((data.get("neutral") or {}).get("motors", {}))
        neutral_motors = {
            axis: _as_float(neutral.get(axis), f"neutral.motors.{axis}")
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
        yaw, pitch, roll = self.clip_virtual(virtual_cmd)

        yaw_eff = self.signs["yaw_sign"] * yaw
        pitch_eff = self.signs["pitch_sign"] * pitch
        roll_eff = self.signs["roll_sign"] * roll

        yaw_ratio = self.joint_to_motor_ratios["yaw"]
        pitch_ratio = self.joint_to_motor_ratios["pitch"]
        roll_ratio = self.joint_to_motor_ratios["roll"]

        m_yaw = self.neutral_motors["yaw"] + yaw_ratio * yaw_eff
        m_ul = self.neutral_motors["upper_left"] + pitch_ratio * pitch_eff + roll_ratio * roll_eff
        m_ur = self.neutral_motors["upper_right"] + pitch_ratio * pitch_eff - roll_ratio * roll_eff

        return np.array([m_yaw, m_ul, m_ur], dtype=float)

    def motor_to_virtual(self, motor_positions: np.ndarray) -> np.ndarray:
        motor = np.asarray(motor_positions, dtype=float).reshape(3,)
        yaw_ratio = self.joint_to_motor_ratios["yaw"]
        pitch_ratio = self.joint_to_motor_ratios["pitch"]
        roll_ratio = self.joint_to_motor_ratios["roll"]

        dy = (motor[0] - self.neutral_motors["yaw"]) / yaw_ratio
        dp = (
            (motor[1] - self.neutral_motors["upper_left"])
            + (motor[2] - self.neutral_motors["upper_right"])
        ) / (2.0 * pitch_ratio)
        dr = (
            (motor[1] - self.neutral_motors["upper_left"])
            - (motor[2] - self.neutral_motors["upper_right"])
        ) / (2.0 * roll_ratio)

        yaw = dy / self.signs["yaw_sign"]
        pitch = dp / self.signs["pitch_sign"]
        roll = dr / self.signs["roll_sign"]
        return self.clip_virtual(np.array([yaw, pitch, roll], dtype=float))

    def motor_limits_for_hardware(self) -> Dict[str, tuple[float, float]]:
        return {
            "yaw": tuple(self.motor_limits["yaw"]),
            "upper_left": tuple(self.motor_limits["upper_left"]),
            "upper_right": tuple(self.motor_limits["upper_right"]),
        }
