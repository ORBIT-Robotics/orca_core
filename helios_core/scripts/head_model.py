"""HELIOS head kinematic mapping between virtual [yaw,pitch,roll] and motor targets."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from helios_core.scripts.head_config import HeadMotorIDs


@dataclass
class HeliosHeadCalibrationModel:
    motor_ids: HeadMotorIDs
    motor_limits: Dict[str, tuple[float, float]]
    neutral_motors: Dict[str, float]
    gains: Dict[str, float]
    signs: Dict[str, float]
    virtual_limits_rad: Dict[str, float]
    jacobian_motor_to_head: Optional[np.ndarray] = None
    fit_rmse_rad: Optional[Dict[str, float]] = None

    @staticmethod
    def _as_float(value, name: str) -> float:
        if value is None:
            raise ValueError(f"Missing calibration value for {name}")
        value_f = float(value)
        if not np.isfinite(value_f):
            raise ValueError(f"Non-finite calibration value for {name}: {value}")
        return value_f

    @classmethod
    def from_yaml_dict(cls, data: Dict, motor_ids: Optional[HeadMotorIDs] = None):
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
        for axis in ("yaw", "upper_left", "upper_right"):
            lim = raw_limits.get(axis)
            if lim is None or len(lim) != 2:
                raise ValueError(f"Invalid motor_limits.{axis}: {lim}")
            lo = cls._as_float(lim[0], f"motor_limits.{axis}[0]")
            hi = cls._as_float(lim[1], f"motor_limits.{axis}[1]")
            if lo >= hi:
                raise ValueError(f"Expected min < max for motor_limits.{axis}, got {lim}")
            motor_limits[axis] = (lo, hi)

        neutral = dict((data.get("neutral") or {}).get("motors", {}))
        neutral_motors = {
            axis: cls._as_float(neutral.get(axis), f"neutral.motors.{axis}")
            for axis in ("yaw", "upper_left", "upper_right")
        }

        raw_gains = dict(data.get("gains", {}))
        gains = {
            "k_y": cls._as_float(raw_gains.get("k_y"), "gains.k_y"),
            "k_p": cls._as_float(raw_gains.get("k_p"), "gains.k_p"),
            "k_r": cls._as_float(raw_gains.get("k_r"), "gains.k_r"),
        }
        for key, value in gains.items():
            if np.isclose(value, 0.0):
                raise ValueError(f"Calibration gain {key} cannot be zero.")

        raw_signs = dict(data.get("signs", {}))
        signs = {
            "yaw_sign": float(raw_signs.get("yaw_sign", 1.0)),
            "pitch_sign": float(raw_signs.get("pitch_sign", 1.0)),
            "roll_sign": float(raw_signs.get("roll_sign", 1.0)),
        }
        for key, value in signs.items():
            if value == 0.0:
                raise ValueError(f"Sign {key} cannot be zero.")

        raw_virtual_limits = dict(data.get("virtual_limits_rad", {}))
        virtual_limits = {
            "yaw": float(raw_virtual_limits.get("yaw", 0.7)),
            "pitch": float(raw_virtual_limits.get("pitch", 0.55)),
            "roll": float(raw_virtual_limits.get("roll", 0.55)),
        }

        jac = None
        model = dict(data.get("model", {}))
        raw_j = model.get("jacobian_motor_to_head")
        if raw_j is not None:
            jac_arr = np.asarray(raw_j, dtype=float)
            if jac_arr.shape == (3, 3) and np.all(np.isfinite(jac_arr)):
                jac = jac_arr

        fit_rmse = None
        if isinstance(model.get("fit_rmse_rad"), dict):
            fit_rmse = {
                key: float(value)
                for key, value in model.get("fit_rmse_rad", {}).items()
            }

        return cls(
            motor_ids=motor_ids,
            motor_limits=motor_limits,
            neutral_motors=neutral_motors,
            gains=gains,
            signs=signs,
            virtual_limits_rad=virtual_limits,
            jacobian_motor_to_head=jac,
            fit_rmse_rad=fit_rmse,
        )

    def clip_virtual(self, virtual_cmd: np.ndarray) -> np.ndarray:
        cmd = np.asarray(virtual_cmd, dtype=float).reshape(3,)
        clipped = cmd.copy()
        clipped[0] = np.clip(clipped[0], -abs(self.virtual_limits_rad["yaw"]), abs(self.virtual_limits_rad["yaw"]))
        clipped[1] = np.clip(clipped[1], -abs(self.virtual_limits_rad["pitch"]), abs(self.virtual_limits_rad["pitch"]))
        clipped[2] = np.clip(clipped[2], -abs(self.virtual_limits_rad["roll"]), abs(self.virtual_limits_rad["roll"]))
        return clipped

    def clip_motor_targets(self, motor_targets: np.ndarray, margin_rad: float = 0.0) -> np.ndarray:
        values = np.asarray(motor_targets, dtype=float).reshape(3,)
        clipped = values.copy()
        margin = max(0.0, float(margin_rad))
        for idx, axis in enumerate(("yaw", "upper_left", "upper_right")):
            lo, hi = self.motor_limits[axis]
            clipped[idx] = float(np.clip(clipped[idx], lo + margin, hi - margin))
        return clipped

    def virtual_to_motor_targets(self, virtual_cmd: np.ndarray) -> np.ndarray:
        yaw, pitch, roll = self.clip_virtual(virtual_cmd)

        yaw_eff = self.signs["yaw_sign"] * yaw
        pitch_eff = self.signs["pitch_sign"] * pitch
        roll_eff = self.signs["roll_sign"] * roll

        m_yaw = self.neutral_motors["yaw"] + self.gains["k_y"] * yaw_eff
        m_22 = self.neutral_motors["upper_left"] + self.gains["k_p"] * pitch_eff + self.gains["k_r"] * roll_eff
        m_23 = self.neutral_motors["upper_right"] + self.gains["k_p"] * pitch_eff - self.gains["k_r"] * roll_eff

        return np.array([m_yaw, m_22, m_23], dtype=float)

    def motor_to_virtual(self, motor_positions: np.ndarray) -> np.ndarray:
        motor = np.asarray(motor_positions, dtype=float).reshape(3,)
        dy = (motor[0] - self.neutral_motors["yaw"]) / self.gains["k_y"]
        dp = (
            (motor[1] - self.neutral_motors["upper_left"])
            + (motor[2] - self.neutral_motors["upper_right"])
        ) / (2.0 * self.gains["k_p"])
        dr = (
            (motor[1] - self.neutral_motors["upper_left"])
            - (motor[2] - self.neutral_motors["upper_right"])
        ) / (2.0 * self.gains["k_r"])

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
