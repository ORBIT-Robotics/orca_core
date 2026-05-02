"""HELIOS 3-DoF head endpoint calibration pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from helios_core.head_config import (
    DEFAULT_HEAD_ROLE,
    HeadMotorIDs,
    calibration_log_dir,
    calibration_output_path,
    load_head_config,
    parse_motor_ids,
    utc_now_iso,
    write_yaml,
)
from hardware.head_hardware import HeliosHeadHardware
from helios_core.head_model import (
    HeliosHeadCalibrationModel,
    MOTOR_AXES,
    VIRTUAL_AXES,
    derive_endpoint_joint_to_motor_ratios,
)


@dataclass
class LimitSearchResult:
    axis: str
    direction: int
    limit_rad: float
    reason: str


class HeliosHeadCalibrator:
    """Field-oriented endpoint calibration for the tendon-driven HELIOS head."""

    def __init__(self, config_path: Optional[str] = None, role: Optional[str] = DEFAULT_HEAD_ROLE):
        config, cfg_path = load_head_config(config_path, role=role)
        self.config = config
        self.config_path = cfg_path

        self.motor_ids: HeadMotorIDs = parse_motor_ids(self.config)
        self.hardware = HeliosHeadHardware(self.config, self.motor_ids)

        self.calibration_path: Path = calibration_output_path(self.config, self.config_path)
        self.log_root: Path = calibration_log_dir(self.config, self.config_path)

        self.motor_limits: Dict[str, tuple[float, float]] = {}
        self.neutral_motors: Optional[Dict[str, float]] = None
        self.endpoint_ratios: Optional[Dict[str, float]] = None
        self._latest_model: Optional[HeliosHeadCalibrationModel] = None

    @property
    def axis_order(self) -> tuple[str, str, str]:
        return MOTOR_AXES

    def connect(self) -> None:
        self.hardware.connect()
        calib_current = float(self.config.get("hardware", {}).get("calibration_current_limit_ma", 300.0))
        self.hardware.set_current_limit(calib_current)
        print("[helios_head] Hardware connected.")

    def disconnect(self) -> None:
        try:
            self.return_to_neutral(ignore_if_missing=True)
        except Exception:
            pass
        self.hardware.disconnect()
        print("[helios_head] Hardware disconnected.")

    def tension_assist(self) -> Dict:
        """
        Software assist for manual tendon tensioning.

        This intentionally does not claim to automate tendon threading/preload.
        """
        cfg = dict((self.config.get("calibration") or {}).get("tension", {}))
        wiggle_enabled = bool(cfg.get("wiggle_enabled", False))
        wiggle_duration_sec = float(cfg.get("wiggle_duration_sec", 3.0))
        wiggle_step_rad = float(cfg.get("wiggle_step_rad", 0.04))
        wiggle_period_sec = float(cfg.get("wiggle_period_sec", 0.10))

        if wiggle_enabled:
            print("[helios_head] Running optional tension wiggle helper...")
            t_start = time.monotonic()
            direction = 1.0
            while time.monotonic() - t_start < wiggle_duration_sec:
                step = np.array([0.0, direction * wiggle_step_rad, -direction * wiggle_step_rad], dtype=float)
                self.hardware.command_relative_offsets(step, limits=None)
                direction *= -1.0
                time.sleep(max(0.0, wiggle_period_sec))

        hold_pos = self.hardware.hold_current_position()
        print(
            "[helios_head] Holding current motor positions for manual tensioning. "
            "Adjust tendons physically, ensure no slack/over-tension, then continue."
        )
        return {
            "hold_motor_positions_rad": {
                "yaw": float(hold_pos[0]),
                "upper_left": float(hold_pos[1]),
                "upper_right": float(hold_pos[2]),
            }
        }

    def _axis_index(self, axis: str) -> int:
        return self.axis_order.index(axis)

    def _wait_for_target(
        self,
        target: np.ndarray,
        *,
        timeout_sec: float = 3.0,
        position_tolerance_rad: float = 0.02,
        stable_cycles: int = 4,
        poll_period_sec: float = 0.05,
    ) -> np.ndarray:
        desired = np.asarray(target, dtype=float).reshape(3,)
        stable_count = 0
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        last_pos = self.hardware.get_motor_positions(as_dict=False)

        while time.monotonic() < deadline:
            pos = self.hardware.get_motor_positions(as_dict=False)
            last_pos = pos
            err = float(np.max(np.abs(pos - desired)))
            if err <= position_tolerance_rad:
                stable_count += 1
                if stable_count >= max(1, int(stable_cycles)):
                    return pos
            else:
                stable_count = 0
            time.sleep(max(0.0, float(poll_period_sec)))

        return last_pos

    def _search_limit_for_axis(
        self,
        axis: str,
        direction: int,
        anchor_positions: np.ndarray,
    ) -> LimitSearchResult:
        cfg = dict((self.config.get("calibration") or {}).get("limit_search", {}))
        step_size_rad = abs(float(cfg.get("step_size_rad", 0.5)))
        step_period_sec = max(0.0, float(cfg.get("step_period_sec", 0.02)))
        stable_window = max(3, int(cfg.get("stable_window", 10)))
        stable_threshold_rad = abs(float(cfg.get("stable_threshold_rad", 0.003)))
        max_steps_per_direction = max(20, int(cfg.get("max_steps_per_direction", 800)))
        current_spike_ma = abs(float(cfg.get("current_spike_ma", 700.0)))
        abort_current_ma = abs(
            float(cfg.get("abort_current_ma", max(current_spike_ma * 1.5, current_spike_ma + 100.0)))
        )

        idx = self._axis_index(axis)
        target = np.asarray(anchor_positions, dtype=float).reshape(3,).copy()
        history: List[float] = []

        for _ in range(max_steps_per_direction):
            target[idx] += float(direction) * step_size_rad
            self.hardware.command_motor_positions(target, limits=None, max_step_rad=step_size_rad * 1.5)
            time.sleep(step_period_sec)

            state = self.hardware.read_state()
            pos = float(state.positions_rad[idx])
            cur = abs(float(state.currents_ma[idx]))
            target[idx] = pos
            history.append(pos)

            if abort_current_ma > 0.0 and cur >= abort_current_ma:
                raise RuntimeError(
                    f"Abort: motor current {cur:.2f} mA exceeded abort threshold "
                    f"{abort_current_ma:.2f} mA on axis '{axis}'."
                )
            if current_spike_ma > 0.0 and cur >= current_spike_ma:
                return LimitSearchResult(axis=axis, direction=direction, limit_rad=pos, reason="current_spike")

            if len(history) >= stable_window:
                window = np.asarray(history[-stable_window:], dtype=float)
                if np.max(window) - np.min(window) <= stable_threshold_rad:
                    return LimitSearchResult(
                        axis=axis,
                        direction=direction,
                        limit_rad=float(np.mean(window)),
                        reason="stable_position",
                    )

        return LimitSearchResult(axis=axis, direction=direction, limit_rad=float(history[-1]), reason="max_steps")

    def find_motor_limits(self) -> Dict[str, tuple[float, float]]:
        cfg = dict((self.config.get("calibration") or {}).get("limit_search", {}))
        reverse_direction_settle_sec = max(0.0, float(cfg.get("reverse_direction_settle_sec", 0.75)))
        base = self.hardware.get_motor_positions(as_dict=False)
        limits: Dict[str, List[float]] = {
            "yaw": [None, None],
            "upper_left": [None, None],
            "upper_right": [None, None],
        }

        for axis in self.axis_order:
            print(f"[helios_head] Searching limits for {axis}...")
            idx = self._axis_index(axis)
            anchor = self.hardware.get_motor_positions(as_dict=False)

            hi = self._search_limit_for_axis(axis, +1, anchor)
            limits[axis][1] = hi.limit_rad
            print(f"  + direction -> {hi.limit_rad:.5f} rad ({hi.reason})")

            self.hardware.command_motor_positions(anchor, limits=None, max_step_rad=0.0)
            self._wait_for_target(anchor)
            time.sleep(reverse_direction_settle_sec)

            lo = self._search_limit_for_axis(axis, -1, anchor)
            limits[axis][0] = lo.limit_rad
            print(f"  - direction -> {lo.limit_rad:.5f} rad ({lo.reason})")

            restore = self.hardware.get_motor_positions(as_dict=False)
            restore[idx] = base[idx]
            self.hardware.command_motor_positions(restore, limits=None, max_step_rad=0.0)
            self._wait_for_target(restore)

        self.motor_limits = {
            axis: (float(vals[0]), float(vals[1])) for axis, vals in limits.items()
        }
        print(f"[helios_head] Motor limits discovered: {self.motor_limits}")
        self.endpoint_ratios = None
        self._latest_model = None
        return self.motor_limits

    def capture_neutral(self) -> Dict:
        settle_cfg = dict((self.config.get("calibration") or {}).get("neutral", {}))
        settle_time_sec = max(0.0, float(settle_cfg.get("settle_time_sec", 0.35)))
        time.sleep(settle_time_sec)

        motor_pos = self.hardware.get_motor_positions(as_dict=False)
        self._validate_neutral_within_limits(motor_pos, margin_rad=0.0)
        self.neutral_motors = {
            "yaw": float(motor_pos[0]),
            "upper_left": float(motor_pos[1]),
            "upper_right": float(motor_pos[2]),
        }
        self.endpoint_ratios = None
        self._latest_model = None

        payload = {"motors": dict(self.neutral_motors)}
        print(f"[helios_head] Captured neutral: {payload['motors']}")
        return payload

    def release_for_manual_neutral(self) -> np.ndarray:
        """Release torque so the operator can manually place the head at neutral."""
        pos = self.hardware.get_motor_positions(as_dict=False)
        self.hardware.disable_torque()
        print(
            "[helios_head] Torque disabled for neutral placement. "
            "Move the head by hand to the desired neutral pose, then continue."
        )
        return pos

    def prepare_capture_neutral(self) -> None:
        """Re-enable control and hold the hand-placed neutral pose before capture."""
        self.hardware.enable_torque()
        self.hardware.set_current_limit(
            float(self.config.get("hardware", {}).get("calibration_current_limit_ma", 180.0))
        )
        self.hardware.hold_current_position()

    def _ensure_neutral(self) -> None:
        if self.neutral_motors is None:
            raise RuntimeError("Neutral is not captured yet. Run capture_neutral() first.")

    def _ensure_limits(self) -> None:
        if not self.motor_limits:
            raise RuntimeError("Motor limits are not calibrated yet. Run find_motor_limits() first.")

    def _motor_limits_with_margin(self, margin_rad: float = 0.0) -> Dict[str, tuple[float, float]]:
        self._ensure_limits()
        margin = max(0.0, float(margin_rad))
        out = {}
        for axis, (lo, hi) in self.motor_limits.items():
            out[axis] = (lo + margin, hi - margin)
        return out

    def _validate_neutral_within_limits(self, motor_pos: np.ndarray, *, margin_rad: float = 0.0) -> None:
        if not self.motor_limits:
            return

        pos = np.asarray(motor_pos, dtype=float).reshape(3,)
        limits = self._motor_limits_with_margin(margin_rad=margin_rad)
        violations: List[str] = []
        for idx, axis in enumerate(self.axis_order):
            lo, hi = limits[axis]
            value = float(pos[idx])
            if value < lo or value > hi:
                violations.append(f"{axis}={value:.5f} rad outside [{lo:.5f}, {hi:.5f}]")

        if violations:
            raise RuntimeError(
                "Captured neutral is outside calibrated motor limits: "
                + "; ".join(violations)
                + ". Reposition the head within the discovered range and retry neutral capture."
            )

    def return_to_neutral(self, ignore_if_missing: bool = False) -> np.ndarray:
        if self.neutral_motors is None:
            if ignore_if_missing:
                return self.hardware.get_motor_positions(as_dict=False)
            raise RuntimeError("Neutral is unknown; cannot return to neutral.")

        target = np.array(
            [
                self.neutral_motors["yaw"],
                self.neutral_motors["upper_left"],
                self.neutral_motors["upper_right"],
            ],
            dtype=float,
        )
        limits = self._motor_limits_with_margin(margin_rad=0.0) if self.motor_limits else None
        return self.hardware.command_motor_positions(target, limits=limits, max_step_rad=0.0)

    def _virtual_limits_from_config(self) -> Dict[str, float]:
        virtual_limits_cfg = dict(self.config.get("safety", {}).get("virtual_limits_rad", {}))
        return {
            "yaw": float(virtual_limits_cfg.get("yaw", 0.7)),
            "pitch": float(virtual_limits_cfg.get("pitch", 0.55)),
            "roll": float(virtual_limits_cfg.get("roll", 0.55)),
        }

    def _signs_from_config(self) -> Dict[str, float]:
        sign_cfg = dict(self.config.get("mapping", {}))
        return {
            "yaw_sign": float(sign_cfg.get("yaw_sign", 1.0)),
            "pitch_sign": float(sign_cfg.get("pitch_sign", 1.0)),
            "roll_sign": float(sign_cfg.get("roll_sign", 1.0)),
        }

    def build_endpoint_model(self) -> HeliosHeadCalibrationModel:
        self._ensure_limits()
        self._ensure_neutral()

        calibration_cfg = dict(self.config.get("calibration", {}))
        virtual_limits = self._virtual_limits_from_config()
        self.endpoint_ratios = derive_endpoint_joint_to_motor_ratios(
            self.motor_limits,
            self.neutral_motors,
            virtual_limits,
            min_ratio=float(calibration_cfg.get("endpoint_min_ratio", 1e-6)),
        )
        self._latest_model = HeliosHeadCalibrationModel(
            motor_ids=self.motor_ids,
            motor_limits=self.motor_limits,
            neutral_motors=self.neutral_motors,
            joint_to_motor_ratios=self.endpoint_ratios,
            signs=self._signs_from_config(),
            virtual_limits_rad=virtual_limits,
        )
        print(
            "[helios_head] endpoint ratios: "
            + ", ".join(
                f"{axis}={self.endpoint_ratios[axis]:.5f} motor_rad/virtual_rad"
                for axis in VIRTUAL_AXES
            )
        )
        return self._latest_model

    def _calibration_payload(self) -> Dict:
        model = self._latest_model or self.build_endpoint_model()
        return {
            "version": 2,
            "calibrated": True,
            "updated_at": utc_now_iso(),
            "hardware": {
                "motor_ids": self.motor_ids.as_dict,
            },
            "motor_limits": {
                axis: [float(val[0]), float(val[1])] for axis, val in self.motor_limits.items()
            },
            "neutral": {
                "motors": {
                    axis: float(self.neutral_motors[axis]) for axis in self.axis_order
                },
            },
            "joint_to_motor_ratios": {
                axis: float(model.joint_to_motor_ratios[axis]) for axis in VIRTUAL_AXES
            },
            "signs": {
                "yaw_sign": float(model.signs["yaw_sign"]),
                "pitch_sign": float(model.signs["pitch_sign"]),
                "roll_sign": float(model.signs["roll_sign"]),
            },
            "virtual_limits_rad": {
                axis: float(model.virtual_limits_rad[axis]) for axis in VIRTUAL_AXES
            },
        }

    def save_results(self) -> Dict:
        payload = self._calibration_payload()
        write_yaml(self.calibration_path, payload)

        stamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        run_dir = self.log_root / f"run_{stamp}"
        run_dir.mkdir(parents=True, exist_ok=True)
        with open(run_dir / "calibration_payload.json", "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

        print(f"[helios_head] Saved calibration YAML: {self.calibration_path}")
        print(f"[helios_head] Saved calibration log: {run_dir / 'calibration_payload.json'}")
        return {
            "calibration_yaml": str(self.calibration_path),
            "log_dir": str(run_dir),
            "joint_to_motor_ratios": dict(payload["joint_to_motor_ratios"]),
        }

    def run_full_pipeline(self) -> Dict:
        """Run limit search -> neutral capture -> endpoint model save."""
        self.find_motor_limits()
        self.capture_neutral()
        return self.save_results()
