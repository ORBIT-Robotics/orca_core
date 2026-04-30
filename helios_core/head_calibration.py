"""HELIOS 3-DoF head calibration pipeline (limits, neutral, IMU probing, fitting, validation)."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import csv
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
    read_yaml,
    utc_now_iso,
    write_yaml,
)
from hardware.head_hardware import HeliosHeadHardware
from helios_core.head_model import HeliosHeadCalibrationModel
from helios_core.sensors.zedx_head_imu import ZedXHeadImu


@dataclass
class LimitSearchResult:
    axis: str
    direction: int
    limit_rad: float
    reason: str


class HeliosHeadCalibrator:
    """Field-oriented calibration workflow for the tendon-driven HELIOS head."""

    def __init__(self, config_path: Optional[str] = None, role: Optional[str] = DEFAULT_HEAD_ROLE):
        config, cfg_path = load_head_config(config_path, role=role)
        self.config = config
        self.config_path = cfg_path

        self.motor_ids: HeadMotorIDs = parse_motor_ids(self.config)
        self.hardware = HeliosHeadHardware(self.config, self.motor_ids)

        imu_cfg = dict(self.config.get("imu", {}))
        serial_number = imu_cfg.get("serial_number")
        if serial_number is None:
            # fallback to shared camera config field to keep existing repo convention
            camera_cfg = read_yaml("configs/camera.yaml")
            serial_number = camera_cfg.get("zedx_head_imu_serial_number")
        if serial_number is None:
            raise ValueError(
                "IMU serial number is not configured. Set imu.serial_number in "
                "orca_core/models/helios_head/config.yaml or "
                "zedx_head_imu_serial_number in configs/camera.yaml."
            )

        self.imu = ZedXHeadImu(
            serial_number=int(serial_number),
            max_sample_age_sec=float(imu_cfg.get("max_sample_age_sec", 0.25)),
            sample_attempts=int(imu_cfg.get("sample_attempts", 25)),
            sample_sleep_sec=float(imu_cfg.get("sample_sleep_sec", 0.01)),
        )

        self.calibration_path: Path = calibration_output_path(self.config, self.config_path)
        self.log_root: Path = calibration_log_dir(self.config, self.config_path)

        self.motor_limits: Dict[str, tuple[float, float]] = {}
        self.neutral_motors: Optional[Dict[str, float]] = None
        self.neutral_imu_quat: Optional[np.ndarray] = None
        self.probe_samples: List[Dict] = []
        self.validation_samples: List[Dict] = []
        self._latest_model: Optional[HeliosHeadCalibrationModel] = None

    @property
    def axis_order(self) -> tuple[str, str, str]:
        return ("yaw", "upper_left", "upper_right")

    def connect(self) -> None:
        self.hardware.connect()
        calib_current = float(self.config.get("hardware", {}).get("calibration_current_limit_ma", 300.0))
        self.hardware.set_current_limit(calib_current)
        self.imu.open()
        print("[helios_head] Hardware and IMU connected.")

    def disconnect(self) -> None:
        try:
            self.return_to_neutral(ignore_if_missing=True)
        except Exception:
            pass
        try:
            self.imu.close()
        finally:
            self.hardware.disconnect()
        print("[helios_head] Hardware and IMU disconnected.")

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
        """Wait until the commanded position is reached closely enough."""
        desired = np.asarray(target, dtype=float).reshape(3,)
        stable_count = 0
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        last_pos = self.hardware.get_motor_positions(as_dict=False)

        while time.monotonic() < deadline:
            pos = self.hardware.get_motor_positions(as_dict=False)
            last_pos = pos
            err = np.max(np.abs(pos - desired))
            if err <= position_tolerance_rad:
                stable_count += 1
                if stable_count >= max(1, int(stable_cycles)):
                    return pos
            else:
                stable_count = 0
            time.sleep(max(0.0, float(poll_period_sec)))

        return last_pos

    def _search_limit_for_axis(self, axis: str, direction: int, anchor_positions: np.ndarray) -> LimitSearchResult:
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

        # Conservative fallback when we run out of search steps.
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
            print(
                f"  + direction -> {hi.limit_rad:.5f} rad ({hi.reason})"
            )

            self.hardware.command_motor_positions(anchor, limits=None, max_step_rad=0.0)
            self._wait_for_target(anchor)
            time.sleep(reverse_direction_settle_sec)

            lo = self._search_limit_for_axis(axis, -1, anchor)
            limits[axis][0] = lo.limit_rad
            print(
                f"  - direction -> {lo.limit_rad:.5f} rad ({lo.reason})"
            )

            # Return close to where this axis started before moving to next axis.
            restore = self.hardware.get_motor_positions(as_dict=False)
            restore[idx] = base[idx]
            self.hardware.command_motor_positions(restore, limits=None, max_step_rad=0.0)
            self._wait_for_target(restore)

        self.motor_limits = {
            axis: (float(vals[0]), float(vals[1])) for axis, vals in limits.items()
        }
        print(f"[helios_head] Motor limits discovered: {self.motor_limits}")
        return self.motor_limits

    def capture_neutral(self) -> Dict:
        settle_cfg = dict((self.config.get("calibration") or {}).get("neutral", {}))
        settle_time_sec = max(0.0, float(settle_cfg.get("settle_time_sec", 0.35)))
        time.sleep(settle_time_sec)

        motor_pos = self.hardware.get_motor_positions(as_dict=False)
        self._validate_neutral_within_limits(motor_pos, margin_rad=0.0)
        imu_sample = self.imu.capture_neutral()

        self.neutral_motors = {
            "yaw": float(motor_pos[0]),
            "upper_left": float(motor_pos[1]),
            "upper_right": float(motor_pos[2]),
        }
        self.neutral_imu_quat = imu_sample.quaternion_xyzw.copy()

        payload = {
            "motors": dict(self.neutral_motors),
            "imu_quaternion_xyzw": self.neutral_imu_quat.tolist(),
            "imu_rpy_rad": {
                "yaw": float(imu_sample.relative_yaw_pitch_roll_rad[0]),
                "pitch": float(imu_sample.relative_yaw_pitch_roll_rad[1]),
                "roll": float(imu_sample.relative_yaw_pitch_roll_rad[2]),
            },
        }
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
        if self.neutral_motors is None or self.neutral_imu_quat is None:
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

    def _record_probe_sample(
        self,
        *,
        trial_name: str,
        repeat_idx: int,
        requested_offsets: np.ndarray,
        commanded_targets: np.ndarray,
        motor_state,
        imu_sample,
    ) -> Dict:
        neutral = np.array(
            [
                self.neutral_motors["yaw"],
                self.neutral_motors["upper_left"],
                self.neutral_motors["upper_right"],
            ],
            dtype=float,
        )
        achieved_offsets = np.asarray(commanded_targets, dtype=float).reshape(3,) - neutral
        measured_ypr = np.asarray(imu_sample.relative_yaw_pitch_roll_rad, dtype=float).reshape(3,)

        sample = {
            "timestamp_utc": utc_now_iso(),
            "trial": trial_name,
            "repeat": int(repeat_idx),
            "requested_dm_yaw": float(requested_offsets[0]),
            "requested_dm_upper_left": float(requested_offsets[1]),
            "requested_dm_upper_right": float(requested_offsets[2]),
            "commanded_dm_yaw": float(achieved_offsets[0]),
            "commanded_dm_upper_left": float(achieved_offsets[1]),
            "commanded_dm_upper_right": float(achieved_offsets[2]),
            "measured_dyaw": float(measured_ypr[0]),
            "measured_dpitch": float(measured_ypr[1]),
            "measured_droll": float(measured_ypr[2]),
            "motor_yaw": float(motor_state.positions_rad[0]),
            "motor_upper_left": float(motor_state.positions_rad[1]),
            "motor_upper_right": float(motor_state.positions_rad[2]),
            "motor_current_yaw": float(motor_state.currents_ma[0]),
            "motor_current_upper_left": float(motor_state.currents_ma[1]),
            "motor_current_upper_right": float(motor_state.currents_ma[2]),
        }
        return sample

    def _probe_motion_table(self) -> Dict[str, np.ndarray]:
        probe_cfg = dict((self.config.get("calibration") or {}).get("probe", {}))
        dy = float(probe_cfg.get("yaw_probe_delta_rad", 0.04))
        dc = float(probe_cfg.get("upper_common_probe_delta_rad", 0.03))
        dd = float(probe_cfg.get("upper_diff_probe_delta_rad", 0.03))
        return {
            "yaw_pos": np.array([+dy, 0.0, 0.0], dtype=float),
            "yaw_neg": np.array([-dy, 0.0, 0.0], dtype=float),
            "upper_common_pos": np.array([0.0, +dc, +dc], dtype=float),
            "upper_common_neg": np.array([0.0, -dc, -dc], dtype=float),
            "upper_diff_pos": np.array([0.0, +dd, -dd], dtype=float),
            "upper_diff_neg": np.array([0.0, -dd, +dd], dtype=float),
        }

    def _fit_motor_to_head_jacobian(self, samples: List[Dict]) -> Dict:
        if len(samples) < 6:
            raise RuntimeError("Not enough probe samples for Jacobian fit.")

        x = np.array(
            [
                [
                    s["commanded_dm_yaw"],
                    s["commanded_dm_upper_left"],
                    s["commanded_dm_upper_right"],
                ]
                for s in samples
            ],
            dtype=float,
        )
        y = np.array(
            [
                [
                    s["measured_dyaw"],
                    s["measured_dpitch"],
                    s["measured_droll"],
                ]
                for s in samples
            ],
            dtype=float,
        )

        b, _, _, _ = np.linalg.lstsq(x, y, rcond=None)
        # y = x @ b, so J = b.T in [dyaw,dpitch,droll]^T = J [dm_yaw,dm_ul,dm_ur]^T
        j = b.T
        residual = y - (x @ b)
        rmse = np.sqrt(np.mean(np.square(residual), axis=0))

        eps = 1e-6
        c_vals = []
        a_vals = []
        b_vals = []
        for row_x, row_y in zip(x, y):
            dm_yaw, dm_ul, dm_ur = row_x
            dyaw, dpitch, droll = row_y
            sum_upper = dm_ul + dm_ur
            diff_upper = dm_ul - dm_ur

            if abs(dm_yaw) > eps and abs(sum_upper) <= eps and abs(diff_upper) <= eps:
                c_vals.append(dyaw / dm_yaw)
            if abs(sum_upper) > eps and abs(diff_upper) <= eps and abs(dm_yaw) <= eps:
                a_vals.append(dpitch / sum_upper)
            if abs(diff_upper) > eps and abs(sum_upper) <= eps and abs(dm_yaw) <= eps:
                b_vals.append(droll / diff_upper)

        if not c_vals or not a_vals or not b_vals:
            raise RuntimeError(
                "Probe set did not contain enough pure yaw/common/differential samples to fit gains."
            )

        c = float(np.mean(c_vals))
        a = float(np.mean(a_vals))
        b_coef = float(np.mean(b_vals))

        if np.isclose(c, 0.0) or np.isclose(a, 0.0) or np.isclose(b_coef, 0.0):
            raise RuntimeError(
                f"Degenerate probe fit (a={a}, b={b_coef}, c={c}). Check tension/probe magnitudes/signs."
            )

        raw_k_y = 1.0 / c
        raw_k_p = 1.0 / (2.0 * a)
        raw_k_r = 1.0 / (2.0 * b_coef)

        sign_cfg = dict(self.config.get("mapping", {}))
        final_signs = {
            "yaw_sign": float(sign_cfg.get("yaw_sign", 1.0)) * float(np.sign(raw_k_y) or 1.0),
            "pitch_sign": float(sign_cfg.get("pitch_sign", 1.0)) * float(np.sign(raw_k_p) or 1.0),
            "roll_sign": float(sign_cfg.get("roll_sign", 1.0)) * float(np.sign(raw_k_r) or 1.0),
        }
        gains = {
            "k_y": abs(float(raw_k_y)),
            "k_p": abs(float(raw_k_p)),
            "k_r": abs(float(raw_k_r)),
        }

        return {
            "jacobian_motor_to_head": j,
            "fit_rmse_rad": {
                "yaw": float(rmse[0]),
                "pitch": float(rmse[1]),
                "roll": float(rmse[2]),
            },
            "coefficients": {
                "a_pitch_per_sum_upper": a,
                "b_roll_per_diff_upper": b_coef,
                "c_yaw_per_yaw_motor": c,
            },
            "gains": gains,
            "signs": final_signs,
        }

    def run_imu_probe_calibration(self) -> Dict:
        self._ensure_limits()
        self._ensure_neutral()

        probe_cfg = dict((self.config.get("calibration") or {}).get("probe", {}))
        settle_time_sec = max(0.0, float(probe_cfg.get("settle_time_sec", 0.30)))
        rest_time_sec = max(0.0, float(probe_cfg.get("rest_time_sec", 0.15)))
        neutral_return_settle_sec = max(0.0, float(probe_cfg.get("neutral_return_settle_sec", 0.50)))
        probe_command_max_step_rad = max(0.0, float(probe_cfg.get("command_max_step_rad", 0.20)))
        repeats = max(1, int(probe_cfg.get("repeats", 3)))

        motions = self._probe_motion_table()
        safety_margin = float(self.config.get("safety", {}).get("motor_limit_margin_rad", 0.0))
        limits = self._motor_limits_with_margin(margin_rad=safety_margin)

        neutral = np.array(
            [
                self.neutral_motors["yaw"],
                self.neutral_motors["upper_left"],
                self.neutral_motors["upper_right"],
            ],
            dtype=float,
        )
        self._validate_neutral_within_limits(
            neutral,
            margin_rad=float(self.config.get("safety", {}).get("motor_limit_margin_rad", 0.0)),
        )

        samples: List[Dict] = []
        for repeat_idx in range(repeats):
            for trial_name, offsets in motions.items():
                self.return_to_neutral()
                self._wait_for_target(neutral, timeout_sec=max(3.0, neutral_return_settle_sec + 2.0))
                time.sleep(neutral_return_settle_sec)

                target = neutral + offsets
                commanded = self.hardware.command_motor_positions(
                    target,
                    limits=limits,
                    max_step_rad=probe_command_max_step_rad,
                )
                self._wait_for_target(
                    commanded,
                    timeout_sec=max(3.0, settle_time_sec + 2.0),
                    position_tolerance_rad=max(0.02, probe_command_max_step_rad * 0.25),
                    stable_cycles=3,
                )
                time.sleep(settle_time_sec)

                motor_state = self.hardware.read_state()
                imu_sample = self.imu.read()

                sample = self._record_probe_sample(
                    trial_name=trial_name,
                    repeat_idx=repeat_idx,
                    requested_offsets=offsets,
                    commanded_targets=commanded,
                    motor_state=motor_state,
                    imu_sample=imu_sample,
                )
                samples.append(sample)
                print(
                    "[helios_head] probe "
                    f"{trial_name} rep={repeat_idx} "
                    f"dm=({sample['commanded_dm_yaw']:.4f},"
                    f"{sample['commanded_dm_upper_left']:.4f},"
                    f"{sample['commanded_dm_upper_right']:.4f}) "
                    f"dYPR=({sample['measured_dyaw']:.4f},"
                    f"{sample['measured_dpitch']:.4f},"
                    f"{sample['measured_droll']:.4f})"
                )

        self.return_to_neutral()
        self._wait_for_target(neutral, timeout_sec=max(3.0, neutral_return_settle_sec + 2.0))
        self.probe_samples = samples

        fit = self._fit_motor_to_head_jacobian(samples)
        virtual_limits_cfg = dict(self.config.get("safety", {}).get("virtual_limits_rad", {}))

        self._latest_model = HeliosHeadCalibrationModel(
            motor_ids=self.motor_ids,
            motor_limits=self.motor_limits,
            neutral_motors=self.neutral_motors,
            gains=fit["gains"],
            signs=fit["signs"],
            virtual_limits_rad={
                "yaw": float(virtual_limits_cfg.get("yaw", 0.7)),
                "pitch": float(virtual_limits_cfg.get("pitch", 0.55)),
                "roll": float(virtual_limits_cfg.get("roll", 0.55)),
            },
            jacobian_motor_to_head=np.asarray(fit["jacobian_motor_to_head"], dtype=float).reshape(3, 3),
            fit_rmse_rad=dict(fit["fit_rmse_rad"]),
        )

        print(
            "[helios_head] fit gains: "
            f"k_y={self._latest_model.gains['k_y']:.4f}, "
            f"k_p={self._latest_model.gains['k_p']:.4f}, "
            f"k_r={self._latest_model.gains['k_r']:.4f} | "
            f"signs={self._latest_model.signs}"
        )
        return fit

    def validate(self) -> Dict:
        if self._latest_model is None:
            raise RuntimeError("No calibrated model available. Run run_imu_probe_calibration() first.")

        cfg = dict((self.config.get("calibration") or {}).get("validation", {}))
        settle_time_sec = max(0.0, float(cfg.get("settle_time_sec", 0.30)))
        commands = dict(cfg.get("commands_rad", {}))

        trials = []

        def _run_trial(name: str, virtual_cmd: np.ndarray):
            self.return_to_neutral()
            target = self._latest_model.virtual_to_motor_targets(virtual_cmd)
            target = self._latest_model.clip_motor_targets(
                target,
                margin_rad=float(self.config.get("safety", {}).get("motor_limit_margin_rad", 0.0)),
            )
            commanded = self.hardware.command_motor_positions(
                target,
                limits=self._latest_model.motor_limits_for_hardware(),
                max_step_rad=0.1,
            )
            time.sleep(settle_time_sec)
            imu_sample = self.imu.read()
            measured = np.asarray(imu_sample.relative_yaw_pitch_roll_rad, dtype=float).reshape(3,)
            error = measured - virtual_cmd

            trial = {
                "timestamp_utc": utc_now_iso(),
                "name": name,
                "command_yaw": float(virtual_cmd[0]),
                "command_pitch": float(virtual_cmd[1]),
                "command_roll": float(virtual_cmd[2]),
                "measured_yaw": float(measured[0]),
                "measured_pitch": float(measured[1]),
                "measured_roll": float(measured[2]),
                "error_yaw": float(error[0]),
                "error_pitch": float(error[1]),
                "error_roll": float(error[2]),
                "motor_yaw": float(commanded[0]),
                "motor_upper_left": float(commanded[1]),
                "motor_upper_right": float(commanded[2]),
                "leak_abs_yaw": float(abs(measured[0] - virtual_cmd[0])),
                "leak_abs_pitch": float(abs(measured[1] - virtual_cmd[1])),
                "leak_abs_roll": float(abs(measured[2] - virtual_cmd[2])),
            }
            print(
                "[helios_head] validate "
                f"{name} cmd=({virtual_cmd[0]:+.3f},{virtual_cmd[1]:+.3f},{virtual_cmd[2]:+.3f}) "
                f"meas=({measured[0]:+.3f},{measured[1]:+.3f},{measured[2]:+.3f})"
            )
            trials.append(trial)

        for yaw in list(commands.get("yaw", [])):
            _run_trial("yaw", np.array([float(yaw), 0.0, 0.0], dtype=float))
        for pitch in list(commands.get("pitch", [])):
            _run_trial("pitch", np.array([0.0, float(pitch), 0.0], dtype=float))
        for roll in list(commands.get("roll", [])):
            _run_trial("roll", np.array([0.0, 0.0, float(roll)], dtype=float))
        for pair in list(commands.get("pitch_roll", [])):
            if not isinstance(pair, (list, tuple)) or len(pair) != 2:
                continue
            _run_trial(
                "pitch_roll",
                np.array([0.0, float(pair[0]), float(pair[1])], dtype=float),
            )

        self.return_to_neutral()
        self.validation_samples = trials

        if not trials:
            summary = {"num_trials": 0}
        else:
            err = np.array(
                [[t["error_yaw"], t["error_pitch"], t["error_roll"]] for t in trials],
                dtype=float,
            )
            summary = {
                "num_trials": int(len(trials)),
                "rmse_yaw": float(np.sqrt(np.mean(np.square(err[:, 0])))),
                "rmse_pitch": float(np.sqrt(np.mean(np.square(err[:, 1])))),
                "rmse_roll": float(np.sqrt(np.mean(np.square(err[:, 2])))),
                "max_abs_error_yaw": float(np.max(np.abs(err[:, 0]))),
                "max_abs_error_pitch": float(np.max(np.abs(err[:, 1]))),
                "max_abs_error_roll": float(np.max(np.abs(err[:, 2]))),
            }

        return {"trials": trials, "summary": summary}

    @staticmethod
    def _write_csv(path: Path, rows: List[Dict]) -> None:
        if not rows:
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = list(rows[0].keys())
        with open(path, "w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)

    def save_results(self) -> Dict:
        if self._latest_model is None:
            raise RuntimeError("No model to save. Run run_imu_probe_calibration() first.")

        now_iso = utc_now_iso()
        validation_payload = self.validate()

        payload = {
            "version": 1,
            "calibrated": True,
            "updated_at": now_iso,
            "hardware": {
                "motor_ids": self.motor_ids.as_dict,
                "imu_serial_number": int(self.imu.serial_number),
            },
            "motor_limits": {
                axis: [float(val[0]), float(val[1])] for axis, val in self.motor_limits.items()
            },
            "neutral": {
                "motors": {
                    axis: float(self.neutral_motors[axis]) for axis in self.axis_order
                },
                "imu_quaternion_xyzw": self.neutral_imu_quat.tolist(),
                "imu_rpy_rad": {
                    "yaw": 0.0,
                    "pitch": 0.0,
                    "roll": 0.0,
                },
            },
            "gains": {
                "k_y": float(self._latest_model.gains["k_y"]),
                "k_p": float(self._latest_model.gains["k_p"]),
                "k_r": float(self._latest_model.gains["k_r"]),
            },
            "signs": {
                "yaw_sign": float(self._latest_model.signs["yaw_sign"]),
                "pitch_sign": float(self._latest_model.signs["pitch_sign"]),
                "roll_sign": float(self._latest_model.signs["roll_sign"]),
            },
            "model": {
                "jacobian_motor_to_head": self._latest_model.jacobian_motor_to_head.tolist(),
                "fit_rmse_rad": dict(self._latest_model.fit_rmse_rad or {}),
            },
            "virtual_limits_rad": {
                "yaw": float(self._latest_model.virtual_limits_rad["yaw"]),
                "pitch": float(self._latest_model.virtual_limits_rad["pitch"]),
                "roll": float(self._latest_model.virtual_limits_rad["roll"]),
            },
            "validation": validation_payload,
        }

        write_yaml(self.calibration_path, payload)

        stamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        run_dir = self.log_root / f"run_{stamp}"
        run_dir.mkdir(parents=True, exist_ok=True)

        self._write_csv(run_dir / "probe_samples.csv", self.probe_samples)
        self._write_csv(run_dir / "validation_samples.csv", self.validation_samples)
        with open(run_dir / "calibration_payload.json", "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

        print(f"[helios_head] Saved calibration YAML: {self.calibration_path}")
        print(f"[helios_head] Saved calibration logs: {run_dir}")
        return {
            "calibration_yaml": str(self.calibration_path),
            "log_dir": str(run_dir),
            "validation_summary": validation_payload.get("summary", {}),
        }

    def run_full_pipeline(self) -> Dict:
        """Run limit search -> neutral capture -> IMU probe -> validation + save."""
        self.find_motor_limits()
        self.capture_neutral()
        self.run_imu_probe_calibration()
        return self.save_results()
