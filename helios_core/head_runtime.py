"""HELIOS runtime coordinator (virtual command -> motor targets -> state)."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional, Dict, Any

import numpy as np

from helios_core.head_config import load_head_config, parse_motor_ids, read_yaml, resolve_repo_path
from helios_core.head_model import HeliosHeadCalibrationModel
from hardware.head_hardware import HeliosHeadHardware


class HeliosHeadRuntime:
    """Runtime helper used by ROS and script wrappers for HELIOS head control."""

    def __init__(self, config_path: Optional[str] = None):
        default_path = "orca_core/models/helios_head/config.yaml"
        cfg, cfg_path = load_head_config(config_path or default_path)
        self.config: Dict[str, Any] = cfg
        self.config_path: Path = cfg_path

        self.motor_ids = parse_motor_ids(self.config)
        self.hardware = HeliosHeadHardware(self.config, self.motor_ids)

        calib_rel = str(
            (self.config.get("calibration") or {}).get(
                "output_path",
                "orca_core/models/helios_head/calibration.yaml",
            )
        )
        calib_path = resolve_repo_path(calib_rel)
        calib_data = read_yaml(calib_path)
        if not calib_data:
            raise RuntimeError(
                f"Calibration YAML missing or empty: {calib_path}. "
                "Run scripts/apps/helios_head_calibrate.py first."
            )

        self.model = HeliosHeadCalibrationModel.from_yaml_dict(calib_data, motor_ids=self.motor_ids)
        hw_cfg = dict(self.config.get("hardware", {}))
        self.command_timeout_sec = float(hw_cfg.get("command_timeout_sec", 0.75))
        self.motor_margin_rad = float(dict(self.config.get("safety", {})).get("motor_limit_margin_rad", 0.0))

        self._latest_virtual_cmd = np.zeros(3, dtype=float)
        self._last_cmd_sec = -np.inf

        self.hardware.connect()
        self.hardware.set_current_limit(float(hw_cfg.get("runtime_current_limit_ma", 260.0)))

    def command_virtual(self, virtual_cmd: np.ndarray) -> None:
        values = np.asarray(virtual_cmd, dtype=float).reshape(-1)
        if values.size != 3 or not np.all(np.isfinite(values)):
            raise ValueError("Expected finite virtual command with shape (3,): [yaw, pitch, roll].")
        self._latest_virtual_cmd[:] = values
        self._last_cmd_sec = time.monotonic()

    def step(self) -> np.ndarray:
        now = time.monotonic()
        cmd_age = now - self._last_cmd_sec
        if cmd_age > self.command_timeout_sec:
            virtual_cmd = np.zeros(3, dtype=float)
        else:
            virtual_cmd = self._latest_virtual_cmd.copy()

        target = self.model.virtual_to_motor_targets(virtual_cmd)
        target = self.model.clip_motor_targets(target, margin_rad=self.motor_margin_rad)
        self.hardware.command_motor_positions(
            target,
            limits=self.model.motor_limits_for_hardware(),
        )

        motor_state = self.hardware.read_state()
        return self.model.motor_to_virtual(motor_state.positions_rad)

    def read_virtual_state(self) -> np.ndarray:
        motor_state = self.hardware.read_state()
        return self.model.motor_to_virtual(motor_state.positions_rad)

    def shutdown(self) -> None:
        self.hardware.disconnect()

