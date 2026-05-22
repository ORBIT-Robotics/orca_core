"""Low-level HELIOS head motor hardware wrapper using ORCA's Dynamixel client."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Dict, Iterable, Optional

import numpy as np

from helios_core.utils.head_config import HeadMotorIDs, parse_disabled_motor_axes
from hardware.dynamixel_client import DynamixelClient


@dataclass(frozen=True)
class HeadMotorState:
    positions_rad: np.ndarray
    velocities_rad_s: np.ndarray
    currents_ma: np.ndarray
    timestamp_sec: float


class HeliosHeadHardware:
    """Thin wrapper over DynamixelClient for direct yaw/pitch/roll motors."""

    def __init__(self, config: Dict, motor_ids: HeadMotorIDs):
        self._cfg = dict(config or {})
        hw_cfg = dict(self._cfg.get("hardware", {}))

        self.motor_ids = motor_ids
        self.motor_order = ("yaw", "pitch", "roll")
        self._ordered_ids = motor_ids.ordered
        self._id_to_index = {motor_id: idx for idx, motor_id in enumerate(self._ordered_ids)}
        self.disabled_motor_axes = parse_disabled_motor_axes(self._cfg)
        self._disabled_indices = tuple(
            idx for idx, axis in enumerate(self.motor_order) if axis in self.disabled_motor_axes
        )
        self._active_indices = tuple(
            idx for idx, axis in enumerate(self.motor_order) if axis not in self.disabled_motor_axes
        )
        self._active_ids = tuple(self._ordered_ids[idx] for idx in self._active_indices)
        self._disabled_ids = tuple(self._ordered_ids[idx] for idx in self._disabled_indices)

        self.port = str(hw_cfg.get("port", "/dev/ttyUSB0"))
        self.baudrate = int(hw_cfg.get("baudrate", 3000000))
        self.control_mode = str(hw_cfg.get("control_mode", "current_based_position"))
        self.startup_current_limit_ma = float(hw_cfg.get("startup_current_limit_ma", 180.0))
        self.runtime_current_limit_ma = float(hw_cfg.get("runtime_current_limit_ma", 260.0))

        safety = dict(self._cfg.get("safety", {}))
        self.max_abs_motor_step_rad = float(safety.get("max_abs_motor_step_rad", 0.08))

        self._dxl_client = None
        self._lock = threading.RLock()
        self._cached_positions_rad = self._disabled_axis_defaults(
            hw_cfg.get("disabled_motor_positions_rad", {})
        )
        self._cached_velocities_rad_s = np.zeros(3, dtype=float)
        self._cached_currents_ma = np.zeros(3, dtype=float)

    def _ensure_client_class(self):
        return DynamixelClient

    def _disabled_axis_defaults(self, raw) -> np.ndarray:
        values = np.zeros(3, dtype=float)
        if not isinstance(raw, dict):
            return values
        for idx, axis in enumerate(self.motor_order):
            if axis not in self.disabled_motor_axes:
                continue
            if axis not in raw:
                continue
            value = float(raw[axis])
            if not np.isfinite(value):
                raise ValueError(f"hardware.disabled_motor_positions_rad.{axis} must be finite.")
            values[idx] = value
        return values

    def set_disabled_motor_positions(self, positions) -> None:
        """Set synthetic positions reported for disabled axes."""
        if not self.disabled_motor_axes:
            return
        if isinstance(positions, dict):
            values = positions
        else:
            arr = np.asarray(positions, dtype=float).reshape(3,)
            values = {axis: arr[idx] for idx, axis in enumerate(self.motor_order)}
        with self._lock:
            for idx, axis in enumerate(self.motor_order):
                if axis not in self.disabled_motor_axes or axis not in values:
                    continue
                value = float(values[axis])
                if not np.isfinite(value):
                    raise ValueError(f"Disabled motor position for {axis} must be finite.")
                self._cached_positions_rad[idx] = value

    def _active_motor_ids(self, motor_ids: Optional[Iterable[int]] = None) -> list[int]:
        requested = list(motor_ids) if motor_ids is not None else list(self._active_ids)
        disabled = set(self._disabled_ids)
        return [motor_id for motor_id in requested if motor_id not in disabled]

    def _disabled_motor_ids(self, motor_ids: Optional[Iterable[int]] = None) -> list[int]:
        requested = list(motor_ids) if motor_ids is not None else list(self._ordered_ids)
        disabled = set(self._disabled_ids)
        return [motor_id for motor_id in requested if motor_id in disabled]

    def _disable_ids_best_effort(self, ids: Iterable[int]) -> None:
        ids_list = list(ids)
        if not ids_list or self._dxl_client is None:
            return
        try:
            self._dxl_client.set_torque_enabled(ids_list, False, retries=0)
        except Exception:
            pass

    @property
    def connected(self) -> bool:
        with self._lock:
            return bool(self._dxl_client is not None and self._dxl_client.is_connected)

    def connect(self) -> None:
        with self._lock:
            if self.connected:
                return
            client_cls = self._ensure_client_class()
            self._dxl_client = client_cls(self._active_ids, self.port, self.baudrate)
            self._dxl_client.connect()
            self._disable_ids_best_effort(self._disabled_ids)
            self.set_control_mode(self.control_mode)
            self.set_current_limit(self.startup_current_limit_ma)
            self.enable_torque()

    def disconnect(self) -> None:
        with self._lock:
            if self._dxl_client is None:
                return
            try:
                self.disable_torque()
            except Exception:
                pass
            try:
                self._dxl_client.disconnect()
            finally:
                self._dxl_client = None

    def enable_torque(self, motor_ids: Optional[Iterable[int]] = None) -> None:
        with self._lock:
            ids = self._active_motor_ids(motor_ids)
            if not ids:
                return
            self._dxl_client.set_torque_enabled(ids, True)

    def disable_torque(self, motor_ids: Optional[Iterable[int]] = None) -> None:
        with self._lock:
            ids = self._active_motor_ids(motor_ids)
            if ids:
                self._dxl_client.set_torque_enabled(ids, False)
            self._disable_ids_best_effort(self._disabled_motor_ids(motor_ids))

    def set_control_mode(self, mode: str, motor_ids: Optional[Iterable[int]] = None) -> None:
        mode_map = {
            "current": 0,
            "velocity": 1,
            "position": 3,
            "multi_turn_position": 4,
            "current_based_position": 5,
        }
        mode_value = mode_map.get(str(mode).strip())
        if mode_value is None:
            raise ValueError(f"Unsupported control mode '{mode}'.")
        ids = self._active_motor_ids(motor_ids)
        if not ids:
            return
        with self._lock:
            self._dxl_client.set_operating_mode(ids, mode_value)

    def set_current_limit(self, current_ma: float, motor_ids: Optional[Iterable[int]] = None) -> None:
        ids = self._active_motor_ids(motor_ids)
        if not ids:
            return
        values = np.full(len(ids), float(current_ma), dtype=float)
        with self._lock:
            self._dxl_client.write_desired_current(ids, values)

    def read_state(self) -> HeadMotorState:
        with self._lock:
            pos, vel, cur = self._dxl_client.read_pos_vel_cur()
            pos_active = np.asarray(pos, dtype=float).reshape(-1)
            vel_active = np.asarray(vel, dtype=float).reshape(-1)
            cur_active = np.asarray(cur, dtype=float).reshape(-1)
            if pos_active.size != len(self._active_indices):
                raise RuntimeError(
                    f"Expected {len(self._active_indices)} active head motor positions, "
                    f"got {pos_active.size}."
                )
            if vel_active.size != len(self._active_indices) or cur_active.size != len(self._active_indices):
                raise RuntimeError("Active head motor state arrays have inconsistent sizes.")
            for active_idx, motor_idx in enumerate(self._active_indices):
                self._cached_positions_rad[motor_idx] = pos_active[active_idx]
                self._cached_velocities_rad_s[motor_idx] = vel_active[active_idx]
                self._cached_currents_ma[motor_idx] = cur_active[active_idx]
        return HeadMotorState(
            positions_rad=self._cached_positions_rad.copy(),
            velocities_rad_s=self._cached_velocities_rad_s.copy(),
            currents_ma=self._cached_currents_ma.copy(),
            timestamp_sec=time.monotonic(),
        )

    def get_motor_positions(self, as_dict: bool = False):
        state = self.read_state()
        if not as_dict:
            return state.positions_rad
        return {
            "yaw": float(state.positions_rad[0]),
            "pitch": float(state.positions_rad[1]),
            "roll": float(state.positions_rad[2]),
        }

    def _targets_from_input(self, targets) -> np.ndarray:
        if isinstance(targets, dict):
            arr = np.array(
                [
                    float(targets["yaw"]),
                    float(targets.get("pitch", targets.get("upper_left"))),
                    float(targets.get("roll", targets.get("upper_right"))),
                ],
                dtype=float,
            )
        else:
            arr = np.asarray(targets, dtype=float).reshape(-1)
            if arr.size != 3:
                raise ValueError(f"Expected 3 motor targets, got {arr.size}.")
        if not np.all(np.isfinite(arr)):
            raise ValueError("Motor targets must be finite.")
        return arr

    @staticmethod
    def _apply_limit_clip(targets: np.ndarray, limits: Optional[Dict[str, tuple[float, float]]]) -> np.ndarray:
        if not limits:
            return targets
        clipped = targets.copy()
        for idx, name in enumerate(("yaw", "pitch", "roll")):
            lim = limits.get(name)
            if lim is None and name == "pitch":
                lim = limits.get("upper_left")
            if lim is None and name == "roll":
                lim = limits.get("upper_right")
            if lim is None:
                continue
            lo, hi = float(lim[0]), float(lim[1])
            clipped[idx] = float(np.clip(clipped[idx], lo, hi))
        return clipped

    def command_motor_positions(
        self,
        targets,
        *,
        limits: Optional[Dict[str, tuple[float, float]]] = None,
        max_step_rad: Optional[float] = None,
    ) -> np.ndarray:
        desired = self._targets_from_input(targets)
        current = self.get_motor_positions(as_dict=False)

        max_step = self.max_abs_motor_step_rad if max_step_rad is None else float(max_step_rad)
        delta = desired - current
        if max_step > 0.0:
            delta = np.clip(delta, -max_step, max_step)
        bounded = current + delta
        bounded = self._apply_limit_clip(bounded, limits)
        for idx in self._disabled_indices:
            bounded[idx] = current[idx]

        with self._lock:
            if self._active_indices:
                self._dxl_client.write_desired_pos(
                    self._active_ids,
                    bounded[list(self._active_indices)],
                )
        return bounded

    def command_relative_offsets(
        self,
        offsets_rad,
        *,
        limits: Optional[Dict[str, tuple[float, float]]] = None,
    ) -> np.ndarray:
        offsets = self._targets_from_input(offsets_rad)
        current = self.get_motor_positions(as_dict=False)
        return self.command_motor_positions(current + offsets, limits=limits)

    def hold_current_position(self) -> np.ndarray:
        pos = self.get_motor_positions(as_dict=False)
        with self._lock:
            if self._active_indices:
                self._dxl_client.write_desired_pos(
                    self._active_ids,
                    pos[list(self._active_indices)],
                )
        return pos
