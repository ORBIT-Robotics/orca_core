"""Low-level HELIOS head motor hardware wrapper using ORCA's Dynamixel client."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Dict, Iterable, Optional

import numpy as np

from helios_core.scripts.head_config import HeadMotorIDs
from hardware.dynamixel_client import DynamixelClient


@dataclass(frozen=True)
class HeadMotorState:
    positions_rad: np.ndarray
    velocities_rad_s: np.ndarray
    currents_ma: np.ndarray
    timestamp_sec: float


class HeliosHeadHardware:
    """Thin wrapper over DynamixelClient for yaw + coupled upper pair motors."""

    def __init__(self, config: Dict, motor_ids: HeadMotorIDs):
        self._cfg = dict(config or {})
        hw_cfg = dict(self._cfg.get("hardware", {}))

        self.motor_ids = motor_ids
        self.motor_order = ("yaw", "upper_left", "upper_right")
        self._ordered_ids = motor_ids.ordered
        self._id_to_index = {motor_id: idx for idx, motor_id in enumerate(self._ordered_ids)}

        self.port = str(hw_cfg.get("port", "/dev/ttyUSB0"))
        self.baudrate = int(hw_cfg.get("baudrate", 3000000))
        self.control_mode = str(hw_cfg.get("control_mode", "current_based_position"))
        self.startup_current_limit_ma = float(hw_cfg.get("startup_current_limit_ma", 180.0))
        self.runtime_current_limit_ma = float(hw_cfg.get("runtime_current_limit_ma", 260.0))

        safety = dict(self._cfg.get("safety", {}))
        self.max_abs_motor_step_rad = float(safety.get("max_abs_motor_step_rad", 0.08))

        self._dxl_client = None
        self._lock = threading.RLock()

    def _ensure_client_class(self):
        return DynamixelClient

    @property
    def connected(self) -> bool:
        with self._lock:
            return bool(self._dxl_client is not None and self._dxl_client.is_connected)

    def connect(self) -> None:
        with self._lock:
            if self.connected:
                return
            client_cls = self._ensure_client_class()
            self._dxl_client = client_cls(self._ordered_ids, self.port, self.baudrate)
            self._dxl_client.connect()
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
            ids = list(motor_ids) if motor_ids is not None else list(self._ordered_ids)
            self._dxl_client.set_torque_enabled(ids, True)

    def disable_torque(self, motor_ids: Optional[Iterable[int]] = None) -> None:
        with self._lock:
            ids = list(motor_ids) if motor_ids is not None else list(self._ordered_ids)
            self._dxl_client.set_torque_enabled(ids, False)

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
        ids = list(motor_ids) if motor_ids is not None else list(self._ordered_ids)
        with self._lock:
            self._dxl_client.set_operating_mode(ids, mode_value)

    def set_current_limit(self, current_ma: float, motor_ids: Optional[Iterable[int]] = None) -> None:
        ids = list(motor_ids) if motor_ids is not None else list(self._ordered_ids)
        values = np.full(len(ids), float(current_ma), dtype=float)
        with self._lock:
            self._dxl_client.write_desired_current(ids, values)

    def read_state(self) -> HeadMotorState:
        with self._lock:
            pos, vel, cur = self._dxl_client.read_pos_vel_cur()
        return HeadMotorState(
            positions_rad=np.asarray(pos, dtype=float).reshape(3,),
            velocities_rad_s=np.asarray(vel, dtype=float).reshape(3,),
            currents_ma=np.asarray(cur, dtype=float).reshape(3,),
            timestamp_sec=time.monotonic(),
        )

    def get_motor_positions(self, as_dict: bool = False):
        state = self.read_state()
        if not as_dict:
            return state.positions_rad
        return {
            "yaw": float(state.positions_rad[0]),
            "upper_left": float(state.positions_rad[1]),
            "upper_right": float(state.positions_rad[2]),
        }

    def _targets_from_input(self, targets) -> np.ndarray:
        if isinstance(targets, dict):
            arr = np.array(
                [
                    float(targets["yaw"]),
                    float(targets["upper_left"]),
                    float(targets["upper_right"]),
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
        for idx, name in enumerate(("yaw", "upper_left", "upper_right")):
            lim = limits.get(name)
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

        with self._lock:
            self._dxl_client.write_desired_pos(self._ordered_ids, bounded)
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
            self._dxl_client.write_desired_pos(self._ordered_ids, pos)
        return pos
