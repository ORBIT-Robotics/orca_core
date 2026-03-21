"""ZED X One IMU wrapper for HELIOS head calibration."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import time
import warnings
from typing import Any, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R


@dataclass(frozen=True)
class HeadImuSample:
    timestamp_sec: float
    quaternion_xyzw: np.ndarray
    angular_velocity_rad_s: np.ndarray
    linear_acceleration_mps2: np.ndarray
    relative_yaw_pitch_roll_rad: np.ndarray


class ZedXHeadImu:
    """
    Best-effort CameraOne IMU reader for calibration workflows.

    The wrapper prefers SDK orientation when available and falls back to
    integrating gyro data over short windows if orientation is unavailable.
    """

    def __init__(
        self,
        serial_number: int,
        *,
        max_sample_age_sec: float = 0.25,
        sample_attempts: int = 25,
        sample_sleep_sec: float = 0.01,
    ):
        self.serial_number = int(serial_number)
        self.max_sample_age_sec = float(max_sample_age_sec)
        self.sample_attempts = int(sample_attempts)
        self.sample_sleep_sec = float(sample_sleep_sec)

        self.sl = None
        self._camera = None
        self._sensors_data = None

        self._neutral_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._integrated_rot = R.identity()
        self._last_integrate_time_sec: Optional[float] = None

    @property
    def connected(self) -> bool:
        return self._camera is not None

    def _load_sdk(self):
        if self.sl is None:
            self.sl = importlib.import_module("pyzed.sl")
        return self.sl

    def _new_sensors_data(self):
        for name in ("SensorsData", "SensorsDataOne"):
            cls = getattr(self.sl, name, None)
            if cls is None:
                continue
            try:
                return cls()
            except Exception:
                continue
        return None

    def open(self) -> None:
        sl = self._load_sdk()
        camera = sl.CameraOne()
        init_params = sl.InitParametersOne()

        if hasattr(init_params, "set_from_serial_number"):
            init_params.set_from_serial_number(self.serial_number)
        elif hasattr(init_params, "input") and hasattr(init_params.input, "set_from_serial_number"):
            init_params.input.set_from_serial_number(self.serial_number)
        else:
            raise RuntimeError("ZED SDK InitParametersOne does not expose serial selection method")

        status = camera.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(
                f"Failed to open ZED X One serial {self.serial_number}: {status}"
            )

        self._camera = camera
        self._sensors_data = self._new_sensors_data()
        self._integrated_rot = R.identity()
        self._last_integrate_time_sec = None

    def close(self) -> None:
        if self._camera is not None:
            try:
                self._camera.close()
            except Exception:
                pass
        self._camera = None

    @staticmethod
    def _to_array(value: Any, expected_size: int) -> Optional[np.ndarray]:
        if value is None:
            return None
        try:
            arr = np.asarray(value, dtype=float).reshape(-1)
        except Exception:
            return None
        if arr.size != expected_size or not np.all(np.isfinite(arr)):
            return None
        return arr

    @staticmethod
    def _call_no_args(value: Any) -> Any:
        if callable(value):
            try:
                return value()
            except TypeError:
                return value
        return value

    def _resolve_chain(self, obj: Any, chain: tuple[str, ...]) -> Any:
        value = obj
        for name in chain:
            if value is None or not hasattr(value, name):
                return None
            value = getattr(value, name)
            value = self._call_no_args(value)
        return value

    def _extract_vector(self, obj: Any, chains: list[tuple[str, ...]], size: int) -> Optional[np.ndarray]:
        for chain in chains:
            value = self._resolve_chain(obj, chain)
            arr = self._to_array(value, size)
            if arr is not None:
                return arr
        return None

    def _extract_timestamp_sec(self, imu_obj: Any, fallback_sec: float) -> float:
        ts_candidates = [
            self._resolve_chain(imu_obj, ("timestamp",)),
            self._resolve_chain(imu_obj, ("get_timestamp",)),
            self._resolve_chain(imu_obj, ("get_timestamp", "get_nanoseconds")),
            self._resolve_chain(imu_obj, ("get_timestamp", "get_microseconds")),
            self._resolve_chain(imu_obj, ("get_timestamp", "get_milliseconds")),
            self._resolve_chain(imu_obj, ("get_timestamp", "get_seconds")),
        ]
        for ts in ts_candidates:
            if ts is None:
                continue
            if isinstance(ts, (int, float)):
                tsf = float(ts)
                # Heuristic unit handling for raw timestamp values.
                if tsf > 1e15:
                    return tsf * 1e-9
                if tsf > 1e12:
                    return tsf * 1e-6
                if tsf > 1e9:
                    return tsf * 1e-3
                return tsf
        return fallback_sec

    def _extract_sdk_orientation(self, imu_obj: Any) -> Optional[np.ndarray]:
        quat = self._extract_vector(
            imu_obj,
            chains=[
                ("get_pose", "get_orientation", "get"),
                ("get_pose_data", "get_orientation", "get"),
                ("pose", "get_orientation", "get"),
                ("orientation", "get"),
                ("orientation",),
                ("quaternion",),
            ],
            size=4,
        )
        if quat is None:
            return None
        norm = float(np.linalg.norm(quat))
        if np.isclose(norm, 0.0):
            return None
        return quat / norm

    def _integrate_orientation(self, gyro_rad_s: np.ndarray, now_sec: float) -> np.ndarray:
        if self._last_integrate_time_sec is None:
            self._last_integrate_time_sec = now_sec
            return self._integrated_rot.as_quat().reshape(4,)

        dt = max(0.0, min(now_sec - self._last_integrate_time_sec, 0.25))
        self._last_integrate_time_sec = now_sec
        if dt <= 0.0 or not np.all(np.isfinite(gyro_rad_s)):
            return self._integrated_rot.as_quat().reshape(4,)

        rot_inc = R.from_rotvec(np.asarray(gyro_rad_s, dtype=float).reshape(3,) * dt)
        self._integrated_rot = self._integrated_rot * rot_inc
        quat = self._integrated_rot.as_quat().reshape(4,)
        norm = float(np.linalg.norm(quat))
        if np.isclose(norm, 0.0):
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        return quat / norm

    @staticmethod
    def _relative_ypr(current_quat: np.ndarray, neutral_quat: np.ndarray) -> np.ndarray:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            rel = R.from_quat(neutral_quat).inv() * R.from_quat(current_quat)
            # ZYX => [yaw, pitch, roll]
            ypr = rel.as_euler("zyx", degrees=False)
        return np.asarray(ypr, dtype=float).reshape(3,)

    def _read_single(self) -> Optional[HeadImuSample]:
        if self._camera is None:
            raise RuntimeError("IMU camera is not connected. Call open() first.")

        now_sec = time.monotonic()
        grab_status = self._camera.grab()
        if grab_status != self.sl.ERROR_CODE.SUCCESS:
            return None

        imu_obj = None
        if self._sensors_data is not None:
            try:
                if hasattr(self.sl, "TIME_REFERENCE") and hasattr(self.sl.TIME_REFERENCE, "CURRENT"):
                    status = self._camera.get_sensors_data(self._sensors_data, self.sl.TIME_REFERENCE.CURRENT)
                else:
                    status = self._camera.get_sensors_data(self._sensors_data)
                if status == self.sl.ERROR_CODE.SUCCESS:
                    imu_obj = self._resolve_chain(self._sensors_data, ("get_imu_data",))
                    if imu_obj is None:
                        imu_obj = self._resolve_chain(self._sensors_data, ("imu_data",))
            except Exception:
                imu_obj = None

        gyro = np.zeros(3, dtype=float)
        accel = np.zeros(3, dtype=float)
        timestamp_sec = now_sec

        if imu_obj is not None:
            gyro_val = self._extract_vector(
                imu_obj,
                chains=[
                    ("get_angular_velocity",),
                    ("get_angular_velocity_calibrated",),
                    ("angular_velocity",),
                ],
                size=3,
            )
            if gyro_val is not None:
                gyro = gyro_val

            accel_val = self._extract_vector(
                imu_obj,
                chains=[
                    ("get_linear_acceleration",),
                    ("linear_acceleration",),
                ],
                size=3,
            )
            if accel_val is not None:
                accel = accel_val

            timestamp_sec = self._extract_timestamp_sec(imu_obj, fallback_sec=now_sec)

        sdk_quat = self._extract_sdk_orientation(imu_obj) if imu_obj is not None else None
        if sdk_quat is None:
            quat = self._integrate_orientation(gyro, now_sec)
        else:
            quat = sdk_quat
            self._integrated_rot = R.from_quat(quat)
            self._last_integrate_time_sec = now_sec

        if not np.all(np.isfinite(quat)):
            return None

        age = abs(now_sec - timestamp_sec)
        if np.isfinite(age) and age > self.max_sample_age_sec:
            return None

        rel_ypr = self._relative_ypr(quat, self._neutral_quat)
        return HeadImuSample(
            timestamp_sec=float(timestamp_sec),
            quaternion_xyzw=np.asarray(quat, dtype=float).reshape(4,),
            angular_velocity_rad_s=np.asarray(gyro, dtype=float).reshape(3,),
            linear_acceleration_mps2=np.asarray(accel, dtype=float).reshape(3,),
            relative_yaw_pitch_roll_rad=rel_ypr,
        )

    def read(self) -> HeadImuSample:
        for _ in range(max(1, self.sample_attempts)):
            sample = self._read_single()
            if sample is not None:
                return sample
            time.sleep(max(0.0, self.sample_sleep_sec))
        raise RuntimeError("Failed to read a valid ZED X One IMU sample")

    def capture_neutral(self) -> HeadImuSample:
        sample = self.read()
        quat = np.asarray(sample.quaternion_xyzw, dtype=float).reshape(4,)
        norm = float(np.linalg.norm(quat))
        if np.isclose(norm, 0.0):
            raise RuntimeError("Cannot set IMU neutral from zero-length quaternion")
        self._neutral_quat = quat / norm
        return sample

    def get_neutral_quat(self) -> np.ndarray:
        return self._neutral_quat.copy()

    def set_neutral_quat(self, quat_xyzw: np.ndarray) -> None:
        quat = np.asarray(quat_xyzw, dtype=float).reshape(4,)
        norm = float(np.linalg.norm(quat))
        if np.isclose(norm, 0.0):
            raise ValueError("Neutral quaternion cannot be zero-length.")
        self._neutral_quat = quat / norm
