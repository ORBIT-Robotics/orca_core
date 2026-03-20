"""Config and YAML utilities for HELIOS head runtime/calibration."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import os
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


@dataclass(frozen=True)
class HeadMotorIDs:
    yaw: int
    upper_left: int
    upper_right: int

    @property
    def ordered(self) -> tuple[int, int, int]:
        return (self.yaw, self.upper_left, self.upper_right)

    @property
    def as_dict(self) -> Dict[str, int]:
        return {
            "yaw": self.yaw,
            "upper_left": self.upper_left,
            "upper_right": self.upper_right,
        }


def _repo_root() -> Path:
    for env_key in ("ORBIT__TELEOP_ROOT", "REPO_ROOT"):
        env_root = os.getenv(env_key)
        if env_root:
            path = Path(env_root).expanduser().resolve()
            if path.exists():
                return path

    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "scripts").exists() and (parent / "ros2_ws").exists():
            return parent

    # Fallback for unusual local execution contexts.
    return Path.cwd().resolve()


def resolve_repo_path(path_str: str | Path) -> Path:
    path = Path(path_str)
    if path.is_absolute():
        return path
    return (_repo_root() / path).resolve()


def read_yaml(path: str | Path) -> Dict[str, Any]:
    yaml_path = resolve_repo_path(path)
    if not yaml_path.exists():
        return {}
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected YAML mapping in {yaml_path}, got {type(data).__name__}")
    return data


def write_yaml(path: str | Path, data: Dict[str, Any]) -> None:
    yaml_path = resolve_repo_path(path)
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_head_config(path: Optional[str] = None) -> tuple[Dict[str, Any], Path]:
    default_path = "orca_core/orca_core/helios_head/models/helios_head_v1/config.yaml"
    cfg_path = resolve_repo_path(path or default_path)
    if cfg_path.is_dir():
        cfg_path = cfg_path / "config.yaml"
    cfg = read_yaml(cfg_path)
    if not cfg:
        raise FileNotFoundError(f"HELIOS head config not found or empty: {cfg_path}")
    return cfg, cfg_path


def parse_motor_ids(config: Dict[str, Any]) -> HeadMotorIDs:
    hw = config.get("hardware", {})
    motor_ids = hw.get("motor_ids", {})
    try:
        yaw = motor_ids.get("yaw")
        upper_left = motor_ids.get("upper_left")
        upper_right = motor_ids.get("upper_right")
    except AttributeError as exc:
        raise ValueError("hardware.motor_ids must be a mapping with yaw/upper_left/upper_right") from exc

    missing = [
        name
        for name, value in {
            "yaw": yaw,
            "upper_left": upper_left,
            "upper_right": upper_right,
        }.items()
        if value is None
    ]
    if missing:
        raise ValueError(
            "Missing HELIOS head motor IDs in config: "
            + ", ".join(missing)
            + ". Set hardware.motor_ids.{yaw,upper_left,upper_right}."
        )

    ids = HeadMotorIDs(int(yaw), int(upper_left), int(upper_right))
    if len(set(ids.ordered)) != 3:
        raise ValueError(f"Head motor IDs must be unique; got {ids.ordered}")
    return ids


def calibration_output_path(config: Dict[str, Any], config_path: Path) -> Path:
    calib_cfg = config.get("calibration", {})
    out = calib_cfg.get(
        "output_path",
        "orca_core/orca_core/helios_head/models/helios_head_v1/calibration.yaml",
    )
    return resolve_repo_path(out)


def calibration_log_dir(config: Dict[str, Any], config_path: Path) -> Path:
    calib_cfg = config.get("calibration", {})
    raw = calib_cfg.get("log_dir", "logs/helios_head_calibration")
    return resolve_repo_path(raw)
