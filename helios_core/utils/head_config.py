"""Config and YAML utilities for HELIOS head runtime/calibration."""

from __future__ import annotations

import copy
from dataclasses import dataclass
from datetime import datetime, timezone
import os
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


DEFAULT_HEAD_ROLE = "helios_head"
DEFAULT_HEAD_CONFIG_PATH = "orca_core/models/helios_head/config.yaml"
_HEAD_STYLE = "helios_core"
_EXACT_PORT_PREFIXES = (
    "/dev/serial/by-id/",
    "/dev/orca/",
)


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


@dataclass(frozen=True)
class HeliosHeadRoleSpec:
    role: str
    enabled: bool
    style: str
    model_path: Path
    config_path: Path
    port: str
    baudrate: int
    motor_ids: HeadMotorIDs

    @property
    def calibration_path(self) -> Path:
        return self.model_path / "calibration.yaml"


def resolve_repo_root(repo_root: str | Path | None = None) -> Path:
    if repo_root is not None:
        return Path(repo_root).expanduser().resolve()

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


def _repo_root() -> Path:
    return resolve_repo_root()


def resolve_repo_path(path_str: str | Path) -> Path:
    path = Path(path_str)
    if path.is_absolute():
        return path

    repo_root = _repo_root()
    local_pkg_root = Path(__file__).resolve().parents[1]
    candidates = []
    if path.parts and path.parts[0] == "orca_core" and len(path.parts) > 1:
        # Support repo-root style "orca_core/..." paths even when executing from
        # inside the standalone orca_core package checkout.
        stripped = Path(*path.parts[1:])
        candidates.append((repo_root / stripped).resolve())
        candidates.append((local_pkg_root / stripped).resolve())
        candidates.append((repo_root / path).resolve())
        candidates.append((local_pkg_root / path).resolve())
    else:
        candidates.append((repo_root / path).resolve())
        candidates.append((local_pkg_root / path).resolve())

    seen = set()
    for candidate in candidates:
        key = str(candidate)
        if key in seen:
            continue
        seen.add(key)
        if candidate.exists():
            return candidate

    if path.parts and path.parts[0] == "orca_core" and len(path.parts) > 1:
        return (local_pkg_root / Path(*path.parts[1:])).resolve()
    return (local_pkg_root / path).resolve()


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


def _load_head_config_path(path: str | Path) -> tuple[Dict[str, Any], Path]:
    cfg_path = resolve_repo_path(path)
    if cfg_path.is_dir():
        cfg_path = cfg_path / "config.yaml"
    cfg = read_yaml(cfg_path)
    if not cfg:
        raise FileNotFoundError(f"HELIOS head config not found or empty: {cfg_path}")
    return cfg, cfg_path


def load_head_config(
    path: Optional[str] = None,
    *,
    role: Optional[str] = DEFAULT_HEAD_ROLE,
) -> tuple[Dict[str, Any], Path]:
    """Load HELIOS head config by explicit path or hardware role.

    An explicit path preserves the legacy behavior. Without a path, the default
    role is resolved from configs/helios.yaml and overlays the
    model config's hardware fields.
    """
    if path is not None and str(path).strip():
        return _load_head_config_path(path)

    role_name = str(role or "").strip()
    if role_name:
        cfg, cfg_path, _ = load_head_role_config(role_name)
        return cfg, cfg_path

    return _load_head_config_path(DEFAULT_HEAD_CONFIG_PATH)


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


def _helios_robot_config_path(repo_root: Path) -> Path:
    return repo_root / "configs" / "helios.yaml"


def _load_yaml_direct(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Config not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Config must be a mapping: {path}")
    return data


def _resolve_path(repo_root: Path, value: str | Path | None, *, label: str) -> Path:
    if value is None or str(value).strip() == "":
        raise ValueError(f"Missing required HELIOS head path for {label}.")
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = (repo_root / candidate).resolve()
    else:
        candidate = candidate.resolve()
    return candidate


def _load_head_role_blocks(repo_root: Path) -> Dict[str, Any]:
    doc = _load_yaml_direct(_helios_robot_config_path(repo_root))
    roles = (doc.get("orca_hardware") or {}).get("head_roles")
    if not isinstance(roles, dict):
        raise ValueError("configs/helios.yaml must define orca_hardware.head_roles.")
    return roles


def _parse_role_motor_ids(value: Any, *, label: str) -> HeadMotorIDs:
    if not isinstance(value, dict):
        raise ValueError(f"{label}.motor_ids must be a mapping with yaw/upper_left/upper_right.")
    return parse_motor_ids({"hardware": {"motor_ids": value}})


def _head_role_spec_from_block(
    role: str,
    block: Dict[str, Any],
    *,
    repo_root: Path,
) -> HeliosHeadRoleSpec:
    label = f"head_roles.{role}"
    style = str(block.get("style") or "").strip()
    if style != _HEAD_STYLE:
        raise ValueError(f"{label}.style must be '{_HEAD_STYLE}', got {style!r}.")

    baudrate = block.get("baudrate", 3000000)
    try:
        baudrate_int = int(baudrate)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}.baudrate must be an integer, got {baudrate!r}.") from exc

    model_path = _resolve_path(repo_root, block.get("model_path"), label=f"{label}.model_path")
    raw_config_path = block.get("config_path")
    config_path = (
        _resolve_path(repo_root, raw_config_path, label=f"{label}.config_path")
        if raw_config_path
        else model_path / "config.yaml"
    )

    return HeliosHeadRoleSpec(
        role=role,
        enabled=bool(block.get("enabled", False)),
        style=style,
        model_path=model_path,
        config_path=config_path.resolve(),
        port=str(block.get("port") or "").strip(),
        baudrate=baudrate_int,
        motor_ids=_parse_role_motor_ids(block.get("motor_ids"), label=label),
    )


def load_head_role(
    role: str,
    *,
    repo_root: str | Path | None = None,
    validate_paths: bool = True,
    require_enabled: bool = True,
    require_port: bool = True,
) -> HeliosHeadRoleSpec:
    role_name = str(role or "").strip()
    if not role_name:
        raise ValueError("HELIOS head role is empty.")

    resolved_repo_root = resolve_repo_root(repo_root)
    roles = _load_head_role_blocks(resolved_repo_root)
    block = roles.get(role_name)
    if not isinstance(block, dict):
        supported = ", ".join(sorted(str(name) for name in roles))
        raise ValueError(f"Unknown HELIOS head role '{role_name}'. Supported roles: {supported}.")

    spec = _head_role_spec_from_block(role_name, block, repo_root=resolved_repo_root)
    validate_head_role_spec(
        spec,
        validate_paths=validate_paths,
        require_enabled=require_enabled,
        require_port=require_port,
    )
    return spec


def validate_head_role_spec(
    spec: HeliosHeadRoleSpec,
    *,
    validate_paths: bool = True,
    require_enabled: bool = True,
    require_port: bool = True,
) -> None:
    if require_enabled and not spec.enabled:
        raise ValueError(
            f"HELIOS head role '{spec.role}' is disabled. Enable it and set an exact port "
            "in configs/helios.yaml before using it for hardware."
        )

    if validate_paths:
        missing = []
        for label, path in (
            ("model_path", spec.model_path),
            ("model config", spec.config_path),
        ):
            if not path.exists():
                missing.append(f"{label}: {path}")
        if missing:
            raise FileNotFoundError(
                f"HELIOS head role '{spec.role}' has missing assets: {'; '.join(missing)}"
            )

    if require_port:
        _validate_exact_port(spec)


def _validate_exact_port(spec: HeliosHeadRoleSpec) -> None:
    if not spec.port:
        raise ValueError(
            f"HELIOS head role '{spec.role}' has no serial port. Set an exact "
            "port in configs/helios.yaml."
        )
    if any(char in spec.port for char in "*?["):
        raise ValueError(
            f"HELIOS head role '{spec.role}' uses non-exact serial port '{spec.port}'. "
            "Use an exact /dev/serial/by-id/... path or /dev/orca/... alias."
        )
    if not spec.port.startswith(_EXACT_PORT_PREFIXES):
        prefixes = " or ".join(_EXACT_PORT_PREFIXES)
        raise ValueError(
            f"HELIOS head role '{spec.role}' uses serial port '{spec.port}'. "
            f"Use {prefixes} so the mapping is persistent."
        )


def load_head_role_config(
    role: str = DEFAULT_HEAD_ROLE,
    *,
    repo_root: str | Path | None = None,
    validate_paths: bool = True,
    require_enabled: bool = True,
    require_port: bool = True,
) -> tuple[Dict[str, Any], Path, HeliosHeadRoleSpec]:
    spec = load_head_role(
        role,
        repo_root=repo_root,
        validate_paths=validate_paths,
        require_enabled=require_enabled,
        require_port=require_port,
    )
    cfg, cfg_path = _load_head_config_path(spec.config_path)
    merged = copy.deepcopy(cfg)
    hw_cfg = dict(merged.get("hardware", {}))
    hw_cfg["style"] = spec.style
    hw_cfg["port"] = spec.port
    hw_cfg["baudrate"] = spec.baudrate
    hw_cfg["motor_ids"] = spec.motor_ids.as_dict
    merged["hardware"] = hw_cfg

    return merged, cfg_path, spec


def calibration_output_path(config: Dict[str, Any], config_path: Path) -> Path:
    calib_cfg = config.get("calibration", {})
    out = calib_cfg.get(
        "output_path",
        "orca_core/models/helios_head/calibration.yaml",
    )
    return resolve_repo_path(out)


def calibration_log_dir(config: Dict[str, Any], config_path: Path) -> Path:
    calib_cfg = config.get("calibration", {})
    raw = calib_cfg.get("log_dir", "logs/helios_head_calibration")
    return resolve_repo_path(raw)
