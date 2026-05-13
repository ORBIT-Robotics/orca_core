# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import yaml


def _package_root() -> Path:
    # .../orca_core/orca_core/utils/yaml_io.py -> .../orca_core
    return Path(__file__).resolve().parents[2]


_MODEL_NAME_ALIASES = {
    "left": "orca_hand_ikarus_left",
    "right": "orca_hand_ikarus_right",
    "legacy_left": "orca_hand_legacy_left",
    "legacy_right": "orca_hand_legacy_right",
    "ikarus_left": "orca_hand_ikarus_left",
    "ikarus_right": "orca_hand_ikarus_right",
    "helios_upper_left": "orca_hand_helios_upper_left",
    "helios_upper_right": "orca_hand_helios_upper_right",
    "helios_lower_left": "orca_hand_helios_lower_left",
    "helios_lower_right": "orca_hand_helios_lower_right",
    # Deprecated aliases kept for compatibility.
    "v1": "orca_hand_ikarus_right",
    "dip": "orca_hand_dip",
    "orca_hand_left": "orca_hand_ikarus_left",
    "orca_hand_right": "orca_hand_ikarus_right",
}

_PROFILE_TO_MODEL_BY_SIDE = {
    "legacy": {
        "left": "orca_hand_legacy_left",
        "right": "orca_hand_legacy_right",
    },
    "v1": {
        "left": "orca_hand_ikarus_left",
        "right": "orca_hand_ikarus_right",
    },
    "v2_upper": {
        "left": "orca_hand_helios_upper_left",
        "right": "orca_hand_dip",
    },
    "v2_lower": {
        "left": "orca_hand_helios_lower_left",
        "right": "orca_hand_helios_lower_right",
    },
}


def _resolve_model_input_path(raw_path: Path) -> Path:
    if raw_path.is_absolute():
        return raw_path

    pkg_root = _package_root()
    candidates = []

    if raw_path.parts and raw_path.parts[0] == "orca_core":
        stripped = Path(*raw_path.parts[1:]) if len(raw_path.parts) > 1 else Path()
        candidates.append((pkg_root / stripped).resolve())
        candidates.append((pkg_root.parent / raw_path).resolve())
        candidates.append(raw_path.expanduser().resolve())
        candidates.append((pkg_root / raw_path).resolve())
    else:
        candidates.append(raw_path.expanduser().resolve())
        candidates.append((pkg_root / raw_path).resolve())

    seen = set()
    for candidate in candidates:
        key = str(candidate)
        if key in seen:
            continue
        seen.add(key)
        if candidate.exists():
            return candidate

    if raw_path.parts and raw_path.parts[0] == "orca_core" and len(raw_path.parts) > 1:
        return (pkg_root / Path(*raw_path.parts[1:])).resolve()
    return (pkg_root / raw_path).resolve()


def _normalize_profile(raw_profile: str | None) -> str:
    profile = (raw_profile or "").strip().lower()
    if profile == "v2":
        return "v2_upper"
    if profile in _PROFILE_TO_MODEL_BY_SIDE:
        return profile
    if not profile:
        return "v1"
    raise ValueError(
        f"Unsupported hand profile '{raw_profile}'. Supported: {', '.join(sorted(_PROFILE_TO_MODEL_BY_SIDE))}."
    )


def get_profile_model_path(profile: str, side: str) -> str:
    normalized_profile = _normalize_profile(profile)
    normalized_side = (side or "").strip().lower()
    if normalized_side not in {"left", "right"}:
        raise ValueError("side must be 'left' or 'right'.")
    model_dir_name = _PROFILE_TO_MODEL_BY_SIDE[normalized_profile][normalized_side]
    return str((_package_root() / "models" / model_dir_name).resolve())


def get_named_model_path(name: str) -> str:
    normalized = (name or "").strip().lower()
    models_dir = _package_root() / "models"
    model_name = _MODEL_NAME_ALIASES.get(normalized, normalized)
    return str((models_dir / model_name).resolve())


def get_model_path(model_path=None):
    if model_path is None or model_path == "models":
        explicit_model = os.getenv("ORCA_RIGHT_MODEL_PATH", "").strip()
        if explicit_model:
            resolved_path = _resolve_model_input_path(Path(explicit_model))
        else:
            resolved_path = Path(get_named_model_path("right"))
    elif isinstance(model_path, str):
        normalized = model_path.strip().lower()
        if normalized in _MODEL_NAME_ALIASES:
            resolved_path = Path(get_named_model_path(normalized))
        else:
            resolved_path = _resolve_model_input_path(Path(model_path))
    else:
        resolved_path = _resolve_model_input_path(Path(model_path))

    if not resolved_path.exists():
        raise FileNotFoundError(f"\033[1;35mModel directory not found: {resolved_path}\033[0m")

    config_file = resolved_path / "config.yaml"
    if not config_file.exists():
        raise FileNotFoundError(
            f"\033[1;35mconfig.yaml not found in {resolved_path}. "
            "Did you specify the correct model directory?\033[0m"
        )

    print(f"Using model path: \033[1;32m{resolved_path}\033[0m")
    return str(resolved_path)


def update_yaml(file_path, key, value):
    """Reads a YAML file, updates a specific key, and writes it back."""
    if isinstance(value, np.ndarray):
        value = value.tolist()

    if isinstance(value, dict):
        value = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in value.items()}

    try:
        with open(file_path, "r+", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}

            new_data = {key: value}
            for existing_key, existing_value in data.items():
                if existing_key != key:
                    new_data[existing_key] = existing_value

            file.seek(0)
            yaml.dump(new_data, file, default_flow_style=False, sort_keys=False)
            file.truncate()
    except FileNotFoundError:
        with open(file_path, "w", encoding="utf-8") as file:
            yaml.dump({key: value}, file, default_flow_style=False, sort_keys=False)


def read_yaml(file_path):
    """Reads a YAML file and returns its content.

    Empty or missing files are treated as empty mappings.
    """
    try:
        with open(file_path, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file)
            return {} if data is None else data
    except FileNotFoundError:
        return {}
