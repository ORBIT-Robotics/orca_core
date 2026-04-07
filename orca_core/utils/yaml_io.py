# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import os
from pathlib import Path

import numpy as np
import yaml


def _package_root() -> Path:
    # .../orca_core/orca_core/utils/yaml_io.py -> .../orca_core
    return Path(__file__).resolve().parents[2]


def _resolve_model_input_path(raw_path: Path) -> Path:
    if raw_path.is_absolute():
        return raw_path

    pkg_root = _package_root()
    candidates = []

    # Support repo-root style paths like "orca_core/models/..." even when
    # running from the nested orca_core package root.
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


def _normalize_right_hand_profile(raw_profile: str | None) -> str:
    profile = (raw_profile or "").strip().lower()
    if not profile:
        return "v1"
    if profile == "right":
        return "v1"
    if profile in {"v1", "dip"}:
        return profile
    return "v1"


def _right_hand_model_dir_for_profile(profile: str) -> str:
    normalized = _normalize_right_hand_profile(profile)
    return "orca_hand_dip" if normalized == "dip" else "orca_hand_right"


def get_named_model_path(name: str) -> str:
    normalized = (name or "").strip().lower()
    models_dir = _package_root() / "models"

    if normalized == "left":
        return str((models_dir / "orca_hand_left").resolve())
    if normalized in {"right", "v1", "dip"}:
        model_dir = _right_hand_model_dir_for_profile(normalized)
        return str((models_dir / model_dir).resolve())

    return get_model_path(name)


def get_model_path(model_path=None):
    if model_path is None or model_path == "models":
        models_dir = _package_root() / "models"
        if not models_dir.exists():
            raise FileNotFoundError(
                "\033[1;35mModels directory not found. Did you download them? "
                "If not find them at https://www.orcahand.com/downloads\033[0m"
            )
        model_dirs = sorted(
            d for d in models_dir.iterdir() if d.is_dir() and d.name.startswith("orca_hand")
        )
        if not model_dirs:
            raise FileNotFoundError(
                "\033[1;35mNo ORCA hand model folders found. Did you download them? "
                "If not find them at https://www.orcahand.com/downloads\033[0m"
            )
        # Temporary profile bridge:
        # The broader launch stack is transitioning away from ambiguous "right"
        # naming toward explicit hand profiles:
        # - profile "v1" means the right-slot hand should use orca_hand_right
        # - profile "dip" means the right-slot hand should use orca_hand_dip
        # The left-slot hand remains orca_hand_left in both cases.
        #
        # ORCA core still has several single-hand entry points that instantiate
        # OrcaHand() without an explicit model path. Until those call sites are
        # all made fully explicit, resolve the default hand model from the same
        # profile signal used by the top-level launch scripts so the runtime does
        # not silently fall back to the first alphabetically sorted hand model.
        #
        # This is intentionally temporary glue. The long-term direction should
        # be to pass concrete model paths everywhere, with profile selection
        # happening one layer above ORCA core rather than inside this helper.
        right_hand_profile = _normalize_right_hand_profile(
            os.getenv("ORCA_HAND_PROFILE") or os.getenv("ORCA_RIGHT_HAND_PROFILE")
        )
        preferred_model_dir = models_dir / _right_hand_model_dir_for_profile(right_hand_profile)
        resolved_path = preferred_model_dir if preferred_model_dir.exists() else model_dirs[0]
    elif isinstance(model_path, str) and model_path.strip().lower() in {"left", "right", "v1", "dip"}:
        resolved_path = Path(get_named_model_path(model_path))
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
    """Reads a YAML file and returns its content."""
    try:
        with open(file_path, "r", encoding="utf-8") as file:
            return yaml.safe_load(file) or None
    except FileNotFoundError:
        return {}
