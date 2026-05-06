#!/usr/bin/env python3
"""Jetson-side HELIOS head runtime wrapper (ROS2 hardware interface package)."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

DEFAULT_HEAD_ROLE = "helios_head"
ORCA_CORE_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = Path(os.getenv("ORBIT__TELEOP_ROOT") or os.getenv("REPO_ROOT") or ORCA_CORE_ROOT.parent).resolve()
if str(ORCA_CORE_ROOT) not in sys.path:
    sys.path.insert(0, str(ORCA_CORE_ROOT))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HELIOS head runtime node")
    parser.add_argument(
        "--config",
        default=None,
        help="Path to HELIOS head runtime config YAML. Overrides --role when set.",
    )
    parser.add_argument(
        "--role",
        default=DEFAULT_HEAD_ROLE,
        help="HELIOS head role from configs/hardware/orca_hands.yaml.",
    )
    return parser.parse_args()


def _resolve_repo_path(path_str: str) -> Path:
    from helios_core.utils.head_config import resolve_repo_path

    path = Path(path_str)
    if path.is_absolute():
        return path
    return resolve_repo_path(path)


def _load_runtime_node():
    try:
        from helios_head_hardware_interface.helios_head_hardware_node import (
            HeliosHeadHardwareNode,
        )
    except ModuleNotFoundError as exc:
        if exc.name != "helios_head_hardware_interface":
            raise
        local_ros_pkg = REPO_ROOT / "ros2_ws" / "src" / "helios_head_hardware_interface"
        if str(local_ros_pkg) not in sys.path:
            sys.path.insert(0, str(local_ros_pkg))
        from helios_head_hardware_interface.helios_head_hardware_node import (
            HeliosHeadHardwareNode,
        )
    return HeliosHeadHardwareNode


def main() -> None:
    args = _parse_args()
    config_path = None
    if args.config:
        config_path = _resolve_repo_path(str(args.config))
        if not config_path.exists():
            raise FileNotFoundError(
                f"Missing HELIOS head config YAML: {config_path}. "
                "Provide --config with a valid file path."
            )
    HeliosHeadHardwareNode = _load_runtime_node()
    import rclpy

    rclpy.init()
    node = HeliosHeadHardwareNode(
        config_path_override=str(config_path) if config_path else None,
        role_override=args.role,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
