from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
ORCA_CORE_ROOT = REPO_ROOT / "orca_core"
if str(ORCA_CORE_ROOT) not in sys.path:
    sys.path.insert(0, str(ORCA_CORE_ROOT))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from orca_core.hand_runtime import OrcaHand  # noqa: E402
from orca_core.scripts._bus_preflight import preflight_motor_role  # noqa: E402
from orca_core.scripts._dynamixel_preflight import (  # noqa: E402
    _role_motor_type,
    check_dynamixel_role_status,
    print_dynamixel_report_errors,
)
from orca_core.utils.yaml_io import read_yaml  # noqa: E402
from openteach.helpers.orca_hand_roles import OrcaHandRoleSpec, load_orca_hand_role  # noqa: E402


def add_role_argument(parser) -> None:
    parser.add_argument(
        "--role",
        required=True,
        help="ORCA hand role from configs/<robot>.yaml.",
    )


def resolve_role(role: str) -> OrcaHandRoleSpec:
    return load_orca_hand_role(
        role,
        repo_root=REPO_ROOT,
        validate_paths=True,
        require_enabled=True,
        require_port=True,
    )


def print_role_summary(spec: OrcaHandRoleSpec) -> None:
    config = read_yaml(str(spec.model_path / "config.yaml"))
    motor_type = str(config.get("motor_type", "dynamixel"))

    print(f"Using ORCA role: {spec.role}")
    print(f"Using model path: {spec.model_path}")
    print(f"Using calibration path: {spec.calibration_path}")
    print(f"Using motor type: {motor_type}")
    print(f"Using serial port: {spec.port}")
    print(f"Using baudrate: {spec.baudrate}")


def create_hand(spec: OrcaHandRoleSpec) -> OrcaHand:
    hand = OrcaHand(str(spec.model_path))
    hand.port = spec.port
    hand.baudrate = spec.baudrate
    return hand

def connect_hand_with_bus_preflight(
    spec: OrcaHandRoleSpec,
    hand: OrcaHand,
) -> tuple[bool, str]:
    motor_type = _role_motor_type(spec)
    if not preflight_motor_role(spec):
        return (
            False,
            f"ORCA hand bus preflight failed for role {spec.role}; "
            "see the motor status, port, and baudrate details above.",
        )

    success, message = hand.connect()
    if success or motor_type != "dynamixel":
        return success, message

    print(
        f"[{spec.role}] ERROR: connection failed after ORCA hand bus preflight: {message}",
        file=sys.stderr,
    )
    report = check_dynamixel_role_status(spec)
    if not report.ok:
        print_dynamixel_report_errors(report)
    return success, message


def connect_hand_with_dynamixel_preflight(
    spec: OrcaHandRoleSpec,
    hand: OrcaHand,
) -> tuple[bool, str]:
    return connect_hand_with_bus_preflight(spec, hand)

