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
from openteach.helpers.orca_hand_roles import OrcaHandRoleSpec, load_orca_hand_role  # noqa: E402


def add_role_argument(parser) -> None:
    parser.add_argument(
        "--role",
        required=True,
        help="ORCA hand role from configs/hardware/orca_hands.yaml.",
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
    print(f"Using ORCA role: {spec.role}")
    print(f"Using model path: {spec.model_path}")
    print(f"Using calibration path: {spec.calibration_path}")
    print(f"Using serial port: {spec.port}")
    print(f"Using baudrate: {spec.baudrate}")


def create_hand(spec: OrcaHandRoleSpec) -> OrcaHand:
    hand = OrcaHand(str(spec.model_path))
    hand.port = spec.port
    hand.baudrate = spec.baudrate
    return hand
