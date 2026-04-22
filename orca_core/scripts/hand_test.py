"""CLI smoke test for ORCA hand runtime control."""

from __future__ import annotations

import argparse
import time

from orca_core.hand_runtime import OrcaHand
from orca_core.utils.yaml_io import get_profile_model_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Run ORCA hand control smoke test.")
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to ORCA hand model directory.",
    )
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        default=None,
        help="Physical hand side when model_path is omitted.",
    )
    parser.add_argument(
        "--profile",
        choices=("legacy", "v1", "v2_upper", "v2_lower"),
        default=None,
        help="Hand profile used with --side when model_path is omitted.",
    )
    args = parser.parse_args()

    if args.model_path and (args.side or args.profile):
        parser.error("Pass either model_path or --side/--profile, not both.")

    if args.model_path:
        model_path = args.model_path
    elif args.side:
        model_path = get_profile_model_path(args.profile or "v1", args.side)
    else:
        parser.error(
            "Refusing to guess a hand model. Pass model_path or --side left|right with --profile."
        )

    hand = OrcaHand(model_path)
    success, message = hand.connect()
    print((success, message))
    if not success:
        return 1

    hand.enable_torque()
    hand.set_joint_pos(
        {
            "thumb_mcp": 30,
            "thumb_abd": 22,
            "thumb_dip": 40,
            "thumb_pip": 50,
            "index_abd": 0,
            "index_mcp": 75,
            "index_pip": 85,
            "middle_abd": -5,
            "middle_mcp": -15,
            "middle_pip": -12,
            "ring_mcp": 85,
            "ring_pip": 95,
            "pinky_mcp": 90,
            "pinky_pip": 100,
        },
        num_steps=25,
        step_size=0.001,
    )

    time.sleep(2)
    hand.disable_torque()
    hand.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
