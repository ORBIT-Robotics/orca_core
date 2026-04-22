"""CLI for moving ORCA hand to neutral position."""

from __future__ import annotations

import argparse

from orca_core.hand_runtime import OrcaHand
from orca_core.utils.yaml_io import get_profile_model_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Move ORCA hand to neutral position.")
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

    hand = OrcaHand(model_path=model_path)
    success, message = hand.connect()
    if not success:
        print(f"Failed to connect: {message}")
        return 1

    hand.enable_torque()
    hand.set_neutral_position()
    hand.disable_torque()
    hand.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
