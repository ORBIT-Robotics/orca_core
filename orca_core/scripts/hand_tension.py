"""CLI for ORCA hand tensioning."""

from __future__ import annotations

import argparse

from orca_core.hand_runtime import OrcaHand
from orca_core.utils.yaml_io import get_named_model_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Run ORCA hand tension routine.")
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to ORCA hand model directory.",
    )
    parser.add_argument(
        "--move_motors",
        action="store_true",
        help="Move motors during tension setup.",
    )
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        default=None,
        help="Physical hand side when model_path is omitted.",
    )
    parser.add_argument(
        "--hand",
        choices=("v1", "dip"),
        default=None,
        help="Temporary right-hand profile selector used when model_path is omitted.",
    )
    args = parser.parse_args()

    if args.model_path and (args.side or args.hand):
        parser.error("Pass either model_path or --side/--hand, not both.")

    if args.model_path:
        model_path = args.model_path
    elif args.side == "left":
        model_path = get_named_model_path("left")
    elif args.side == "right":
        model_path = get_named_model_path(args.hand or "v1")
    elif args.hand:
        model_path = get_named_model_path(args.hand)
    else:
        parser.error(
            "Refusing to guess a hand model. Pass model_path, --side left|right, or --hand v1|dip."
        )

    hand = OrcaHand(model_path)
    success, message = hand.connect()
    print((success, message))
    if not success:
        return 1

    hand.enable_torque()
    hand.tension(args.move_motors)
    hand.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
