"""CLI for ORCA hand tensioning."""

from __future__ import annotations

import argparse

from orca_core.scripts.hand_runtime import OrcaHand


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
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
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
