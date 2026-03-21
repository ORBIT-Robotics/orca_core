"""CLI for moving ORCA hand to neutral position."""

from __future__ import annotations

import argparse

from orca_core.scripts.hand_runtime import OrcaHand


def main() -> int:
    parser = argparse.ArgumentParser(description="Move ORCA hand to neutral position.")
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to ORCA hand model directory.",
    )
    args = parser.parse_args()

    hand = OrcaHand(model_path=args.model_path)
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
