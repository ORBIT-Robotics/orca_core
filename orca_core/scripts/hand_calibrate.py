"""CLI for ORCA hand calibration."""

from __future__ import annotations

import argparse

from orca_core.scripts.hand_runtime import OrcaHand


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate ORCA hand.")
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to ORCA hand model directory.",
    )
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    success, message = hand.connect()
    print((success, message))
    if not success:
        return 1

    hand.calibrate()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
