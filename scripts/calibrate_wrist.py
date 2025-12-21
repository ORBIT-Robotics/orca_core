import argparse
from orca_core import OrcaHand


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate only the wrist joint of the ORCA Hand."
    )
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1)"
    )
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        raise SystemExit(1)

    original_sequence = hand.calib_sequence
    hand.calib_sequence = [
        {"step": 1, "joints": {"wrist": "flex"}},
        {"step": 2, "joints": {"wrist": "extend"}},
    ]
    try:
        hand.calibrate()
    finally:
        hand.calib_sequence = original_sequence


if __name__ == "__main__":
    main()
