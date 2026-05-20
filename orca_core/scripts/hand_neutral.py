"""CLI for moving ORCA hand to neutral position."""

from __future__ import annotations

import argparse

try:
    from orca_core.scripts._role_cli import (
        add_role_argument,
        create_hand,
        connect_hand_with_dynamixel_preflight,
        print_role_summary,
        resolve_role,
    )
except ModuleNotFoundError:
    from _role_cli import add_role_argument, connect_hand_with_dynamixel_preflight, create_hand, print_role_summary, resolve_role


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Move ORCA hand to neutral position.")
    add_role_argument(parser)
    parser.add_argument(
        "--num-steps",
        type=int,
        default=25,
        help="Number of interpolation steps for the neutral move. Default: 25.",
    )
    parser.add_argument(
        "--step-size",
        type=float,
        default=0.001,
        help="Seconds between interpolation steps. Default: 0.001.",
    )
    args = parser.parse_args(argv)

    if args.num_steps < 1:
        print("ERROR: --num-steps must be >= 1")
        return 2
    if args.step_size < 0:
        print("ERROR: --step-size must be >= 0")
        return 2

    try:
        role = resolve_role(args.role)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1
    print_role_summary(role)
    hand = create_hand(role)
    success, message = connect_hand_with_dynamixel_preflight(role, hand)
    if not success:
        print(f"Failed to connect: {message}")
        return 1

    try:
        hand.enable_torque()
        hand.set_neutral_position(num_steps=args.num_steps, step_size=args.step_size)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        try:
            hand.disable_torque()
        finally:
            hand.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
