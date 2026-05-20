"""CLI smoke test for ORCA hand runtime control."""

from __future__ import annotations

import argparse
import time

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


def main() -> int:
    parser = argparse.ArgumentParser(description="Run ORCA hand control smoke test.")
    add_role_argument(parser)
    args = parser.parse_args()

    try:
        role = resolve_role(args.role)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1
    print_role_summary(role)
    hand = create_hand(role)
    success, message = connect_hand_with_dynamixel_preflight(role, hand)
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
