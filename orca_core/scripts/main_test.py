"""Run a full-ROM sinusoidal ORCA hand motion test."""

from __future__ import annotations

import argparse
import math
import time
from typing import Mapping, Sequence

try:
    from orca_core.scripts._role_cli import (
        add_role_argument,
        create_hand,
        print_role_summary,
        resolve_role,
    )
except ModuleNotFoundError:
    from _role_cli import add_role_argument, create_hand, print_role_summary, resolve_role


FINGER_PHASE_INDEX = {
    "thumb": 0,
    "index": 1,
    "middle": 2,
    "ring": 3,
    "pinky": 4,
}

THUMB_OUT_FRACTIONS = {
    "thumb_cmc": 1.0,
    "thumb_abd": 1.0,
    "thumb_mcp": 0.0,
    "thumb_pip": 0.0,
    "thumb_dip": 0.0,
}


def validate_rom_scale(value: float) -> float:
    value = float(value)
    if value <= 0.0 or value > 1.0:
        raise argparse.ArgumentTypeError("ROM scale must be in the range (0, 1].")
    return value


def validate_positive(value: float) -> float:
    value = float(value)
    if value <= 0.0:
        raise argparse.ArgumentTypeError("Value must be positive.")
    return value


def _finger_name(joint: str) -> str | None:
    if joint == "wrist":
        return None
    return joint.split("_", 1)[0]


def _clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def thumb_out_position(joint: str, lower: float, upper: float) -> float:
    """Return an open thumb target that keeps flexion joints extended."""
    fraction = THUMB_OUT_FRACTIONS.get(joint, 0.0)
    return lower + fraction * (upper - lower)


def build_sine_pose(
    joint_ids: Sequence[str],
    joint_roms: Mapping[str, Sequence[float]],
    neutral_position: Mapping[str, float],
    elapsed: float,
    period: float,
    rom_scale: float = 1.0,
    finger_phase_step: float = 0.12,
    wrist_phase: float = 0.25,
    ramp_seconds: float = 1.0,
    include_wrist: bool = True,
    include_abduction: bool = True,
    hold_thumb_out: bool = True,
) -> dict[str, float]:
    """Build a full-ROM sine pose, ramped from neutral at startup.

    Phase values are in cycles, so 0.25 means a quarter-wave offset.
    """
    pose: dict[str, float] = {}
    ramp = 1.0 if ramp_seconds <= 0.0 else _clip(elapsed / ramp_seconds, 0.0, 1.0)

    for joint in joint_ids:
        if joint == "wrist" and not include_wrist:
            continue
        if not include_abduction and joint.endswith("_abd"):
            continue
        if joint not in joint_roms:
            continue

        lower, upper = float(joint_roms[joint][0]), float(joint_roms[joint][1])
        neutral = float(neutral_position.get(joint, (lower + upper) / 2.0))

        if hold_thumb_out and joint.startswith("thumb_"):
            pose[joint] = thumb_out_position(joint, lower, upper)
            continue

        center = (lower + upper) / 2.0
        amplitude = (upper - lower) * 0.5 * rom_scale

        if joint == "wrist":
            phase_cycles = wrist_phase
        else:
            finger = _finger_name(joint)
            phase_cycles = FINGER_PHASE_INDEX.get(finger, 0) * finger_phase_step

        wave = center + amplitude * math.sin(
            2.0 * math.pi * ((elapsed / period) - phase_cycles)
        )
        pose[joint] = _clip(neutral + ramp * (wave - neutral), lower, upper)

    return pose


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Move all ORCA hand finger joints through their configured ROM with "
            "phase offsets, plus sinusoidal wrist motion."
        )
    )
    add_role_argument(parser)
    parser.add_argument("--cycles", type=validate_positive, default=3.0)
    parser.add_argument("--period", type=validate_positive, default=4.0)
    parser.add_argument("--rate-hz", type=validate_positive, default=25.0)
    parser.add_argument(
        "--rom-scale",
        type=validate_rom_scale,
        default=1.0,
        help="Fraction of each joint ROM to use. Default: 1.0 for full ROM.",
    )
    parser.add_argument(
        "--finger-phase-step",
        type=float,
        default=0.12,
        help="Phase offset between neighboring fingers, in sine cycles.",
    )
    parser.add_argument(
        "--wrist-phase",
        type=float,
        default=0.25,
        help="Wrist phase offset, in sine cycles.",
    )
    parser.add_argument("--ramp-seconds", type=float, default=1.0)
    parser.add_argument("--neutral-steps", type=int, default=50)
    parser.add_argument("--neutral-step-size", type=float, default=0.01)
    parser.add_argument("--no-wrist", action="store_true")
    parser.add_argument("--no-abduction", action="store_true")
    parser.add_argument(
        "--move-thumb",
        action="store_true",
        help="Include thumb joints in the sine motion. By default thumb stays fully outside/open.",
    )
    parser.add_argument(
        "--allow-partial",
        action="store_true",
        help="Run even if calibration.yaml is not fully calibrated.",
    )
    args = parser.parse_args()

    try:
        role = resolve_role(args.role)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1

    print_role_summary(role)
    hand = create_hand(role)
    success, message = hand.connect()
    print((success, message))
    if not success:
        return 1

    try:
        if not hand.calibrated and not args.allow_partial:
            print("ERROR: Hand is not fully calibrated. Use --allow-partial to override.")
            return 1

        hand.enable_torque()
        hand.set_control_mode(hand.control_mode)
        hand.set_max_current(hand.max_current)

        print("Moving to neutral...")
        hand.set_neutral_position(
            num_steps=args.neutral_steps,
            step_size=args.neutral_step_size,
        )

        duration = float(args.cycles) * float(args.period)
        dt = 1.0 / float(args.rate_hz)
        start = time.monotonic()
        print(
            "Running sine ROM test: "
            f"cycles={args.cycles}, period={args.period}s, rate={args.rate_hz}Hz, "
            f"rom_scale={args.rom_scale}."
        )

        while True:
            elapsed = time.monotonic() - start
            if elapsed >= duration:
                break

            pose = build_sine_pose(
                joint_ids=hand.joint_ids,
                joint_roms=hand.joint_roms_dict,
                neutral_position=hand.neutral_position,
                elapsed=elapsed,
                period=float(args.period),
                rom_scale=float(args.rom_scale),
                finger_phase_step=float(args.finger_phase_step),
                wrist_phase=float(args.wrist_phase),
                ramp_seconds=float(args.ramp_seconds),
                include_wrist=not args.no_wrist,
                include_abduction=not args.no_abduction,
                hold_thumb_out=not args.move_thumb,
            )
            hand.set_joint_pos(pose)
            time.sleep(dt)

        print("Returning to neutral...")
        hand.set_neutral_position(
            num_steps=args.neutral_steps,
            step_size=args.neutral_step_size,
        )
        return 0
    except KeyboardInterrupt:
        print("\nTest interrupted.")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        try:
            hand.disable_torque()
        finally:
            hand.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
