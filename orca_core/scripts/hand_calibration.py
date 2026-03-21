"""Hand calibration helpers for OrcaHand runtime objects."""

from __future__ import annotations

from orca_core.scripts.hand_runtime import OrcaHand


def run_hand_calibration(hand: OrcaHand, *, blocking: bool = True) -> None:
    hand.calibrate(blocking=blocking)


def run_hand_tension(hand: OrcaHand, *, move_motors: bool = False, blocking: bool = True) -> None:
    hand.tension(move_motors=move_motors, blocking=blocking)
