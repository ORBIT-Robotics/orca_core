import unittest
from pathlib import Path
from unittest import mock

from orca_core.scripts import hand_neutral, hand_neutral_all


class FakeHand:
    def __init__(self):
        self.calls = []

    def connect(self):
        self.calls.append(("connect",))
        return True, "ok"

    def enable_torque(self):
        self.calls.append(("enable_torque",))

    def set_neutral_position(self, num_steps, step_size):
        self.calls.append(("set_neutral_position", num_steps, step_size))

    def disable_torque(self):
        self.calls.append(("disable_torque",))

    def disconnect(self):
        self.calls.append(("disconnect",))


class TestHandNeutralScripts(unittest.TestCase):
    def test_single_hand_passes_step_options_to_runtime(self):
        fake_hand = FakeHand()

        with (
            mock.patch.object(hand_neutral, "resolve_role", return_value=object()),
            mock.patch.object(hand_neutral, "print_role_summary"),
            mock.patch.object(hand_neutral, "create_hand", return_value=fake_hand),
            mock.patch.object(
                hand_neutral,
                "connect_hand_with_dynamixel_preflight",
                side_effect=lambda _role, hand: hand.connect(),
            ),
        ):
            result = hand_neutral.main(
                [
                    "--role",
                    "helios_lower_left",
                    "--num-steps",
                    "7",
                    "--step-size",
                    "0.02",
                ]
            )

        self.assertEqual(result, 0)
        self.assertEqual(
            fake_hand.calls,
            [
                ("connect",),
                ("enable_torque",),
                ("set_neutral_position", 7, 0.02),
                ("disable_torque",),
                ("disconnect",),
            ],
        )

    def test_single_hand_rejects_invalid_step_count(self):
        result = hand_neutral.main(
            ["--role", "helios_lower_left", "--num-steps", "0"]
        )

        self.assertEqual(result, 2)

    def test_all_hands_child_command_targets_single_hand_neutral_script(self):
        command = hand_neutral_all._build_child_command(
            "helios_lower_left",
            num_steps=5,
            step_size=0.01,
        )

        self.assertEqual(Path(command[1]).name, "hand_neutral.py")
        self.assertIn("--role", command)
        self.assertIn("helios_lower_left", command)
        self.assertIn("--num-steps", command)
        self.assertIn("5", command)
        self.assertIn("--step-size", command)
        self.assertIn("0.01", command)

    def test_all_hands_dry_run_does_not_spawn_processes(self):
        spec = mock.Mock()
        spec.role = "helios_lower_left"

        with (
            mock.patch.object(hand_neutral_all, "_validate_roles", return_value=[spec]),
            mock.patch.object(hand_neutral_all, "print_role_summary"),
            mock.patch.object(hand_neutral_all.subprocess, "Popen") as popen,
        ):
            result = hand_neutral_all.main(["--role", spec.role, "--dry-run"])

        self.assertEqual(result, 0)
        popen.assert_not_called()


if __name__ == "__main__":
    unittest.main()
