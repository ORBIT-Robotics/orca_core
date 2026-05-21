import unittest
from unittest import mock

from orca_core.scripts import hand_reboot_all


class TestHandRebootAllScript(unittest.TestCase):
    def test_dry_run_prints_plan_without_rebooting(self):
        spec = mock.Mock()
        spec.role = "helios_lower_left"

        with (
            mock.patch.object(hand_reboot_all, "_validate_roles", return_value=[spec]),
            mock.patch.object(hand_reboot_all, "print_role_summary"),
            mock.patch.object(hand_reboot_all, "_role_motor_type", return_value="dynamixel"),
            mock.patch.object(hand_reboot_all, "_role_motor_ids", return_value=[1, 2]),
            mock.patch.object(hand_reboot_all, "_reboot_dynamixel_roles") as reboot,
        ):
            result = hand_reboot_all.main(["--role", spec.role, "--dry-run"])

        self.assertEqual(result, 0)
        reboot.assert_not_called()

    def test_runs_shared_dynamixel_reboot_logic_without_confirmation(self):
        spec = mock.Mock()
        spec.role = "helios_lower_left"

        with (
            mock.patch.object(hand_reboot_all, "_validate_roles", return_value=[spec]),
            mock.patch.object(hand_reboot_all, "print_role_summary"),
            mock.patch.object(hand_reboot_all, "_role_motor_type", return_value="dynamixel"),
            mock.patch.object(hand_reboot_all, "_role_motor_ids", return_value=[1, 2]),
            mock.patch.object(hand_reboot_all, "_reboot_dynamixel_roles", return_value=True) as reboot,
            mock.patch("builtins.input") as input_mock,
        ):
            result = hand_reboot_all.main(["--role", spec.role])

        self.assertEqual(result, 0)
        reboot.assert_called_once_with([spec])
        input_mock.assert_not_called()


if __name__ == "__main__":
    unittest.main()
