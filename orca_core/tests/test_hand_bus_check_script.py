import unittest
from unittest import mock

from orca_core.scripts import _bus_preflight, hand_bus_check


class TestHandBusCheckScript(unittest.TestCase):
    def test_robot_bus_check_uses_enabled_hardware_roles(self):
        spec = mock.Mock(role="helios_upper_left_feetech")

        with (
            mock.patch.object(hand_bus_check, "load_robot_orca_hardware_roles", return_value=[spec]) as load_roles,
            mock.patch.object(hand_bus_check, "print_role_summary"),
            mock.patch.object(hand_bus_check, "_role_motor_type", return_value="feetech"),
            mock.patch.object(hand_bus_check, "_role_motor_ids", return_value=[1, 2]),
            mock.patch.object(hand_bus_check, "preflight_motor_roles", return_value=True) as preflight,
        ):
            result = hand_bus_check.main(["--robot", "helios"])

        self.assertEqual(result, 0)
        load_roles.assert_called_once()
        self.assertEqual(load_roles.call_args.args[0], "helios")
        preflight.assert_called_once_with([spec])

    def test_dry_run_does_not_touch_hardware_preflight(self):
        spec = mock.Mock(role="helios_lower_left")

        with (
            mock.patch.object(hand_bus_check, "load_robot_orca_hardware_roles", return_value=[spec]),
            mock.patch.object(hand_bus_check, "print_role_summary"),
            mock.patch.object(hand_bus_check, "_role_motor_type", return_value="dynamixel"),
            mock.patch.object(hand_bus_check, "_role_motor_ids", return_value=[1, 2]),
            mock.patch.object(hand_bus_check, "preflight_motor_roles") as preflight,
        ):
            result = hand_bus_check.main(["--robot", "helios", "--dry-run"])

        self.assertEqual(result, 0)
        preflight.assert_not_called()

    def test_feetech_preflight_checks_configured_ids(self):
        spec = mock.Mock(role="helios_upper_left_feetech", port="/dev/test", baudrate=1000000)
        rows = [mock.Mock(motor_id=1, ok=True), mock.Mock(motor_id=2, ok=True)]

        with (
            mock.patch.object(_bus_preflight, "_role_motor_type", return_value="feetech"),
            mock.patch.object(_bus_preflight, "_role_motor_ids", return_value=[1, 2]),
            mock.patch.object(_bus_preflight, "check_motors", return_value=rows) as check,
            mock.patch.object(_bus_preflight, "print_results"),
        ):
            result = _bus_preflight.preflight_feetech_role(spec)

        self.assertTrue(result)
        check.assert_called_once_with("/dev/test", 1000000, [1, 2])

    def test_feetech_preflight_fails_missing_motor(self):
        spec = mock.Mock(role="helios_upper_left_feetech", port="/dev/test", baudrate=1000000)
        rows = [mock.Mock(motor_id=1, ok=True), mock.Mock(motor_id=2, ok=False)]

        with (
            mock.patch.object(_bus_preflight, "_role_motor_type", return_value="feetech"),
            mock.patch.object(_bus_preflight, "_role_motor_ids", return_value=[1, 2]),
            mock.patch.object(_bus_preflight, "check_motors", return_value=rows),
            mock.patch.object(_bus_preflight, "print_results"),
        ):
            result = _bus_preflight.preflight_feetech_role(spec)

        self.assertFalse(result)


if __name__ == "__main__":
    unittest.main()
