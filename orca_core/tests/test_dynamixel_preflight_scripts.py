import unittest
from unittest import mock

from orca_core.scripts import _dynamixel_preflight, _role_cli, hand_dynamixel_preflight


class TestDynamixelPreflightScripts(unittest.TestCase):
    def test_robot_preflight_uses_enabled_hardware_roles(self):
        spec = mock.Mock(role="helios_lower_left")

        with (
            mock.patch.object(hand_dynamixel_preflight, "load_robot_orca_hardware_roles", return_value=[spec]) as load_roles,
            mock.patch.object(hand_dynamixel_preflight, "print_role_summary"),
            mock.patch.object(hand_dynamixel_preflight, "_role_motor_type", return_value="dynamixel"),
            mock.patch.object(hand_dynamixel_preflight, "_role_motor_ids", return_value=[1, 2]),
            mock.patch.object(hand_dynamixel_preflight, "preflight_dynamixel_roles", return_value=True) as preflight,
        ):
            result = hand_dynamixel_preflight.main(["--robot", "helios"])

        self.assertEqual(result, 0)
        load_roles.assert_called_once()
        self.assertEqual(load_roles.call_args.args[0], "helios")
        preflight.assert_called_once_with([spec])

    def test_dry_run_does_not_touch_hardware_preflight(self):
        spec = mock.Mock(role="helios_lower_left")

        with (
            mock.patch.object(hand_dynamixel_preflight, "load_robot_orca_hardware_roles", return_value=[spec]),
            mock.patch.object(hand_dynamixel_preflight, "print_role_summary"),
            mock.patch.object(hand_dynamixel_preflight, "_role_motor_type", return_value="dynamixel"),
            mock.patch.object(hand_dynamixel_preflight, "_role_motor_ids", return_value=[1, 2]),
            mock.patch.object(hand_dynamixel_preflight, "preflight_dynamixel_roles") as preflight,
        ):
            result = hand_dynamixel_preflight.main(["--robot", "helios", "--dry-run"])

        self.assertEqual(result, 0)
        preflight.assert_not_called()

    def test_preflight_reboots_only_bad_dynamixel_ids(self):
        spec = mock.Mock(role="helios_lower_left")
        report = _dynamixel_preflight.DynamixelRoleStatusReport(
            role=spec.role,
            port="/dev/test",
            baudrate=1000000,
            statuses=(
                _dynamixel_preflight.DynamixelMotorStatus(
                    motor_id=1,
                    hardware_error_status=0,
                    comm_result=0,
                    dxl_error=0,
                    packet_status="ok",
                ),
                _dynamixel_preflight.DynamixelMotorStatus(
                    motor_id=2,
                    hardware_error_status=1,
                    comm_result=0,
                    dxl_error=0,
                    packet_status="voltage",
                ),
            ),
        )

        with (
            mock.patch.object(_dynamixel_preflight, "_role_motor_type", return_value="dynamixel"),
            mock.patch.object(_dynamixel_preflight, "check_dynamixel_role_status", return_value=report),
            mock.patch.object(_dynamixel_preflight, "_reboot_dynamixel_role", return_value=True) as reboot,
        ):
            result = _dynamixel_preflight.preflight_dynamixel_role(spec)

        self.assertTrue(result)
        reboot.assert_called_once_with(spec, motor_ids=[2])

    def test_preflight_transport_failure_does_not_reboot(self):
        spec = mock.Mock(role="helios_lower_left")
        report = _dynamixel_preflight.DynamixelRoleStatusReport(
            role=spec.role,
            port="/dev/test",
            baudrate=1000000,
            transport_error="failed to open port /dev/test",
        )

        with (
            mock.patch.object(_dynamixel_preflight, "_role_motor_type", return_value="dynamixel"),
            mock.patch.object(_dynamixel_preflight, "check_dynamixel_role_status", return_value=report),
            mock.patch.object(_dynamixel_preflight, "_reboot_dynamixel_role") as reboot,
        ):
            result = _dynamixel_preflight.preflight_dynamixel_role(spec)

        self.assertFalse(result)
        reboot.assert_not_called()

    def test_connect_helper_stops_before_connect_when_preflight_fails(self):
        spec = mock.Mock(role="helios_lower_left")
        hand = mock.Mock()

        with (
            mock.patch.object(_role_cli, "_role_motor_type", return_value="dynamixel"),
            mock.patch.object(_role_cli, "preflight_dynamixel_role", return_value=False) as preflight,
        ):
            success, message = _role_cli.connect_hand_with_dynamixel_preflight(spec, hand)

        self.assertFalse(success)
        self.assertIn("Dynamixel preflight failed", message)
        preflight.assert_called_once_with(spec)
        hand.connect.assert_not_called()

    def test_connect_helper_skips_preflight_for_non_dynamixel_roles(self):
        spec = mock.Mock(role="helios_upper_left_feetech")
        hand = mock.Mock()
        hand.connect.return_value = (True, "Connection successful")

        with (
            mock.patch.object(_role_cli, "_role_motor_type", return_value="feetech"),
            mock.patch.object(_role_cli, "preflight_dynamixel_role") as preflight,
        ):
            success, message = _role_cli.connect_hand_with_dynamixel_preflight(spec, hand)

        self.assertTrue(success)
        self.assertEqual(message, "Connection successful")
        preflight.assert_not_called()
        hand.connect.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
