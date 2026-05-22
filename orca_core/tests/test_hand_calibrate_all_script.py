import tempfile
import unittest
from unittest import mock

from orca_core.scripts import hand_calibrate_all


def _spec(role: str, port: str):
    return mock.Mock(role=role, port=port)


class TestHandCalibrateAllScript(unittest.TestCase):
    def test_default_roles_start_with_lower_hands(self):
        self.assertEqual(
            hand_calibrate_all.DEFAULT_ROLES[:2],
            ("helios_lower_left", "helios_lower_right"),
        )

    def test_dry_run_does_not_preflight_or_spawn(self):
        spec = _spec("helios_lower_left", "/dev/test-left")

        with (
            mock.patch.object(hand_calibrate_all, "_validate_roles", return_value=[spec]),
            mock.patch.object(hand_calibrate_all, "print_role_summary"),
            mock.patch.object(hand_calibrate_all, "preflight_motor_role") as preflight,
            mock.patch.object(hand_calibrate_all, "_run_child_process") as run_child,
        ):
            result = hand_calibrate_all.main(["--role", spec.role, "--dry-run"])

        self.assertEqual(result, 0)
        preflight.assert_not_called()
        run_child.assert_not_called()

    def test_sequential_default_stops_after_preflight_failure(self):
        specs = [
            _spec("helios_lower_left", "/dev/test-left"),
            _spec("helios_lower_right", "/dev/test-right"),
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            with (
                mock.patch.object(hand_calibrate_all, "_validate_roles", return_value=specs),
                mock.patch.object(hand_calibrate_all, "print_role_summary"),
                mock.patch.object(hand_calibrate_all, "preflight_motor_role", return_value=False) as preflight,
                mock.patch.object(hand_calibrate_all, "_run_child_process") as run_child,
            ):
                result = hand_calibrate_all.main(["--log-dir", tmpdir])

        self.assertEqual(result, 1)
        preflight.assert_called_once_with(specs[0])
        run_child.assert_not_called()

    def test_continue_on_failure_moves_to_next_role(self):
        specs = [
            _spec("helios_lower_left", "/dev/test-left"),
            _spec("helios_lower_right", "/dev/test-right"),
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            with (
                mock.patch.object(hand_calibrate_all, "_validate_roles", return_value=specs),
                mock.patch.object(hand_calibrate_all, "print_role_summary"),
                mock.patch.object(hand_calibrate_all.time, "sleep"),
                mock.patch.object(hand_calibrate_all, "preflight_motor_role", side_effect=[False, True]) as preflight,
                mock.patch.object(hand_calibrate_all, "_run_child_process", return_value=0) as run_child,
            ):
                result = hand_calibrate_all.main([
                    "--log-dir",
                    tmpdir,
                    "--continue-on-failure",
                ])

        self.assertEqual(result, 1)
        self.assertEqual(preflight.call_count, 2)
        run_child.assert_called_once()
        self.assertEqual(run_child.call_args.args[0], specs[1])


if __name__ == "__main__":
    unittest.main()
