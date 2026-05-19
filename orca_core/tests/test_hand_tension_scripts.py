import unittest
from unittest import mock

from orca_core.scripts import hand_tension, hand_tension_all


class TestHandTensionScripts(unittest.TestCase):
    def test_all_hands_wrapper_delegates_to_tension_main(self):
        argv = ["--dry-run", "--yes"]

        with mock.patch.object(hand_tension, "main", return_value=0) as tension_main:
            result = hand_tension_all.main(argv)

        self.assertEqual(result, 0)
        tension_main.assert_called_once_with(argv)

    def test_no_role_defaults_to_all_hands_multi_role_path(self):
        specs = [mock.Mock(role="helios_upper_left_feetech"), mock.Mock(role="helios_lower_left")]

        with (
            mock.patch.object(hand_tension, "_validate_roles", return_value=specs) as validate_roles,
            mock.patch.object(hand_tension, "_run_multi_role", return_value=0) as run_multi_role,
        ):
            result = hand_tension.main(["--dry-run"])

        self.assertEqual(result, 0)
        validate_roles.assert_called_once_with(list(hand_tension.DEFAULT_ROLES))
        run_multi_role.assert_called_once()
        args, called_specs = run_multi_role.call_args.args
        self.assertTrue(args.dry_run)
        self.assertEqual(called_specs, specs)


if __name__ == "__main__":
    unittest.main()
