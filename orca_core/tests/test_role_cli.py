import unittest
from pathlib import Path
from unittest import mock

from orca_core.scripts import _role_cli


REPO_ROOT = Path(__file__).resolve().parents[2]


class TestRoleCli(unittest.TestCase):
    def test_create_hand_uses_resolved_role_port_without_env_override(self):
        spec = mock.Mock(
            model_path=REPO_ROOT / "models" / "orca_hand_helios_upper_right_feetech",
            port="/dev/serial/by-id/role-port",
            baudrate=1000000,
        )

        hand = _role_cli.create_hand(spec)

        self.assertEqual(hand.port, "/dev/serial/by-id/role-port")
        self.assertEqual(hand.baudrate, 1000000)
        self.assertFalse(hand.allow_env_port_override)


if __name__ == "__main__":
    unittest.main()
