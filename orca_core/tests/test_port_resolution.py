import unittest
from unittest.mock import patch

from orca_core.hand_runtime import OrcaHand


class TestOrcaPortResolution(unittest.TestCase):
    def _hand(self, port: str, side: str | None = None) -> OrcaHand:
        hand = object.__new__(OrcaHand)
        hand.port = port
        hand.type = side
        return hand

    def test_exact_by_id_port_does_not_fall_back_to_other_ftdi(self):
        hand = self._hand("/dev/serial/by-id/right-hand")

        def exists(path: str) -> bool:
            return path == "/dev/ttyUSB1"

        with patch("orca_core.hand_runtime.os.path.exists", side_effect=exists), patch(
            "orca_core.hand_runtime.glob.glob",
            side_effect=lambda pattern: ["/dev/ttyUSB1"] if pattern == "/dev/ttyUSB*" else [],
        ):
            self.assertEqual(hand._resolve_port_path(), "/dev/serial/by-id/right-hand")

    def test_side_env_exact_port_does_not_fall_back_to_other_ftdi(self):
        hand = self._hand("/dev/serial/by-id/model-port", side="right")

        with patch.dict(
            "orca_core.hand_runtime.os.environ",
            {"ORCA_RIGHT_PORT": "/dev/serial/by-id/right-hand"},
            clear=False,
        ), patch("orca_core.hand_runtime.os.path.exists", return_value=False), patch(
            "orca_core.hand_runtime.glob.glob",
            return_value=["/dev/ttyUSB1"],
        ):
            self.assertEqual(hand._resolve_port_path(), "/dev/serial/by-id/right-hand")

    def test_role_resolved_port_can_ignore_legacy_env_ports(self):
        hand = self._hand("/dev/serial/by-id/role-port", side="right")
        hand.allow_env_port_override = False

        with patch.dict(
            "orca_core.hand_runtime.os.environ",
            {
                "ORCA_RIGHT_PORT": "/dev/serial/by-id/legacy-right",
                "ORCA_SERIAL_PORT": "/dev/serial/by-id/global-port",
            },
            clear=False,
        ), patch("orca_core.hand_runtime.os.path.exists", return_value=False):
            self.assertEqual(hand._resolve_port_path(), "/dev/serial/by-id/role-port")

    def test_wildcard_port_can_still_resolve_matching_device(self):
        hand = self._hand("/dev/serial/by-id/usb-FTDI*")

        def glob_paths(pattern: str) -> list[str]:
            if pattern == "/dev/serial/by-id/usb-FTDI*":
                return ["/dev/serial/by-id/usb-FTDI_MATCH"]
            if pattern == "/dev/ttyUSB*":
                return ["/dev/ttyUSB1"]
            return []

        def exists(path: str) -> bool:
            return path == "/dev/serial/by-id/usb-FTDI_MATCH"

        with patch("orca_core.hand_runtime.glob.glob", side_effect=glob_paths), patch(
            "orca_core.hand_runtime.os.path.exists",
            side_effect=exists,
        ):
            self.assertEqual(
                hand._resolve_port_path(),
                "/dev/serial/by-id/usb-FTDI_MATCH",
            )


if __name__ == "__main__":
    unittest.main()
