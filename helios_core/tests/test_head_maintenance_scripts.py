import io
import sys
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

ORCA_CORE_ROOT = Path(__file__).resolve().parents[2]
if str(ORCA_CORE_ROOT) not in sys.path:
    sys.path.insert(0, str(ORCA_CORE_ROOT))

from helios_core.scripts import helios_head_reboot, helios_head_tension


def _head_config():
    return {
        "hardware": {
            "port": "/dev/serial/by-id/head",
            "baudrate": 3000000,
            "motor_ids": {"yaw": 21, "pitch": 22, "roll": 23},
            "disabled_motor_axes": ["yaw"],
        }
    }


class TestHeliosHeadMaintenanceScripts(unittest.TestCase):
    def test_reboot_dry_run_targets_only_active_motor_axes(self):
        out = io.StringIO()
        with mock.patch.object(
            helios_head_reboot,
            "load_head_config",
            return_value=(_head_config(), Path("head_config.yaml")),
        ):
            with redirect_stdout(out):
                result = helios_head_reboot.main(["--role", "helios_head", "--dry-run"])

        self.assertEqual(result, 0)
        text = out.getvalue()
        self.assertIn("disabled_axes=('yaw',)", text)
        self.assertIn("active_ids=[22, 23]", text)
        self.assertNotIn("id=21", text)

    def test_tension_dry_run_resolves_without_opening_hardware(self):
        out = io.StringIO()
        with mock.patch.object(
            helios_head_tension,
            "load_head_config",
            return_value=(_head_config(), Path("head_config.yaml")),
        ), mock.patch.object(helios_head_tension, "HeliosHeadCalibrator") as calibrator:
            with redirect_stdout(out):
                result = helios_head_tension.main(["--role", "helios_head", "--dry-run"])

        self.assertEqual(result, 0)
        calibrator.assert_not_called()
        self.assertIn("Tension hold plan", out.getvalue())


if __name__ == "__main__":
    unittest.main()
