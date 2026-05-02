import unittest

from helios_core.head_calibration import HeliosHeadCalibrator
from helios_core.head_config import HeadMotorIDs
from helios_core.head_model import HeliosHeadCalibrationModel
from helios_core.head_runtime import HeliosHeadRuntime
from hardware.head_hardware import HeliosHeadHardware


class TestHeliosImports(unittest.TestCase):
    def test_symbols_resolve(self):
        self.assertIsNotNone(HeliosHeadCalibrator)
        self.assertIsNotNone(HeliosHeadRuntime)
        self.assertIsNotNone(HeliosHeadCalibrationModel)
        self.assertIsNotNone(HeadMotorIDs)
        self.assertIsNotNone(HeliosHeadHardware)


if __name__ == "__main__":
    unittest.main()

