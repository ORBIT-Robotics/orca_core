import unittest

from orca_core.scripts.feetech_set_origin import DEFAULT_MOTOR_IDS, validate_target_raw


class TestFeetechSetOriginScript(unittest.TestCase):
    def test_validate_target_raw_accepts_servo_range(self):
        self.assertEqual(validate_target_raw(0), 0)
        self.assertEqual(validate_target_raw(2048), 2048)
        self.assertEqual(validate_target_raw(4095), 4095)

    def test_validate_target_raw_rejects_out_of_range(self):
        with self.assertRaises(ValueError):
            validate_target_raw(-1)
        with self.assertRaises(ValueError):
            validate_target_raw(4096)

    def test_default_motor_ids_are_full_feetech_hand(self):
        self.assertEqual(DEFAULT_MOTOR_IDS, tuple(range(1, 18)))


if __name__ == "__main__":
    unittest.main()
