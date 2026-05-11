import unittest

from orca_core.scripts.feetech_check import MotorCheckResult, parse_motor_ids


class TestFeetechCheckScript(unittest.TestCase):
    def test_parse_motor_ids_accepts_ranges_and_lists(self):
        self.assertEqual(parse_motor_ids("1-3,5,7-8"), (1, 2, 3, 5, 7, 8))

    def test_parse_motor_ids_deduplicates_preserving_order(self):
        self.assertEqual(parse_motor_ids("1,2,1,3"), (1, 2, 3))

    def test_parse_motor_ids_rejects_empty(self):
        with self.assertRaises(ValueError):
            parse_motor_ids("")

    def test_result_dataclass_marks_missing_motor(self):
        result = MotorCheckResult(
            motor_id=1,
            ok=False,
            model=None,
            position=None,
            speed=None,
            current_ma=None,
            temperature_c=None,
            result=-6,
            error=0,
            message="timeout",
        )

        self.assertFalse(result.ok)
        self.assertEqual(result.motor_id, 1)


if __name__ == "__main__":
    unittest.main()
