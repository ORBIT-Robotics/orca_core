import unittest

from orca_core.scripts.feetech_scan_baud import (
    DEFAULT_BAUDRATES,
    BaudScanResult,
    parse_baudrates,
)


class TestFeetechScanBaudScript(unittest.TestCase):
    def test_default_baudrates_include_common_feetech_values(self):
        self.assertIn(1000000, DEFAULT_BAUDRATES)
        self.assertIn(500000, DEFAULT_BAUDRATES)
        self.assertIn(115200, DEFAULT_BAUDRATES)

    def test_parse_baudrates_deduplicates_preserving_order(self):
        self.assertEqual(parse_baudrates("1000000,500000,1000000"), (1000000, 500000))

    def test_parse_baudrates_rejects_empty(self):
        with self.assertRaises(ValueError):
            parse_baudrates("")

    def test_parse_baudrates_rejects_non_positive(self):
        with self.assertRaises(ValueError):
            parse_baudrates("1000000,0")

    def test_scan_result_counts_responding_ids(self):
        result = BaudScanResult(
            baudrate=1000000,
            responding_ids=(1, 2),
            missing_ids=(3,),
        )

        self.assertEqual(result.ok_count, 2)


if __name__ == "__main__":
    unittest.main()
