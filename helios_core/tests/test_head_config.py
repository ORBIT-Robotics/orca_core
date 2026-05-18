import unittest
from pathlib import Path

from helios_core.utils.head_config import resolve_repo_path


class TestHeliosHeadConfigPaths(unittest.TestCase):
    def test_missing_repo_style_orca_core_path_stays_under_submodule_root(self):
        orca_core_root = Path(__file__).resolve().parents[2]

        resolved = resolve_repo_path("orca_core/models/__missing_head__/calibration.yaml")

        self.assertEqual(
            resolved,
            orca_core_root / "models" / "__missing_head__" / "calibration.yaml",
        )
        self.assertNotIn("/helios_core/models/", str(resolved))


if __name__ == "__main__":
    unittest.main()
