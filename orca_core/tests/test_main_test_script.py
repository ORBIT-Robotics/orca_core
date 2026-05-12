import unittest

from orca_core.scripts.main_test import (
    build_sine_pose,
    thumb_out_position,
    validate_rom_scale,
)


class TestMainTestScript(unittest.TestCase):
    def test_build_sine_pose_starts_at_neutral_during_ramp(self):
        pose = build_sine_pose(
            joint_ids=["index_mcp", "wrist"],
            joint_roms={"index_mcp": [-25, 100], "wrist": [-65, 35]},
            neutral_position={"index_mcp": 10, "wrist": -20},
            elapsed=0.0,
            period=4.0,
            ramp_seconds=1.0,
        )

        self.assertEqual(pose["index_mcp"], 10)
        self.assertEqual(pose["wrist"], -20)

    def test_build_sine_pose_respects_rom_bounds(self):
        for elapsed in [0.0, 0.5, 1.0, 2.0, 3.0]:
            pose = build_sine_pose(
                joint_ids=["index_mcp", "middle_mcp", "wrist"],
                joint_roms={
                    "index_mcp": [-25, 100],
                    "middle_mcp": [-25, 100],
                    "wrist": [-65, 35],
                },
                neutral_position={"index_mcp": 0, "middle_mcp": 0, "wrist": -20},
                elapsed=elapsed,
                period=4.0,
                ramp_seconds=0.0,
            )
            self.assertGreaterEqual(pose["index_mcp"], -25)
            self.assertLessEqual(pose["index_mcp"], 100)
            self.assertGreaterEqual(pose["middle_mcp"], -25)
            self.assertLessEqual(pose["middle_mcp"], 100)
            self.assertGreaterEqual(pose["wrist"], -65)
            self.assertLessEqual(pose["wrist"], 35)

    def test_fingers_are_phase_offset(self):
        pose = build_sine_pose(
            joint_ids=["index_mcp", "middle_mcp"],
            joint_roms={"index_mcp": [-25, 100], "middle_mcp": [-25, 100]},
            neutral_position={"index_mcp": 0, "middle_mcp": 0},
            elapsed=1.0,
            period=4.0,
            ramp_seconds=0.0,
            finger_phase_step=0.12,
        )

        self.assertNotEqual(pose["index_mcp"], pose["middle_mcp"])

    def test_thumb_is_held_fully_out_by_default(self):
        pose = build_sine_pose(
            joint_ids=["thumb_mcp", "thumb_abd", "thumb_pip", "thumb_dip", "index_mcp"],
            joint_roms={
                "thumb_mcp": [-45, 33],
                "thumb_abd": [-18, 55],
                "thumb_pip": [-25, 100],
                "thumb_dip": [-15, 107],
                "index_mcp": [-25, 100],
            },
            neutral_position={
                "thumb_mcp": 0,
                "thumb_abd": 50,
                "thumb_pip": 33,
                "thumb_dip": 18,
                "index_mcp": 0,
            },
            elapsed=1.0,
            period=4.0,
            ramp_seconds=0.0,
        )

        self.assertEqual(pose["thumb_mcp"], -45)
        self.assertEqual(pose["thumb_abd"], 55)
        self.assertEqual(pose["thumb_pip"], -25)
        self.assertEqual(pose["thumb_dip"], -15)

    def test_thumb_out_position_uses_abduction_out_and_flexion_extended(self):
        self.assertEqual(thumb_out_position("thumb_abd", -18, 55), 55)
        self.assertEqual(thumb_out_position("thumb_mcp", -45, 33), -45)
        self.assertEqual(thumb_out_position("thumb_pip", -25, 100), -25)
        self.assertEqual(thumb_out_position("thumb_dip", -15, 107), -15)

    def test_thumb_can_be_moved_when_requested(self):
        pose = build_sine_pose(
            joint_ids=["thumb_mcp"],
            joint_roms={"thumb_mcp": [-45, 33]},
            neutral_position={"thumb_mcp": 0},
            elapsed=0.5,
            period=4.0,
            ramp_seconds=0.0,
            hold_thumb_out=False,
        )

        self.assertNotEqual(pose["thumb_mcp"], 33)

    def test_validate_rom_scale_rejects_above_full_rom(self):
        with self.assertRaises(Exception):
            validate_rom_scale(1.1)


if __name__ == "__main__":
    unittest.main()
