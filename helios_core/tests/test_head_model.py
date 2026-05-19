import unittest

import numpy as np

from helios_core.utils.head_config import HeadMotorIDs
from helios_core.head_model import (
    HeliosHeadCalibrationModel,
    PITCH_ROLL_COUPLING_COMMON_PITCH_DIFFERENTIAL_ROLL,
    derive_endpoint_joint_to_motor_ratios,
)


def _calibration_payload():
    return {
        "version": 2,
        "calibrated": True,
        "hardware": {
            "motor_ids": {
                "yaw": 21,
                "pitch": 22,
                "roll": 23,
            },
        },
        "motor_limits": {
            "yaw": [-2.0, 2.0],
            "pitch": [-2.0, 2.0],
            "roll": [-2.0, 2.0],
        },
        "neutral": {
            "motors": {
                "yaw": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
            },
        },
        "joint_to_motor_ratios": {
            "yaw": 2.0,
            "pitch": 2.0,
            "roll": 2.0,
        },
        "signs": {
            "yaw_sign": 1.0,
            "pitch_sign": 1.0,
            "roll_sign": 1.0,
        },
        "virtual_limits_rad": {
            "yaw": 0.5,
            "pitch": 0.5,
            "roll": 0.5,
        },
    }


class TestHeliosHeadEndpointModel(unittest.TestCase):
    def test_zero_command_maps_to_neutral(self):
        model = HeliosHeadCalibrationModel.from_yaml_dict(_calibration_payload())

        target = model.virtual_to_motor_targets(np.zeros(3, dtype=float))

        np.testing.assert_allclose(target, np.zeros(3, dtype=float))

    def test_virtual_motor_round_trip(self):
        model = HeliosHeadCalibrationModel.from_yaml_dict(_calibration_payload())
        cmd = np.array([0.25, 0.2, -0.1], dtype=float)

        target = model.virtual_to_motor_targets(cmd)
        state = model.motor_to_virtual(target)

        np.testing.assert_allclose(target, np.array([0.5, 0.4, -0.2], dtype=float))
        np.testing.assert_allclose(state, cmd)

    def test_coupled_pitch_roll_round_trip(self):
        payload = _calibration_payload()
        payload["pitch_roll_coupling"] = PITCH_ROLL_COUPLING_COMMON_PITCH_DIFFERENTIAL_ROLL
        model = HeliosHeadCalibrationModel.from_yaml_dict(payload)
        cmd = np.array([0.25, 0.2, -0.1], dtype=float)

        target = model.virtual_to_motor_targets(cmd)
        state = model.motor_to_virtual(target)

        np.testing.assert_allclose(target, np.array([0.5, 0.2, 0.6], dtype=float))
        np.testing.assert_allclose(state, cmd)

        pitch_only = model.virtual_to_motor_targets(np.array([0.0, 0.2, 0.0], dtype=float))
        roll_only = model.virtual_to_motor_targets(np.array([0.0, 0.0, 0.2], dtype=float))
        np.testing.assert_allclose(pitch_only, np.array([0.0, 0.4, 0.4], dtype=float))
        np.testing.assert_allclose(roll_only, np.array([0.0, 0.4, -0.4], dtype=float))

    def test_virtual_and_motor_clipping(self):
        model = HeliosHeadCalibrationModel.from_yaml_dict(_calibration_payload())

        target = model.virtual_to_motor_targets(np.array([2.0, 2.0, 0.0], dtype=float))
        clipped = model.clip_motor_targets(np.array([5.0, -5.0, 0.0], dtype=float), margin_rad=0.25)

        np.testing.assert_allclose(target, np.array([1.0, 1.0, 0.0], dtype=float))
        np.testing.assert_allclose(clipped, np.array([1.75, -1.75, 0.0], dtype=float))

    def test_old_or_uncalibrated_payload_fails_fast(self):
        with self.assertRaisesRegex(ValueError, "schema version 2"):
            HeliosHeadCalibrationModel.from_yaml_dict({"version": 1, "calibrated": True})

        payload = _calibration_payload()
        payload["calibrated"] = False
        with self.assertRaisesRegex(ValueError, "not marked calibrated"):
            HeliosHeadCalibrationModel.from_yaml_dict(payload)

    def test_missing_endpoint_ratio_fails_fast(self):
        payload = _calibration_payload()
        payload["joint_to_motor_ratios"].pop("roll")

        with self.assertRaisesRegex(ValueError, "joint_to_motor_ratios.roll"):
            HeliosHeadCalibrationModel.from_yaml_dict(payload)

    def test_endpoint_ratio_derivation(self):
        ratios = derive_endpoint_joint_to_motor_ratios(
            motor_limits={
                "yaw": (-1.0, 2.0),
                "pitch": (-4.0, 5.0),
                "roll": (-3.0, 7.0),
            },
            neutral_motors={
                "yaw": 0.0,
                "pitch": 1.0,
                "roll": 2.0,
            },
            virtual_limits_rad={
                "yaw": 0.5,
                "pitch": 1.0,
                "roll": 2.0,
            },
        )

        self.assertEqual(ratios, {"yaw": 2.0, "pitch": 4.0, "roll": 2.5})

    def test_coupled_endpoint_ratio_derivation_uses_shared_upper_motor_margin(self):
        ratios = derive_endpoint_joint_to_motor_ratios(
            motor_limits={
                "yaw": (-1.0, 2.0),
                "pitch": (-4.0, 5.0),
                "roll": (-3.0, 7.0),
            },
            neutral_motors={
                "yaw": 0.0,
                "pitch": 1.0,
                "roll": 2.0,
            },
            virtual_limits_rad={
                "yaw": 0.5,
                "pitch": 1.0,
                "roll": 2.0,
            },
            pitch_roll_coupling=PITCH_ROLL_COUPLING_COMMON_PITCH_DIFFERENTIAL_ROLL,
        )

        self.assertEqual(ratios, {"yaw": 2.0, "pitch": 4.0, "roll": 2.0})

    def test_explicit_motor_ids_override_yaml(self):
        model = HeliosHeadCalibrationModel.from_yaml_dict(
            _calibration_payload(),
            motor_ids=HeadMotorIDs(yaw=31, pitch=32, roll=33),
        )

        self.assertEqual(model.motor_ids.ordered, (31, 32, 33))

    def test_legacy_upper_motor_axis_names_still_load(self):
        payload = _calibration_payload()
        payload["hardware"]["motor_ids"] = {
            "yaw": 21,
            "upper_left": 22,
            "upper_right": 23,
        }
        payload["motor_limits"] = {
            "yaw": [-2.0, 2.0],
            "upper_left": [-2.0, 2.0],
            "upper_right": [-2.0, 2.0],
        }
        payload["neutral"]["motors"] = {
            "yaw": 0.0,
            "upper_left": 0.0,
            "upper_right": 0.0,
        }

        model = HeliosHeadCalibrationModel.from_yaml_dict(payload)

        self.assertEqual(model.motor_ids.ordered, (21, 22, 23))
        np.testing.assert_allclose(
            model.virtual_to_motor_targets(np.array([0.1, 0.2, 0.3], dtype=float)),
            np.array([0.2, 0.4, 0.6], dtype=float),
        )


if __name__ == "__main__":
    unittest.main()
