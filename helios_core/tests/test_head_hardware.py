import unittest

import numpy as np

from helios_core.utils.head_config import HeadMotorIDs
from hardware.head_hardware import HeliosHeadHardware


class RecordingDynamixelClient:
    instances = []

    def __init__(self, motor_ids, port, baudrate):
        self.motor_ids = list(motor_ids)
        self.port = port
        self.baudrate = baudrate
        self.is_connected = False
        self.torque_calls = []
        self.mode_calls = []
        self.current_calls = []
        self.position_calls = []
        self.positions = np.array([1.2, -0.4], dtype=float)
        self.velocities = np.array([0.1, -0.2], dtype=float)
        self.currents = np.array([11.0, 12.0], dtype=float)
        RecordingDynamixelClient.instances.append(self)

    def connect(self):
        self.is_connected = True
        self.set_torque_enabled(self.motor_ids, True)

    def disconnect(self):
        self.is_connected = False

    def set_torque_enabled(self, motor_ids, enabled, retries=-1, retry_interval=0.25):
        self.torque_calls.append((tuple(motor_ids), bool(enabled)))
        return []

    def set_operating_mode(self, motor_ids, mode_value):
        self.mode_calls.append((tuple(motor_ids), int(mode_value)))

    def write_desired_current(self, motor_ids, current):
        self.current_calls.append((tuple(motor_ids), np.asarray(current, dtype=float).copy()))

    def read_pos_vel_cur(self):
        return self.positions.copy(), self.velocities.copy(), self.currents.copy()

    def write_desired_pos(self, motor_ids, positions):
        self.position_calls.append((tuple(motor_ids), np.asarray(positions, dtype=float).copy()))


class TestHeliosHeadHardwareDisabledAxes(unittest.TestCase):
    def setUp(self):
        RecordingDynamixelClient.instances = []

    def _hardware(self):
        class TestHardware(HeliosHeadHardware):
            def _ensure_client_class(self):
                return RecordingDynamixelClient

        return TestHardware(
            {
                "hardware": {
                    "port": "/dev/test-head",
                    "baudrate": 3000000,
                    "motor_ids": {"yaw": 21, "pitch": 22, "roll": 23},
                    "disabled_motor_axes": ["yaw"],
                    "disabled_motor_positions_rad": {"yaw": -2.5},
                }
            },
            HeadMotorIDs(yaw=21, pitch=22, roll=23),
        )

    def test_disabled_yaw_is_not_commanded(self):
        hardware = self._hardware()

        hardware.connect()
        client = RecordingDynamixelClient.instances[-1]
        bounded = hardware.command_motor_positions(np.array([9.0, 1.4, -0.1], dtype=float), max_step_rad=0.0)
        hardware.disable_torque()

        self.assertEqual(client.motor_ids, [22, 23])
        self.assertNotIn(((21,), True), client.torque_calls)
        self.assertIn(((21,), False), client.torque_calls)
        self.assertEqual(client.position_calls[-1][0], (22, 23))
        self.assertEqual(float(bounded[0]), -2.5)
        np.testing.assert_allclose(client.position_calls[-1][1], np.array([1.4, -0.1], dtype=float))

    def test_read_state_preserves_three_axis_shape_with_synthetic_yaw(self):
        hardware = self._hardware()
        hardware.connect()

        state = hardware.read_state()

        np.testing.assert_allclose(state.positions_rad, np.array([-2.5, 1.2, -0.4], dtype=float))
        np.testing.assert_allclose(state.velocities_rad_s, np.array([0.0, 0.1, -0.2], dtype=float))
        np.testing.assert_allclose(state.currents_ma, np.array([0.0, 11.0, 12.0], dtype=float))


if __name__ == "__main__":
    unittest.main()
