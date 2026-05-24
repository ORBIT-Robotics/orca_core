import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

from orca_core.hand_runtime import OrcaHand

try:
    from orca_core.hardware.dynamixel_client import DynamixelClient
    from orca_core.hardware.feetech_client import FeetechClient, POSITION_DIRECTION
except ModuleNotFoundError:
    from hardware.dynamixel_client import DynamixelClient
    from hardware.feetech_client import FeetechClient, POSITION_DIRECTION


REPO_ROOT = Path(__file__).resolve().parents[2]


class _RecordingMotorClient:
    def __init__(self, active_motor_ids):
        self.active_motor_ids = list(active_motor_ids)
        self.torque_calls = []
        self.operating_mode_calls = []
        self.current_writes = []

    def set_torque_enabled(self, motor_ids, enabled):
        self.torque_calls.append((tuple(motor_ids), enabled))
        return []

    def set_operating_mode(self, motor_ids, mode):
        self.operating_mode_calls.append((tuple(motor_ids), mode))

    def write_desired_current(self, motor_ids, current):
        self.current_writes.append((tuple(motor_ids), tuple(current)))

    def read_pos_vel_cur(self):
        positions = np.array([float(motor_id) for motor_id in self.active_motor_ids])
        velocities = np.array([float(motor_id) * 10.0 for motor_id in self.active_motor_ids])
        currents = np.array([float(motor_id) * 10.0 for motor_id in self.active_motor_ids])
        return positions, velocities, currents

    def read_temperature(self):
        return np.array([float(motor_id) + 30.0 for motor_id in self.active_motor_ids])


class TestMotorBackendSelection(unittest.TestCase):
    def test_existing_dynamixel_config_selects_dynamixel_client(self):
        hand = OrcaHand(REPO_ROOT / "models" / "orca_hand_ikarus_right")

        self.assertEqual(hand.motor_type, "dynamixel")
        self.assertEqual(hand._motor_client_class().__name__, DynamixelClient.__name__)

    def test_feetech_config_selects_feetech_client(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            source = REPO_ROOT / "models" / "orca_hand_helios_upper_right"
            model_path = Path(tmp_dir) / "orca_hand_feetech"
            shutil.copytree(source, model_path)

            config_path = model_path / "config.yaml"
            config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
            config["motor_type"] = "feetech"
            config["port"] = "/dev/ttyUSB-feetech"
            config["baudrate"] = 1000000
            config_path.write_text(
                yaml.safe_dump(config, sort_keys=False),
                encoding="utf-8",
            )

            hand = OrcaHand(model_path)

            self.assertEqual(hand.motor_type, "feetech")
            self.assertEqual(hand._motor_client_class().__name__, FeetechClient.__name__)
            self.assertEqual(
                type(hand._create_motor_client("/dev/null")).__name__,
                FeetechClient.__name__,
            )

    def test_upper_left_feetech_uses_configured_disabled_motor_ids(self):
        hand = OrcaHand(REPO_ROOT / "models" / "orca_hand_helios_upper_left_feetech")
        client = hand._create_motor_client("/dev/null")
        try:
            self.assertEqual(hand.disabled_motor_ids, {6})
            self.assertEqual(client.disabled_motor_ids, {6})
            self.assertIn(1, client.active_motor_ids)
            self.assertIn(2, client.active_motor_ids)
            self.assertNotIn(6, client.active_motor_ids)
        finally:
            client.disconnect()


    def test_dynamixel_disabled_motor_ids_are_not_activated(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            source = REPO_ROOT / "models" / "orca_hand_helios_lower_right"
            model_path = Path(tmp_dir) / "orca_hand_disabled"
            shutil.copytree(source, model_path)

            config_path = model_path / "config.yaml"
            config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
            config["disabled_motor_ids"] = [8]
            config_path.write_text(
                yaml.safe_dump(config, sort_keys=False),
                encoding="utf-8",
            )

            hand = OrcaHand(model_path)
            client = _RecordingMotorClient(active_motor_ids=hand.active_motor_ids)
            hand._motor_client = client

            hand.enable_torque()
            hand.set_control_mode("current_based_position")
            hand.set_max_current(123)
            hand.disable_torque()

            self.assertIn(8, hand.disabled_motor_ids)
            self.assertNotIn(8, hand.active_motor_ids)
            self.assertEqual(client.torque_calls[0], (tuple(hand.active_motor_ids), True))
            self.assertNotIn(8, client.operating_mode_calls[0][0])
            self.assertNotIn(8, client.current_writes[0][0])
            self.assertNotIn(8, client.torque_calls[-1][0])

    def test_active_motor_client_reads_expand_to_configured_motor_order(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            source = REPO_ROOT / "models" / "orca_hand_helios_lower_right"
            model_path = Path(tmp_dir) / "orca_hand_disabled"
            shutil.copytree(source, model_path)

            config_path = model_path / "config.yaml"
            config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
            config["disabled_motor_ids"] = [8]
            config_path.write_text(
                yaml.safe_dump(config, sort_keys=False),
                encoding="utf-8",
            )

            hand = OrcaHand(model_path)
            hand._motor_client = _RecordingMotorClient(active_motor_ids=hand.active_motor_ids)

            positions = hand.get_motor_pos(as_dict=True)
            currents = hand.get_motor_current(as_dict=True)
            temps = hand.get_motor_temp(as_dict=True)

            self.assertEqual(set(positions), set(hand.motor_ids))
            self.assertEqual(positions[8], 0.0)
            self.assertEqual(currents[8], 0.0)
            self.assertEqual(temps[8], 0.0)
            self.assertEqual(positions[9], 9.0)
            self.assertEqual(currents[9], 90.0)
            self.assertEqual(temps[9], 39.0)

    def test_feetech_runtime_convention_helpers(self):
        client = FeetechClient([1], port="/dev/null")
        try:
            raw_position = 100
            runtime_position = raw_position * client.pos_scale * POSITION_DIRECTION

            self.assertEqual(POSITION_DIRECTION, -1)
            self.assertFalse(client.requires_offset_calibration)
            self.assertEqual(client._position_raw_to_rad(raw_position), runtime_position)
            self.assertEqual(client._position_rad_to_raw(runtime_position), raw_position)
            self.assertEqual(client._offset_calibration_target(upper=True), 500)
            self.assertEqual(client._offset_calibration_target(upper=False), 3595)
            self.assertEqual(client._torque_from_current(-1200), 1000)
            self.assertEqual(client._torque_from_current(250), 250)
            self.assertEqual(client._default_speed, 150)
            self.assertEqual(client._default_acc, 50)
        finally:
            client.disconnect()


if __name__ == "__main__":
    unittest.main()
