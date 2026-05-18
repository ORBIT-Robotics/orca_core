#!/usr/bin/env python3
"""Debug-only HELIOS head runtime scaffold.

Current behavior:
- subscribes to `/helios/head/command`
- republishes latest command as `/helios/head/state`

The hardware runtime lives in `ros2_ws/src/helios_head_hardware_interface`.
This helper is only a no-hardware ROS echo for local wiring checks.
"""

from __future__ import annotations

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class HeliosHeadRuntime(Node):
    def __init__(self, cmd_topic: str, state_topic: str, rate_hz: float):
        super().__init__("helios_head_runtime")
        self._cmd_topic = cmd_topic
        self._state_topic = state_topic
        self._head_cmd = np.zeros(3, dtype=float)
        self._rx_count = 0
        self._warned_invalid = False

        self._sub_cmd = self.create_subscription(
            Float64MultiArray,
            self._cmd_topic,
            self._on_cmd,
            10,
        )
        self._pub_state = self.create_publisher(Float64MultiArray, self._state_topic, 10)
        self._timer_state = self.create_timer(max(1e-3, 1.0 / float(rate_hz)), self._publish_state)
        self._timer_log = self.create_timer(1.0, self._log_status)

        self.get_logger().info(
            f"HELIOS head runtime started | cmd={self._cmd_topic} state={self._state_topic} rate={rate_hz:.1f}Hz"
        )

    def _on_cmd(self, msg: Float64MultiArray) -> None:
        cmd = np.asarray(msg.data, dtype=float).reshape(-1)
        if cmd.size != 3 or not np.all(np.isfinite(cmd)):
            if not self._warned_invalid:
                self.get_logger().warning(f"Ignoring invalid head command payload (len={cmd.size}).")
                self._warned_invalid = True
            return

        self._head_cmd[:] = cmd
        self._rx_count += 1
        self._warned_invalid = False

    def _publish_state(self) -> None:
        self._pub_state.publish(Float64MultiArray(data=self._head_cmd.tolist()))

    def _log_status(self) -> None:
        self.get_logger().debug(
            f"HELIOS head runtime alive | rx={self._rx_count} last={self._head_cmd.tolist()}"
        )


def _parse_args():
    parser = argparse.ArgumentParser(description="HELIOS head runtime scaffold")
    parser.add_argument("--cmd-topic", default="/helios/head/command")
    parser.add_argument("--state-topic", default="/helios/head/state")
    parser.add_argument("--rate-hz", type=float, default=80.0)
    return parser.parse_args()


def main():
    args = _parse_args()
    rclpy.init()
    node = HeliosHeadRuntime(
        cmd_topic=args.cmd_topic,
        state_topic=args.state_topic,
        rate_hz=args.rate_hz,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
