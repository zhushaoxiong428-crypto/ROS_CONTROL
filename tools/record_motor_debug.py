#!/usr/bin/env python3
"""Record the micro-ROS /motor_debug array to a timestamped CSV file."""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path


LEGACY_FIELDS = (
    "left_target_rpm",
    "left_actual_rpm",
    "left_raw_pwm",
    "left_final_pwm",
    "right_target_rpm",
    "right_actual_rpm",
    "right_raw_pwm",
    "right_final_pwm",
    "sync_error_rpm",
    "sync_correction_pwm",
    "sync_active",
    "startup_active",
    "startup_fault",
)
TIMING_FIELDS = (
    "control_seq_mod65536",
    "control_time_ms_hi16",
    "control_time_ms_lo16",
    "publish_seq_mod65536",
    "publish_time_ms_hi16",
    "publish_time_ms_lo16",
)
FIELDS = LEGACY_FIELDS + TIMING_FIELDS

# Keep the active-trial view reasonably fresh.  Passive diagnostics can raise
# this with --qos-depth; the phone-hotspot A/B used 100 without changing the
# firmware or the command path.
DEFAULT_MOTOR_DEBUG_QOS_DEPTH = 10


def motor_debug_values(data, *, require_timing: bool = False) -> tuple[float, ...]:
    """Accept exact legacy/new schemas; never invent MCU timing fields."""
    allowed_lengths = (len(FIELDS),) if require_timing else (len(LEGACY_FIELDS), len(FIELDS))
    if len(data) not in allowed_lengths:
        expected = " or ".join(str(length) for length in allowed_lengths)
        raise ValueError(f"/motor_debug needs {expected} values, got {len(data)}")
    values = tuple(float(value) for value in data)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("/motor_debug contains a non-finite value")
    if len(values) == len(FIELDS):
        for value in values[len(LEGACY_FIELDS):]:
            if not 0.0 <= value <= 65535.0 or not value.is_integer():
                raise ValueError("/motor_debug MCU timing fields must be uint16 parts")
    return values


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default="/motor_debug")
    parser.add_argument(
        "--qos-depth",
        type=int,
        default=DEFAULT_MOTOR_DEBUG_QOS_DEPTH,
        help=(
            "subscriber history depth (use a larger value only for passive "
            "high-jitter-link diagnostics)"
        ),
    )
    args = parser.parse_args()
    if args.qos_depth <= 0:
        parser.error("--qos-depth must be positive")

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float32MultiArray
    except ImportError as error:
        parser.error(f"ROS 2 Python packages are unavailable: {error}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    output_stream = args.output.open("x", encoding="utf-8", newline="", buffering=1)
    writer = csv.writer(output_stream)
    writer.writerow(("time_s", *FIELDS))
    start_time = time.monotonic()

    class Recorder(Node):
        def __init__(self) -> None:
            super().__init__("motor_debug_recorder")
            debug_qos = QoSProfile(
                depth=args.qos_depth,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.create_subscription(Float32MultiArray, args.topic, self.on_message, debug_qos)

        def on_message(self, message: Float32MultiArray) -> None:
            try:
                values = motor_debug_values(message.data)
            except ValueError as error:
                self.get_logger().error(str(error))
                return
            writer.writerow((time.monotonic() - start_time, *values,
                             *("" for _ in range(len(FIELDS) - len(values)))))

    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ROS 2 may already have shut the context down on SIGINT.
        if rclpy.ok():
            rclpy.shutdown()
        output_stream.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
