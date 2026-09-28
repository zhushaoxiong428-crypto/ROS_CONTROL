#!/usr/bin/env python3
"""Run one bounded, suspended-wheel startup trial and capture aligned ROS data.

This script publishes /cmd_vel and records the transmit events, /motor_debug,
and /battery_state against one host monotonic clock. The 19-field motor telemetry
also carries MCU control and publish sequence/timestamp fields, so wheel-response
timing can be reconstructed even when Wi-Fi delivers messages in bursts.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

from record_motor_debug import (
    DEFAULT_MOTOR_DEBUG_QOS_DEPTH,
    FIELDS,
    motor_debug_values,
)

WHEEL_DIAMETER_MM = 65.0
TRIAL_BATTERY_MIN_V = 6.4
TRIAL_BATTERY_MAX_V = 8.5
BATTERY_STABLE_NS = 1_000_000_000
BATTERY_FRESH_NS = 1_000_000_000
INTERLOCK_REARM_ZERO_SECONDS = 0.3


@dataclass(frozen=True)
class TrialSettings:
    speed_mps: float = 0.10
    drive_seconds: float = 3.0
    zero_seconds: float = 1.0
    rate_hz: float = 10.0
    wait_seconds: float = 10.0

    def validate(self) -> None:
        if not math.isfinite(self.speed_mps) or not 0.0 < self.speed_mps <= 0.10:
            raise ValueError("speed must be in (0, 0.10] m/s for this suspended-wheel trial")
        if not math.isfinite(self.drive_seconds) or not 0.5 <= self.drive_seconds <= 10.0:
            raise ValueError("drive duration must be between 0.5 and 10 seconds")
        if not math.isfinite(self.zero_seconds) or not 1.0 <= self.zero_seconds <= 10.0:
            raise ValueError("zero-command duration must be between 1 and 10 seconds")
        if not math.isfinite(self.rate_hz) or not 5.0 <= self.rate_hz <= 50.0:
            raise ValueError("rate must be between 5 and 50 Hz")
        if not math.isfinite(self.wait_seconds) or self.wait_seconds <= 0.0:
            raise ValueError("warm-up timeout must be positive")


def battery_voltage_error(voltage_v: float) -> str | None:
    """Validate the fixed 2S battery range used by this robot."""
    if not math.isfinite(voltage_v):
        return "battery voltage is not finite"
    if voltage_v < TRIAL_BATTERY_MIN_V:
        return (
            f"battery voltage {voltage_v:.3f} V is below the "
            f"{TRIAL_BATTERY_MIN_V:.1f} V motion-test limit"
        )
    if voltage_v > TRIAL_BATTERY_MAX_V:
        return (
            f"battery voltage {voltage_v:.3f} V exceeds the "
            f"{TRIAL_BATTERY_MAX_V:.1f} V plausible 2S limit"
        )
    return None


def battery_message_error(voltage_v: float, sample_present: bool) -> str | None:
    """Validate both the reported voltage and MCU sample freshness/validity."""
    if not sample_present:
        return "battery state reports no fresh valid ADC sample"
    return battery_voltage_error(voltage_v)


def battery_readiness_error(
    voltage_v: float | None,
    valid_since_ns: int | None,
    last_valid_ns: int | None,
    now_ns: int,
) -> str | None:
    if voltage_v is None:
        return "no battery voltage received"
    error = battery_voltage_error(voltage_v)
    if error is not None:
        return error
    if last_valid_ns is None or now_ns - last_valid_ns > BATTERY_FRESH_NS:
        return "battery telemetry has not provided an in-range sample for over 1 second"
    if valid_since_ns is None or now_ns - valid_since_ns < BATTERY_STABLE_NS:
        return "battery voltage has not remained in range for 1 second"
    return None


def next_battery_valid_since_ns(
    voltage_v: float,
    previous_valid_since_ns: int | None,
    previous_valid_sample_ns: int | None,
    now_ns: int,
) -> int | None:
    if battery_voltage_error(voltage_v) is not None:
        return None
    if (
        previous_valid_since_ns is None or
        previous_valid_sample_ns is None or
        now_ns - previous_valid_sample_ns > BATTERY_FRESH_NS
    ):
        return now_ns
    return previous_valid_since_ns


def motion_command_safety_error(speed_mps: float, health_error: str | None) -> str | None:
    """Reject malformed commands and every unhealthy nonzero command."""
    if not math.isfinite(speed_mps):
        return "commanded speed is not finite"
    if speed_mps != 0.0 and health_error is not None:
        return health_error
    return None


def startup_fault_rearm_error(
    rearm_start_ns: int,
    last_motor_ns: int | None,
    startup_fault_active: bool,
) -> str | None:
    """Require a new fault-free MCU sample after the explicit re-arm zero."""
    if last_motor_ns is None or last_motor_ns <= rearm_start_ns:
        return "no fresh motor telemetry received after interlock re-arm zero"
    if startup_fault_active:
        return "startup fault did not clear after interlock re-arm zero"
    return None


def stop_verification_error(
    last_motor: tuple[float, ...] | None,
    last_motor_ns: int | None,
    last_drive_tx_ns: int | None,
    first_zero_tx_ns: int | None,
    zero_target_pwm_ns: int | None,
    now_ns: int,
    subscriber_count: int,
    cmd_publisher_count: int,
) -> str | None:
    """Require early target/PWM zero evidence, then fresh settled wheel speeds."""
    if last_drive_tx_ns is None or first_zero_tx_ns is None or zero_target_pwm_ns is None:
        return "no target/PWM-zero telemetry observed after the explicit zero command"
    if zero_target_pwm_ns <= first_zero_tx_ns:
        return "target/PWM-zero telemetry was not received after the zero command"
    if zero_target_pwm_ns - last_drive_tx_ns >= 400_000_000:
        return "target/PWM-zero observation is too late to distinguish zero command from watchdog"
    if last_motor is None or last_motor_ns is None or now_ns - last_motor_ns > 750_000_000:
        return "fresh motor telemetry was not available after stopping"
    if subscriber_count <= 0:
        return "/cmd_vel subscriber disconnected during stop phase"
    if cmd_publisher_count != 1:
        return "another /cmd_vel publisher may be competing with the trial"
    if any(abs(last_motor[index]) >= 0.1 for index in (0, 3, 4, 7)):
        return "wheel targets or PWM did not return to zero"
    if abs(last_motor[1]) >= 1.0 or abs(last_motor[5]) >= 1.0:
        return "filtered wheel speeds did not settle below 1 rpm"
    return None


def battery_summary(
    readings: list[tuple[int, float]],
    first_drive_tx_ns: int | None,
    first_zero_tx_ns: int | None,
) -> dict[str, float | None]:
    """Summarise received voltage; ADC samples may be repeated by ROS."""
    if first_drive_tx_ns is None or first_zero_tx_ns is None:
        return {
            "battery_before_drive_v": None,
            "battery_min_during_drive_v": None,
            "battery_after_stop_v": None,
        }
    before = [value for stamp, value in readings if stamp < first_drive_tx_ns]
    during = [value for stamp, value in readings if first_drive_tx_ns <= stamp < first_zero_tx_ns]
    after = [value for stamp, value in readings if stamp >= first_zero_tx_ns]
    return {
        "battery_before_drive_v": before[-1] if before else None,
        "battery_min_during_drive_v": min(during) if during else None,
        "battery_after_stop_v": after[-1] if after else None,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path, help="new directory for CSVs and metadata")
    parser.add_argument("--firmware-id", required=True, help="explicit firmware/build label")
    parser.add_argument("--speed-mps", type=float, default=0.10)
    parser.add_argument("--drive-seconds", type=float, default=3.0)
    parser.add_argument("--zero-seconds", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--wait-seconds", type=float, default=10.0)
    parser.add_argument(
        "--wheels-suspended",
        action="store_true",
        required=True,
        help="confirm the chassis is secured with both drive wheels clear",
    )
    args = parser.parse_args()
    settings = TrialSettings(
        speed_mps=args.speed_mps,
        drive_seconds=args.drive_seconds,
        zero_seconds=args.zero_seconds,
        rate_hz=args.rate_hz,
        wait_seconds=args.wait_seconds,
    )
    try:
        settings.validate()
    except ValueError as error:
        parser.error(str(error))
    args.settings = settings
    if not args.firmware_id.strip():
        parser.error("--firmware-id must not be blank")
    return args


def main() -> int:
    args = parse_args()

    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import BatteryState
        from std_msgs.msg import Float32MultiArray
    except ImportError as error:
        raise SystemExit(f"ROS 2 Python packages are unavailable: {error}") from error

    # Refuse to overwrite evidence from any earlier run.
    args.output_dir.mkdir(parents=True, exist_ok=False)
    motor_path = args.output_dir / "motor_debug.csv"
    events_path = args.output_dir / "events.csv"
    metadata_path = args.output_dir / "metadata.json"
    origin_ns = time.monotonic_ns()
    metadata = {
        "schema_version": 2,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "clock": "host time.monotonic_ns relative to trial start",
        "firmware_id": args.firmware_id,
        "settings": asdict(args.settings),
        "safety_limits": {
            "battery_min_inclusive_v": TRIAL_BATTERY_MIN_V,
            "battery_max_inclusive_v": TRIAL_BATTERY_MAX_V,
            "battery_stable_seconds": BATTERY_STABLE_NS / 1e9,
            "battery_freshness_seconds": BATTERY_FRESH_NS / 1e9,
            "interlock_rearm_zero_seconds": INTERLOCK_REARM_ZERO_SECONDS,
        },
        "status": "aborted",
        "reason": "trial did not finish",
    }

    with motor_path.open("w", encoding="utf-8", newline="", buffering=1) as motor_stream, \
            events_path.open("w", encoding="utf-8", newline="", buffering=1) as events_stream:
        motor_writer = csv.writer(motor_stream)
        event_writer = csv.writer(events_stream)
        motor_writer.writerow(("time_s", *FIELDS, "battery_voltage_v"))
        event_writer.writerow((
            "time_s", "event", "cmd_linear_x_mps", "battery_voltage_v",
            "battery_header_stamp_s", "detail",
        ))

        def elapsed_s(now_ns: int | None = None) -> float:
            return ((time.monotonic_ns() if now_ns is None else now_ns) - origin_ns) / 1e9

        class TrialNode(Node):
            def __init__(self) -> None:
                super().__init__("motor_startup_trial")
                self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
                debug_qos = QoSProfile(
                    depth=DEFAULT_MOTOR_DEBUG_QOS_DEPTH,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                )
                self.create_subscription(
                    Float32MultiArray, "/motor_debug", self.on_motor_debug, debug_qos,
                )
                self.create_subscription(
                    BatteryState, "/battery_state", self.on_battery,
                    qos_profile_sensor_data,
                )
                self.last_motor_ns: int | None = None
                self.last_motor: tuple[float, ...] | None = None
                self.last_battery_ns: int | None = None
                self.battery_voltage_v: float | None = None
                self.battery_valid_since_ns: int | None = None
                self.idle_since_ns: int | None = None
                self.invalid_motor_data = False
                self.startup_fault_active = False
                self.trial_fault_monitoring = False
                self.startup_fault_during_trial = False
                self.motor_count = 0
                self.battery_count = 0
                self.battery_readings: list[tuple[int, float]] = []
                self.cmd_count = 0
                self.first_drive_tx_ns: int | None = None
                self.last_drive_tx_ns: int | None = None
                self.first_zero_tx_ns: int | None = None
                self.zero_target_pwm_ns: int | None = None
                self.drive_target_seen = False
                self.drive_plateau_seen = False
                self.both_wheels_moved = False

            def on_motor_debug(self, message: Float32MultiArray) -> None:
                now_ns = time.monotonic_ns()
                try:
                    values = motor_debug_values(message.data, require_timing=True)
                except ValueError as error:
                    self.invalid_motor_data = True
                    event_writer.writerow((elapsed_s(now_ns), "telemetry_error", "", "", "", str(error)))
                    return
                self.last_motor_ns = now_ns
                self.last_motor = values
                self.motor_count += 1
                self.startup_fault_active = values[12] > 0.5
                if self.trial_fault_monitoring:
                    self.startup_fault_during_trial |= self.startup_fault_active
                if self.first_drive_tx_ns is not None and self.first_zero_tx_ns is None:
                    self.drive_target_seen |= values[0] > 0.5 and values[4] > 0.5
                    target_rpm = args.settings.speed_mps * 1000.0 * 60.0 / (math.pi * WHEEL_DIAMETER_MM)
                    self.drive_plateau_seen |= values[0] >= 0.9 * target_rpm and values[4] >= 0.9 * target_rpm
                    self.both_wheels_moved |= values[1] >= 1.0 and values[5] >= 1.0
                if (
                    self.first_zero_tx_ns is not None and now_ns > self.first_zero_tx_ns and
                    all(abs(values[index]) < 0.1 for index in (0, 3, 4, 7)) and
                    self.zero_target_pwm_ns is None
                ):
                    self.zero_target_pwm_ns = now_ns
                if (
                    abs(values[0]) < 0.1 and abs(values[3]) < 0.1 and
                    abs(values[4]) < 0.1 and abs(values[7]) < 0.1 and
                    abs(values[1]) < 1.0 and abs(values[5]) < 1.0
                ):
                    if self.idle_since_ns is None:
                        self.idle_since_ns = now_ns
                else:
                    self.idle_since_ns = None
                motor_writer.writerow((
                    elapsed_s(now_ns), *values,
                    "" if self.battery_voltage_v is None else self.battery_voltage_v,
                ))

            def on_battery(self, message: BatteryState) -> None:
                now_ns = time.monotonic_ns()
                voltage = float(message.voltage)
                self.battery_voltage_v = voltage
                error = battery_message_error(voltage, bool(message.present))
                if error is not None:
                    self.battery_valid_since_ns = None
                    event_writer.writerow((
                        elapsed_s(now_ns), "battery_error", "", voltage, "", error,
                    ))
                    return
                self.battery_valid_since_ns = next_battery_valid_since_ns(
                    voltage,
                    self.battery_valid_since_ns,
                    self.last_battery_ns,
                    now_ns,
                )
                self.last_battery_ns = now_ns
                self.battery_count += 1
                self.battery_readings.append((now_ns, voltage))
                stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
                event_writer.writerow((elapsed_s(now_ns), "battery_rx", "", voltage, stamp, ""))

            def publish_cmd(
                self,
                speed_mps: float,
                *,
                record_stop_phase: bool = True,
                detail: str = "",
            ) -> None:
                # Keep the safety invariant at the only method that can publish:
                # no caller can accidentally bypass health checks for motion.
                now_ns = time.monotonic_ns()
                error = motion_command_safety_error(
                    speed_mps,
                    self.health_error(now_ns) if speed_mps != 0.0 else None,
                )
                if error is not None:
                    raise RuntimeError(error)
                command = Twist()
                command.linear.x = speed_mps
                self.cmd_publisher.publish(command)
                now_ns = time.monotonic_ns()
                if speed_mps > 0.0:
                    if self.first_drive_tx_ns is None:
                        self.first_drive_tx_ns = now_ns
                    self.last_drive_tx_ns = now_ns
                elif record_stop_phase and self.first_zero_tx_ns is None:
                    self.first_zero_tx_ns = now_ns
                self.cmd_count += 1
                event_writer.writerow((elapsed_s(now_ns), "cmd_vel_tx", speed_mps, "", "", detail))

            def health_error(self, now_ns: int) -> str | None:
                if self.invalid_motor_data:
                    return "incompatible /motor_debug schema (19 MCU-timed fields required)"
                if self.startup_fault_during_trial:
                    return "startup fault reported by firmware"
                if self.last_motor_ns is None or now_ns - self.last_motor_ns > 750_000_000:
                    return "motor telemetry missing for over 750 ms"
                battery_error = battery_readiness_error(
                    self.battery_voltage_v,
                    self.battery_valid_since_ns,
                    self.last_battery_ns,
                    now_ns,
                )
                if battery_error is not None:
                    return battery_error
                if self.cmd_publisher.get_subscription_count() <= 0:
                    return "/cmd_vel subscriber disconnected"
                if self.count_publishers("/cmd_vel") != 1:
                    return "another /cmd_vel publisher is active"
                if self.first_drive_tx_ns is not None:
                    drive_elapsed_ns = now_ns - self.first_drive_tx_ns
                    if drive_elapsed_ns > 700_000_000 and not self.drive_target_seen:
                        return "firmware did not adopt the nonzero /cmd_vel target"
                    if drive_elapsed_ns > 1_000_000_000 and not self.both_wheels_moved:
                        return "both wheels did not reach 1 rpm during the drive phase"
                    if drive_elapsed_ns > 1_000_000_000 and not self.drive_plateau_seen:
                        return "firmware wheel targets did not reach the commanded plateau"
                    if (
                        drive_elapsed_ns > 500_000_000 and self.drive_target_seen and
                        self.last_motor is not None and
                        (self.last_motor[0] < 0.5 or self.last_motor[4] < 0.5)
                    ):
                        return "wheel target dropped while nonzero commands were still being sent"
                if self.last_motor is not None:
                    left_rpm, right_rpm = self.last_motor[1], self.last_motor[5]
                    max_rpm = args.settings.speed_mps * 1000.0 * 60.0 / (math.pi * WHEEL_DIAMETER_MM)
                    if left_rpm < -1.0 or right_rpm < -1.0:
                        return "encoder reports motion opposite the forward command"
                    if max(left_rpm, right_rpm) > 1.5 * max_rpm:
                        return "wheel speed exceeded 1.5 times the commanded speed"
                    if abs(left_rpm - right_rpm) > 10.0:
                        return "wheel speed difference exceeded 10 rpm"
                return None

        rclpy.init()
        node = TrialNode()
        drive_completed = False
        period_ns = round(1e9 / args.settings.rate_hz)

        def wait_until_ready() -> None:
            deadline_ns = time.monotonic_ns() + round(args.settings.wait_seconds * 1e9)
            while time.monotonic_ns() < deadline_ns:
                # spin_once dispatches one callback.  Keep the live safety view
                # shallow; complete startup evidence must come from a separate
                # MCU-side trace rather than stale queued live samples.
                rclpy.spin_once(node, timeout_sec=0.005)
                now_ns = time.monotonic_ns()
                if node.invalid_motor_data:
                    raise RuntimeError("/motor_debug must have 19 MCU-timed fields; flash the instrumented firmware first")
                if (
                    node.count_subscribers("/cmd_vel") > 0 and
                    node.count_publishers("/cmd_vel") == 1 and
                    node.idle_since_ns is not None and
                    now_ns - node.idle_since_ns >= 500_000_000 and
                    node.last_motor_ns is not None and
                    now_ns - node.last_motor_ns < 750_000_000 and
                    battery_readiness_error(
                        node.battery_voltage_v,
                        node.battery_valid_since_ns,
                        node.last_battery_ns,
                        now_ns,
                    ) is None
                ):
                    return
            battery_error = battery_readiness_error(
                node.battery_voltage_v,
                node.battery_valid_since_ns,
                node.last_battery_ns,
                time.monotonic_ns(),
            )
            if battery_error is not None:
                raise RuntimeError(f"battery not ready: {battery_error}")
            raise RuntimeError("no fresh idle motor data, battery data, or /cmd_vel subscriber")

        def publish_phase(
            speed_mps: float,
            duration_s: float,
            *,
            record_stop_phase: bool = True,
            monitor_zero_health: bool = False,
            detail: str = "",
        ) -> None:
            phase_end_ns = time.monotonic_ns() + round(duration_s * 1e9)
            next_publish_ns = time.monotonic_ns()
            while time.monotonic_ns() < phase_end_ns:
                now_ns = time.monotonic_ns()
                if now_ns >= next_publish_ns:
                    if monitor_zero_health:
                        error = node.health_error(now_ns)
                        if error:
                            raise RuntimeError(error)
                    node.publish_cmd(
                        speed_mps,
                        record_stop_phase=record_stop_phase,
                        detail=detail,
                    )
                    next_publish_ns += period_ns
                    if next_publish_ns <= now_ns:
                        next_publish_ns = now_ns + period_ns
                rclpy.spin_once(
                    node,
                    timeout_sec=min(0.005, max(0.0, (next_publish_ns - time.monotonic_ns()) / 1e9)),
                )
                if speed_mps != 0.0 or monitor_zero_health:
                    error = node.health_error(time.monotonic_ns())
                    if error:
                        raise RuntimeError(error)

        try:
            print("Waiting for idle /motor_debug, valid /battery_state, and /cmd_vel subscriber...")
            wait_until_ready()
            event_writer.writerow((
                elapsed_s(), "interlock_rearm_start", 0.0,
                node.battery_voltage_v, "", "explicit all-stop",
            ))
            rearm_start_ns = time.monotonic_ns()
            publish_phase(
                0.0,
                INTERLOCK_REARM_ZERO_SECONDS,
                record_stop_phase=False,
                monitor_zero_health=True,
                detail="interlock_rearm",
            )
            rearm_error = startup_fault_rearm_error(
                rearm_start_ns,
                node.last_motor_ns,
                node.startup_fault_active,
            )
            if rearm_error is not None:
                raise RuntimeError(rearm_error)
            event_writer.writerow((
                elapsed_s(), "interlock_rearm_complete", 0.0,
                node.battery_voltage_v, "", "",
            ))
            node.startup_fault_during_trial = False
            node.trial_fault_monitoring = True
            event_writer.writerow((elapsed_s(), "trial_start", "", node.battery_voltage_v, "", ""))
            print("Starting bounded suspended-wheel command; keep hardware power cutoff in reach.")
            publish_phase(args.settings.speed_mps, args.settings.drive_seconds)
            if not (node.drive_target_seen and node.drive_plateau_seen and node.both_wheels_moved):
                raise RuntimeError("commanded motion was not confirmed by motor telemetry")
            drive_completed = True
        except (KeyboardInterrupt, Exception) as error:
            metadata["reason"] = str(error) or "interrupted by operator"
            print(f"Trial aborted: {metadata['reason']}")
        finally:
            # Never insert a wait between the final drive sample and the first
            # explicit zero command. The firmware watchdog remains a fallback.
            zero_phase_completed = False
            try:
                node.publish_cmd(0.0)
                event_writer.writerow((elapsed_s(), "zero_phase_start", 0.0, "", "", ""))
                publish_phase(0.0, args.settings.zero_seconds)
                zero_phase_completed = True
            except (KeyboardInterrupt, Exception) as error:  # best effort, including ROS shutdown
                metadata["status"] = "aborted"
                metadata["reason"] = f"zero-command phase failed: {error}"
                print(metadata["reason"])
            if node.invalid_motor_data or node.startup_fault_during_trial:
                metadata["status"] = "aborted"
                metadata["reason"] = "invalid motor data or startup fault observed"
            elif drive_completed and zero_phase_completed:
                stop_error = stop_verification_error(
                    node.last_motor,
                    node.last_motor_ns,
                    node.last_drive_tx_ns,
                    node.first_zero_tx_ns,
                    node.zero_target_pwm_ns,
                    time.monotonic_ns(),
                    node.cmd_publisher.get_subscription_count(),
                    node.count_publishers("/cmd_vel"),
                )
                if stop_error is None:
                    metadata["status"] = "completed"
                    metadata["reason"] = "target/PWM zero observed before watchdog; wheel speeds later settled"
                else:
                    metadata["status"] = "aborted"
                    metadata["reason"] = stop_error
            if node.last_drive_tx_ns is not None and node.zero_target_pwm_ns is not None:
                target_pwm_observation_s = (node.zero_target_pwm_ns - node.last_drive_tx_ns) / 1e9
                metadata["last_drive_tx_to_zero_target_pwm_observed_s"] = target_pwm_observation_s
                metadata["zero_target_pwm_observed_inside_watchdog_window"] = target_pwm_observation_s < 0.5
            metadata["motor_samples"] = node.motor_count
            metadata["battery_samples"] = node.battery_count
            metadata["cmd_messages"] = node.cmd_count
            metadata.update(battery_summary(
                node.battery_readings, node.first_drive_tx_ns, node.first_zero_tx_ns,
            ))
            if node.last_drive_tx_ns is not None and node.first_zero_tx_ns is not None:
                metadata["last_drive_tx_to_first_zero_tx_s"] = (
                    node.first_zero_tx_ns - node.last_drive_tx_ns
                ) / 1e9
            metadata["finished_utc"] = datetime.now(timezone.utc).isoformat()
            event_writer.writerow((elapsed_s(), "trial_end", "", node.battery_voltage_v, "", metadata["status"]))
            node.destroy_node()
            rclpy.shutdown()
            metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(f"Trial {metadata['status']}: {metadata['reason']}")
    print(f"Data saved to {args.output_dir}")
    return 0 if metadata["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
