#!/usr/bin/env python3
"""Safe command-line client for the Leader 1 Nano USB I/O protocol."""

from __future__ import annotations

import argparse
import json
import sys
import time

import serial


RECORD_FIELDS = {
    "STATUS": (
        "sequence", "mode", "x_mm", "y_mm", "heading_deg", "left_count",
        "right_count", "left_tps", "right_tps", "left_target_tps",
        "right_target_tps", "left_pwm", "right_pwm", "fault_code",
        "encoder_preflight", "imu_present", "imu_calibrated", "path_number",
        "path_step",
    ),
    "TELEMETRY": (
        "sequence", "mode", "x_mm", "y_mm", "heading_deg", "left_count",
        "right_count", "left_tps", "right_tps", "left_target_tps",
        "right_target_tps", "left_pwm", "right_pwm", "fault_code",
        "encoder_preflight", "imu_present", "imu_calibrated", "path_number",
        "path_step",
    ),
    "CONFIG": (
        "sequence", "left_ticks_per_rev", "right_ticks_per_rev",
        "wheel_diameter_mm", "wheel_track_mm", "turn_pwm_bias", "max_pwm",
        "max_manual_hold_ms", "max_approved_path",
    ),
    "IMU": (
        "sequence", "present", "calibrated", "address", "roll_deg",
        "pitch_deg", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "bias_raw",
        "read_failures", "consecutive_read_failures",
    ),
    "ENCODER": (
        "sequence", "preflight", "left_a_level", "left_b_level",
        "right_a_level", "right_b_level", "left_count", "left_valid_edges",
        "left_a_edges", "left_b_edges", "left_invalid", "right_count",
        "right_valid_edges", "right_a_edges", "right_b_edges", "right_invalid",
    ),
}

SESSION_RESPONSE_TYPES = {
    "STATUS": {"STATUS"},
    "CONFIG": {"CONFIG"},
    "IMU": {"IMU"},
    "ENCODER": {"ENCODER"},
    "STOP": {"ACK"},
    "RESET": {"ACK"},
    "CALIBRATE": {"ACK"},
    "MOTOR": {"ACK"},
    "PATH": {"ACK"},
    "TELEMETRY": {"ACK"},
    "PING": {"PONG"},
}


class ProtocolError(RuntimeError):
    """The Nano rejected a syntactically valid I/O command."""


class NanoUsbIo:
    """Line-based Leader 1 USB client with sequence-matched replies."""

    def __init__(self, port: str, baud: int = 115200, boot_wait: float = 2.0) -> None:
        self.device = serial.Serial(port, baudrate=baud, timeout=0.15, write_timeout=1)
        self._sequence = 0
        # Opening a Nano USB CDC port normally resets the Nano. Do not send a
        # movement command during this safe boot period.
        time.sleep(boot_wait)
        self.device.reset_input_buffer()
        self.request("HELLO", "1", expected={"HELLO"})
        self.request("STOP", expected={"ACK"})

    def close(self) -> None:
        """Attempt a stop before relinquishing the serial port."""
        if not self.device.is_open:
            return
        try:
            self.request("STOP", expected={"ACK"}, timeout=1.0)
        except (ProtocolError, serial.SerialException, TimeoutError):
            pass
        self.device.close()

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence % 65535) + 1
        return self._sequence

    @staticmethod
    def _parse_line(raw: bytes) -> list[str] | None:
        text = raw.decode("ascii", errors="replace").strip()
        if not text:
            return None
        if not text.startswith("IO,"):
            # Existing human diagnostic lines may coexist with this protocol.
            print(f"Nano diagnostic: {text}", file=sys.stderr)
            return None
        return text.split(",")

    def request(
        self, operation: str, *arguments: str, expected: set[str], timeout: float = 3.0
    ) -> list[str]:
        sequence = self._next_sequence()
        command = ",".join(("IO", str(sequence), operation, *arguments)) + "\n"
        self.device.write(command.encode("ascii"))
        self.device.flush()

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            record = self._parse_line(self.device.readline())
            if record is None or len(record) < 3:
                continue
            if record[1] == "ERROR" and record[2] == str(sequence):
                detail = record[3] if len(record) > 3 else "UNKNOWN"
                raise ProtocolError(f"{operation} rejected by Nano: {detail}")
            if record[1] in expected and record[2] == str(sequence):
                return record
        raise TimeoutError(f"No {sorted(expected)} reply for {operation} sequence {sequence}")

    def next_telemetry(self, timeout: float = 1.0) -> list[str] | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            record = self._parse_line(self.device.readline())
            if record and len(record) >= 3 and record[1] == "TELEMETRY" and record[2] == "0":
                return record
        return None


def record_as_dict(record: list[str]) -> dict[str, str]:
    record_type = record[1]
    fields = RECORD_FIELDS.get(record_type)
    if fields is None:
        return {"record": ",".join(record)}
    return dict(zip(fields, record[2:], strict=True))


def print_record(record: list[str]) -> None:
    print(json.dumps(record_as_dict(record), ensure_ascii=False, indent=2))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Control Leader 1 through Nano USB serial.")
    parser.add_argument("--port", required=True, help="Use /dev/serial/by-id/... on Raspberry Pi")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--boot-wait", type=float, default=2.0)
    subparsers = parser.add_subparsers(dest="action", required=True)
    for name in ("status", "config", "imu", "encoders", "stop", "reset", "calibrate"):
        subparsers.add_parser(name)

    drive = subparsers.add_parser("drive", help="PWM command; Nano stops when hold time expires")
    drive.add_argument("left_pwm", type=int)
    drive.add_argument("right_pwm", type=int)
    drive.add_argument("--hold-ms", type=int, default=500)

    path = subparsers.add_parser("path", help="Run an approved preset path while monitoring it")
    path.add_argument("number", type=int, choices=(1, 2, 3))
    path.add_argument("--timeout", type=float, default=90.0)
    path.add_argument("--telemetry-ms", type=int, default=500)

    telemetry = subparsers.add_parser("telemetry", help="Print subscribed telemetry for a duration")
    telemetry.add_argument("--interval-ms", type=int, default=500)
    telemetry.add_argument("--seconds", type=float, default=10.0)
    subparsers.add_parser(
        "session",
        help="Keep one USB connection open for calibration, encoder checks, and driving",
    )
    return parser.parse_args()


def require_range(value: int, minimum: int, maximum: int, name: str) -> None:
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be in {minimum}..{maximum}")


def print_telemetry_for(client: NanoUsbIo, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        record = client.next_telemetry(timeout=min(1.0, deadline - time.monotonic()))
        if record:
            print_record(record)


def interactive_session(client: NanoUsbIo) -> None:
    """Run a single persistent USB session so Nano state is not reset."""
    print(
        "Persistent Nano session. Commands: status, config, imu, encoder, stop, "
        "reset, calibrate, motor L R HOLD_MS, path N, telemetry MS, "
        "watch SECONDS, ping, quit."
    )
    while True:
        try:
            words = input("leader> ").strip().split()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not words:
            continue
        command = words[0].upper()
        if command in {"QUIT", "EXIT", "Q"}:
            return
        if command in {"HELP", "?"}:
            print(
                "status | config | imu | encoder | stop | reset | calibrate | "
                "motor <left_pwm> <right_pwm> <hold_ms> | path <1|2|3> | "
                "telemetry <0|100..2000> | watch <seconds> | ping | quit"
            )
            continue
        try:
            if command == "WATCH":
                if len(words) != 2:
                    raise ValueError("watch requires a duration in seconds")
                print_telemetry_for(client, float(words[1]))
                continue
            if command == "ENCODER":
                command = "ENCODER"
            if command == "MOTOR":
                if len(words) != 4:
                    raise ValueError("motor requires: left_pwm right_pwm hold_ms")
                require_range(int(words[1]), -165, 165, "left_pwm")
                require_range(int(words[2]), -165, 165, "right_pwm")
                require_range(int(words[3]), 50, 1200, "hold_ms")
            elif command == "PATH":
                if len(words) != 2:
                    raise ValueError("path requires: 1, 2, or 3")
                require_range(int(words[1]), 1, 3, "path_number")
            elif command == "TELEMETRY":
                if len(words) != 2:
                    raise ValueError("telemetry requires an interval in milliseconds")
                interval = int(words[1])
                if interval != 0:
                    require_range(interval, 100, 2000, "telemetry interval")
            elif command in {"STATUS", "CONFIG", "IMU", "ENCODER", "STOP", "RESET",
                             "CALIBRATE", "PING"}:
                if len(words) != 1:
                    raise ValueError(f"{command.lower()} takes no arguments")
            else:
                raise ValueError("unknown command; type help")

            timeout = 5.0 if command == "CALIBRATE" else 3.0
            print_record(client.request(
                command, *words[1:], expected=SESSION_RESPONSE_TYPES[command], timeout=timeout
            ))
        except (ProtocolError, TimeoutError, ValueError) as error:
            print(f"ERROR: {error}", file=sys.stderr)


def main() -> None:
    args = parse_args()
    client = NanoUsbIo(args.port, args.baud, args.boot_wait)
    try:
        if args.action == "status":
            print_record(client.request("STATUS", expected={"STATUS"}))
        elif args.action == "config":
            print_record(client.request("CONFIG", expected={"CONFIG"}))
        elif args.action == "imu":
            print_record(client.request("IMU", expected={"IMU"}))
        elif args.action == "encoders":
            print_record(client.request("ENCODER", expected={"ENCODER"}))
        elif args.action == "stop":
            print_record(client.request("STOP", expected={"ACK"}))
        elif args.action == "reset":
            print_record(client.request("RESET", expected={"ACK"}))
        elif args.action == "calibrate":
            print_record(client.request("CALIBRATE", expected={"ACK"}, timeout=5.0))
        elif args.action == "drive":
            require_range(args.left_pwm, -165, 165, "left_pwm")
            require_range(args.right_pwm, -165, 165, "right_pwm")
            require_range(args.hold_ms, 50, 1200, "hold_ms")
            print_record(client.request(
                "MOTOR", str(args.left_pwm), str(args.right_pwm), str(args.hold_ms), expected={"ACK"}
            ))
            # Keep the process alive for the requested pulse. close() sends a
            # final STOP as a belt-and-suspenders safety measure.
            time.sleep(args.hold_ms / 1000.0)
            print_record(client.request("STATUS", expected={"STATUS"}))
        elif args.action == "path":
            require_range(args.telemetry_ms, 100, 2000, "telemetry_ms")
            print_record(client.request("PATH", str(args.number), expected={"ACK"}))
            client.request("TELEMETRY", str(args.telemetry_ms), expected={"ACK"})
            print_telemetry_for(client, args.timeout)
        elif args.action == "telemetry":
            require_range(args.interval_ms, 100, 2000, "interval_ms")
            client.request("TELEMETRY", str(args.interval_ms), expected={"ACK"})
            print_telemetry_for(client, args.seconds)
        elif args.action == "session":
            interactive_session(client)
    finally:
        client.close()


if __name__ == "__main__":
    try:
        main()
    except (ProtocolError, TimeoutError, ValueError, serial.SerialException) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2) from error
