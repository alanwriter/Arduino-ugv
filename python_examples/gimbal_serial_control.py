#!/usr/bin/env python3
"""Control the two-axis Nano SG90 gimbal through USB serial."""

from __future__ import annotations

import argparse
import time

import serial


class Gimbal:
    """Small reusable client for the `pan,tilt` Nano serial protocol."""

    def __init__(self, port: str, baud: int = 115200, reset_wait: float = 2.0) -> None:
        self.device = serial.Serial(port, baudrate=baud, timeout=5.0)
        # Opening a Nano USB serial port normally resets it.
        time.sleep(reset_wait)
        self.device.reset_input_buffer()

    def close(self) -> None:
        self.device.close()

    def command(self, text: str) -> str:
        self.device.write((text.strip() + "\n").encode("ascii"))
        self.device.flush()
        return self.device.readline().decode("utf-8", errors="replace").strip()

    def set_angles(self, pan: int, tilt: int) -> str:
        if not 0 <= pan <= 180 or not 0 <= tilt <= 180:
            raise ValueError("pan and tilt must both be in the range 0..180")
        return self.command(f"{pan},{tilt}")

    def center(self) -> str:
        return self.command("C")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Control a two-axis SG90 Nano gimbal.")
    parser.add_argument("--port", required=True, help="Nano port, e.g. /dev/cu.usbserial-120")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--set", nargs=2, type=int, metavar=("PAN", "TILT"),
                        help="send one angle pair and exit")
    parser.add_argument("--center", action="store_true", help="center both axes and exit")
    return parser.parse_args()


def interactive_mode(gimbal: Gimbal) -> None:
    print("Enter PAN,TILT (e.g. 120,45), P120, T45, C, or ?. Type q to quit.")
    while True:
        try:
            text = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if text.lower() in {"q", "quit", "exit"}:
            break
        if text:
            print(gimbal.command(text))


def main() -> None:
    args = parse_args()
    gimbal = Gimbal(args.port, args.baud)
    try:
        if args.set is not None:
            print(gimbal.set_angles(*args.set))
        elif args.center:
            print(gimbal.center())
        else:
            interactive_mode(gimbal)
    finally:
        gimbal.close()


if __name__ == "__main__":
    main()
