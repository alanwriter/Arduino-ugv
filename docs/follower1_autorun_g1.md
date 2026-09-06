# Follower 1 — one-shot autonomous G1 demo

This is an autonomous demo image specifically for **Follower 1**. It carries
Follower 1's verified pin map and calibration values, so it must not be flashed
to Follower 2 or Leader 1.

## Relationship to the full Follower 1 controller

| Profile | PlatformIO environment | Purpose | How motion starts |
| --- | --- | --- | --- |
| Follower 1 full controller | `nanoatmega328` | Manual motor checks, encoder/MPU diagnostics, calibration and `G1`/`G2`/`G3`. | A serial command from a Mac or Raspberry Pi. |
| Follower 1 demo | `nanoatmega328_follower1_demo_g1` | One autonomous straight-line demonstration. | Each Nano power cycle/reset. |

The demo uses the same PID wheel control, odometry, MPU6050 yaw fusion and
fault handling as the full controller. Its separate entry point is
`src/follower1_demo_g1.cpp`; it enables only the Follower-1-demo power-on state
machine in `src/main.cpp`.

```text
power/reset
  -> motor outputs forced off
  -> 3 s stillness delay
  -> 300-sample MPU6050 gyro-Z calibration (about 1.5 s)
  -> reset encoder counts, pose and PID history
  -> run G1: 500 mm forward, closed-loop speed and heading control
  -> complete or fault: motor outputs off until the next power cycle
```

There are no serial motion commands in this demo. Serial at 115200 baud is
only optional observation of its boot message and faults.

## Safety and certification

The demo marks encoder preflight as certified before G1 because an autonomous
image cannot ask a person to inspect the test. That bypass applies only to this
Follower 1 demo build, after the full Follower 1 profile has confirmed:

- both wheels move physically forward under `F`;
- both logical encoder counts increase during forward motion;
- both A/B phases have balanced valid edge counts; and
- MPU6050 at `0x68` completes gyro-Z calibration.

Runtime I2C/MPU, wheel-stall, path-stall and timeout protection remains active.
Use a reachable physical motor-power switch as the emergency stop.

## Build and run

Select **`nanoatmega328_follower1_demo_g1`** in PlatformIO, or run:

```sh
platformio run -e nanoatmega328_follower1_demo_g1 --target upload
```

For a floor run, leave at least 1 m clear ahead. After power-on, do not touch
the vehicle for roughly 4.5 seconds: it waits three seconds, then calibrates
the gyro before driving its one 500 mm segment. It stays stopped after the
attempt. Turn motor power off before handling it.

## Follower 1 values embedded in this demo

- Left encoder: 1203.5 decoded ticks/revolution, measured over 10 marked turns.
- Right encoder: 1200 ticks/revolution is still provisional.
- Wheel diameter: 65 mm; track width: 130 mm.
- Logical forward uses `RIGHT_ENCODER_REVERSED = true`.

Follower 2 and Leader 1 need their own measurements and their own autonomous
demo builds after their calibration profiles pass.
