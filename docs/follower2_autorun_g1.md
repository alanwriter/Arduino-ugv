# F2 — one-shot autonomous G1 test

`follower2` is the next test profile, not a replacement for the full F1
controller. It is for the moment when the Nano is powered from a battery and
there is no Mac or Raspberry Pi connected.

## Architecture

| Profile | PlatformIO environment | Purpose | How motion starts |
| --- | --- | --- | --- |
| F1 | `nanoatmega328` | Full controller: manual motor checks, encoder/MPU diagnostics, calibration and `G1`/`G2`/`G3` commands. | A serial command from a Mac or Raspberry Pi. |
| F2 | `nanoatmega328_f2_autorun_g1` | A single autonomous straight-line test. | Each Nano power cycle/reset. |

F2 uses the same confirmed Nano #1 pin map, PID wheel control, odometry,
MPU6050 yaw fusion and fault handling as F1. It does **not** replace the F1
source file: `src/f2_autorun_g1.cpp` is a separate build entry point which
turns on the F2-only state machine in `src/main.cpp`.

The F2 power-on sequence is:

```text
power/reset
  -> motor outputs forced off
  -> 3 s stillness delay
  -> 300-sample MPU6050 gyro-Z calibration (about 1.5 s; vehicle must not move)
  -> reset encoder counts, pose and PID history
  -> run G1: 500 mm forward, closed-loop speed and heading control
  -> complete or fault: motor outputs off forever, until the next power cycle
```

F2 deliberately has no serial commands and never repeats the path by itself.
Serial output at 115200 baud remains useful for observing the boot message or a
fault, but is not required for the demo to run.

## Safety and the F1 certification requirement

F2 cannot ask a person to lift the vehicle and verify the encoders at every
boot. Immediately before G1, it therefore marks the encoder preflight as
certified. That bypass exists **only** in the F2 build and only because F1 has
already established these hardware facts:

- `F` drives both wheels physically forward.
- Both encoder counts increase while moving forward (`RIGHT_ENCODER_REVERSED`
  is enabled for Nano #1).
- Both A and B phases produce balanced edge totals with no excessive invalid
  transitions.
- MPU6050 is detected at `0x68` and gyro-Z can calibrate.

F2 still stops on an MPU read/I2C failure, a per-wheel encoder stall, a path
stall or a path timeout. It must only be flashed to a vehicle whose wiring has
already passed the F1 checks. If any encoder or motor wiring changes, return to
F1 first.

There is no software `S` stop command in F2. Put a reachable physical switch
in the motor-power line; that is the emergency stop. Keep clear space ahead of
the vehicle and do not touch it during the first 4.5 seconds after power-on.

## Build and upload

F1 remains the default VS Code/PlatformIO environment. To make the autonomous
image, explicitly select **`nanoatmega328_f2_autorun_g1`** and upload it, or
run:

```sh
platformio run -e nanoatmega328_f2_autorun_g1 --target upload
```

For the first F2 upload, leave external motor power switched off. With a serial
monitor open you should see `FIRMWARE_PROFILE=F2 (one-shot autonomous G1)`.
It will attempt G1 after calibration; without motor power it should stop with
an encoder-stall fault, which confirms the protection is active. Power-cycle
before the real run.

## First autonomous run procedure

1. Re-flash and validate F1 first if any connector moved: run `C`, `R`, then a
   raised-wheel `F` test and inspect `D`. Both forward counts must increase and
   preflight must pass.
2. Upload the F2 environment. Disconnect USB after upload if battery operation
   is the goal; never feed conflicting 5 V sources into the Nano.
3. Place the vehicle straight on the test floor, with at least 1 m clear ahead.
   Make the physical motor-power switch reachable.
4. Turn on Nano and motor power. Leave the vehicle fully still for the initial
   three-second wait and the following roughly 1.5-second gyro calibration.
5. F2 drives one 500 mm G1 segment, then remains stopped. Turn motor power off
   before handling the vehicle.
6. Measure actual distance and sideways/heading error. Repeat from the same
   starting conditions. Adjust F2 calibration only from repeatable results;
   keep the known-good F1 baseline unchanged.

## Current Nano #1 baseline carried into F2

- Left encoder: 1203.5 decoded ticks/revolution, measured over 10 marked wheel
  turns (`12035` counts; A/B edges 6017/6022).
- Right encoder: 1200 ticks/revolution is still a provisional value until an
  equivalent marked 10-turn measurement is recorded.
- Wheel diameter: 65 mm; track width: 130 mm.
- IMU yaw uses gyro Z; the most recent static calibration succeeded. Roll and
  pitch are diagnostic-only and are not used to steer G1.

The expected first F2 issue is geometric error caused by the provisional right
encoder scale or wheel-speed mismatch. That is a calibration observation, not a
reason to alter the F1 interactive/safety baseline.
