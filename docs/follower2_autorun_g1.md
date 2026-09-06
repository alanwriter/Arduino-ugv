# Follower 2 — one-shot autonomous G1 demo

`follower2-demo` is an autonomous image derived from the `follower2` full
calibration profile. It is separate from Follower 1 demo and carries F2's own
encoder values, directions and provisional geometry:

- left/right ticks per wheel revolution: 1216.2 / 1237.4;
- wheel diameter: 65 mm (provisional);
- wheel track: 128 mm (provisional); and
- right encoder inversion enabled.

## Power-on sequence

```text
power/reset
  -> motor outputs forced off
  -> 3 s stillness delay
  -> 300-sample gyro-Z calibration (about 1.5 s)
  -> reset encoder counts, pose and PID history
  -> G1: 500 mm forward with wheel PID and gyro-Z heading correction
  -> stop permanently until the next power cycle
```

The demo has no serial motion commands. Serial at 115200 baud is optional for
reading its status and faults. It assumes the F2 full profile has already
passed the manual checks of motor direction, encoder A/B phase health and
logical encoder sign. Runtime I2C, MPU, wheel stall and path timeout safety
checks remain active.

## Build

Select **`nanoatmega328_follower2_demo_g1`** in PlatformIO, or run:

```sh
platformio run -e nanoatmega328_follower2_demo_g1 --target upload
```

On boot, it should print `FIRMWARE_PROFILE=F2-DEMO`. Keep the vehicle still for
about 4.5 seconds after power-on. Use a reachable physical motor-power switch
as the emergency stop; the demo is intentionally autonomous and runs once per
power cycle.

This demo is prepared and stored before the F2 G1 ground test. Do not use its
floor-run result to approve G2/G3: tune F2's full controller using repeated G1
measurements first, then regenerate the demo if its geometry changes.
