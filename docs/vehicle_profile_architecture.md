# Shared F1 / F2 / L1 controller and demo architecture

Follower 1, Follower 2 and Leader 1 use the same controller source and the
same autonomous demo state machine. Branches differ only in
`src/vehicle_profile.h` and their measurement records.

```text
src/main.cpp              shared interactive controller, PID, safety and paths
src/demo_autorun.cpp      shared one-shot power-on demo entry point
platformio.ini            shared full and demo build environments
src/vehicle_profile.h     the only vehicle-specific pin and parameter file
```

Each vehicle has these build environments in its own branch:

| Environment | Role |
| --- | --- |
| `nanoatmega328` | Full interactive calibration/controller image. |
| `nanoatmega328_demo_g1` | Optional: wait 3 s, calibrate gyro Z, reset, run G1 once, then stop. |
| `nanoatmega328_demo_square` | Standard demo: wait 3 s, calibrate gyro Z, reset, run the shared 700 mm G2 square once, then stop. |

Only these profile items may vary between vehicles:

- motor and encoder direction flags;
- gyro-Z sign;
- left/right decoded ticks per wheel revolution;
- wheel diameter and track width; and
- the highest enabled preset path (`0` none, `1` G1 only, `2` G1–G2, `3` G1–G3).

The current shared quadrature ISR is standardized for encoder pins D2/D8/D7/D12.
Changing that map requires a deliberate low-level ISR refactor, not merely a
profile edit. The demo control flow, 700 mm square definition, PID constants,
MPU reading, encoder decoder, watchdog, stall protection and timeout protection
remain identical across all vehicles. A profile approval gate prevents a
vehicle from moving for any path it has not yet passed.
