# Shared F1 / F2 / L1 controller and demo architecture

Follower 1, Follower 2 and Leader 1 use the same controller source and the
same autonomous demo state machine. Branches differ only in
`src/vehicle_profile.h` and their measurement records.

```text
src/main.cpp              shared interactive controller, PID, safety and paths
src/demo_autorun_g1.cpp   shared one-shot power-on demo entry point
platformio.ini            shared full and demo build environments
src/vehicle_profile.h     the only vehicle-specific pin and parameter file
```

Each vehicle has two build environments in its own branch:

| Environment | Role |
| --- | --- |
| `nanoatmega328` | Full interactive calibration/controller image. |
| `nanoatmega328_demo_g1` | Wait 3 s, calibrate gyro Z, reset, G1 once, then stop. |

Only these profile items may vary between vehicles:

- motor and encoder direction flags;
- gyro-Z sign;
- left/right decoded ticks per wheel revolution;
- wheel diameter and track width; and
- the highest approved preset path (`0` none, `1` G1 only, `3` G1–G3).

The current shared quadrature ISR is standardized for encoder pins D2/D8/D7/D12.
Changing that map requires a deliberate low-level ISR refactor, not merely a
profile edit. The demo control flow, PID constants, MPU reading, encoder
decoder, watchdog, stall protection and timeout protection remain identical
across all vehicles.
