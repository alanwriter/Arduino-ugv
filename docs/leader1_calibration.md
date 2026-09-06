# Leader 1 — calibration record and test sequence

Leader 1 uses the same full controller and `nanoatmega328_demo_g1` build target
as Follower 1 and Follower 2. Its `vehicle_profile.h` is intentionally
uncalibrated: `MAX_APPROVED_PRESET_PATH = 0` blocks all path motion, including
the demo, until L1 measurements are entered.

## Required sequence

1. Confirm the standardized encoder pin map (left A/B D2/D8, right A/B D7/D12)
   and L298N/MPU wiring before connecting motor power.
2. Upload the full `nanoatmega328` environment; check the L1 boot banner and
   send `I`, `C`, `T` to confirm MPU6050 address, gyro-Z bias and yaw sign.
3. With wheels raised, use `R`, hand turns and `D` to verify both encoder A/B
   phases and zero/low invalid transitions.
4. Use `R`, `F`, `P`/`D` to establish motor direction and logical count sign.
5. Mark each wheel and record exact 10-turn counts. Measure loaded wheel
   diameter and centre-to-centre wheel track.
6. Update only `src/vehicle_profile.h`; set `MAX_APPROVED_PRESET_PATH = 1`
   for cautious G1 ground tests. Enable `3` only after repeatable straight and
   turn calibration.

## Record

| Item | Leader 1 value | Evidence / notes |
| --- | --- | --- |
| MPU address |  | `I` |
| gyro-Z raw bias |  | `C` while still |
| yaw sign |  | left / right 90° test |
| Left/right motor direction |  | raised-wheel `F` |
| Left/right encoder sign |  | forward count sign |
| Left/right ticks per revolution |  | 10 marked turns each |
| Left/right A/B/invalid |  | `D` |
| Wheel diameter / track |  | loaded measurements |

After G1 is approved, the same `nanoatmega328_demo_g1` environment becomes
L1's autonomous demo without any core-code change.
