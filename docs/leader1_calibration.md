# Leader 1 — calibration record and test sequence

Leader 1 uses the same full controller and `nanoatmega328_demo_square` build
target as Follower 1 and Follower 2. Its first MPU/encoder calibration is complete.
The profile permits cautious straight-line `G1` and one monitored 700 mm-square
`G2` test. `G3` remains locked until the square's closure error is measured.

The firmware now identifies itself as:

```text
FIRMWARE_PROFILE=L1 (leader1 G2 square test ready)
```

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
| MPU address | not recorded | This test log did not include `I`; it is not a tuning value. |
| gyro-Z raw bias | -1.80 raw | `C` while still |
| yaw sign | left +92.1°, right -91.0°, return +0.5 to +0.6° | Z positive is logical left; no reversal needed |
| Left/right motor direction | both physically forward | raised-wheel `F`; both motor reversal flags remain `false` |
| Left/right encoder sign | left forward raw `+`; right forward raw `-` | `LEFT_ENCODER_REVERSED=false`, `RIGHT_ENCODER_REVERSED=true` |
| Left/right ticks per revolution | 1216.4 / 1222.4 | 12,164 / 12,224 counts over exact 10 marked forward turns |
| Left A/B/invalid | 6082 / 6084 / 0 | 12,166 valid edges |
| Right A/B/invalid | 6118 / 6128 / 0 | 12,246 valid edges |
| Wheel diameter / track | 65 mm / 130 mm, provisional | shared F1/F2/L1 physical-dimension assumption |

## 2026-09-06 first acceptance result

Both encoders passed phase quality: each A/B pair is balanced and neither side
reported an invalid transition. The old test firmware displayed the raw right
count as negative while both wheels were physically moving forward; this is
expected wiring polarity, so the L1 profile now inverts only the right encoder
logical sign. That prevents a forward command from being interpreted as a
turn in odometry.

The gyro-Z test is also healthy for the intended yaw controller. After the
still calibration, a left 90° rotation read about `+92.1°`; the corresponding
right rotation reached about `-91.0°`; the final return settled near `+0.5°`.
The small error is appropriate for a hand-rotated check and does not justify a
sign reversal or an additional filter.

All three vehicles use the current shared provisional geometry of 65 mm wheel
diameter and 130 mm wheel-centre track. The successful first G1 run authorizes
L1's monitored 700 mm-square G2 test. Its closure error will determine the
later wheel-diameter/track refinements; G3 remains locked in the meantime.

## L1 ground-test procedure

1. Upload the current `leader1` branch and verify its boot line says
   `FIRMWARE_PROFILE=L1`.
2. For the initial G1 check, use level ground with at least 1 m clear ahead.
   Keep the physical motor-power switch reachable; send `C` while completely
   still, then `R`, then `G1`. Measure travel, side offset and final heading.
3. For the first square, clear at least a 2 m × 2 m area. In the full controller
   send `C`, then `R`, then `G2`; it drives four 700 mm sides with four logical
   left 90° turns. Stop with `S` or the motor-power switch if its path is unsafe.
4. Record the final return-to-start distance, final heading and each visibly
   incorrect turn. Tune wheel diameter from repeated straight error first, then
   tune track width from the square's turn/closure error. Do not unlock G3 yet.

## Autonomous square demo

After one observed full-controller G2 run, build and upload
`nanoatmega328_demo_square`. On every reset it waits 3 s, calibrates gyro Z,
resets the pose and runs the same 700 mm square once, then stops. It accepts no
serial commands during the run, so keep the physical motor-power switch within
reach and do not use it until the car is placed in the cleared test area.

The same `nanoatmega328_demo_square` environment is the standard autonomous
demo for F1, F2 and L1; profile approval determines whether it can move.
