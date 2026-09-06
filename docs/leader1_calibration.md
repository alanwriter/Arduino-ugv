# Leader 1 — calibration record and test sequence

Leader 1 uses the same full controller and `nanoatmega328_demo_g1` build target
as Follower 1 and Follower 2. Its first MPU/encoder calibration is complete.
The profile permits cautious straight-line `G1`; paths containing turns (`G2`,
`G3`) remain locked until repeated floor tests verify the provisional geometry.

The firmware now identifies itself as:

```text
FIRMWARE_PROFILE=L1 (leader1 G1 calibration ready)
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
diameter and 130 mm wheel-centre track. It enables L1's first straight G1 test
only; actual floor travel determines the later diameter/track refinements.

## First L1 ground G1 procedure

1. Upload the current `leader1` branch and verify its boot line says
   `FIRMWARE_PROFILE=L1`.
2. On level ground with at least 1 m clear ahead, keep the physical motor-power
   switch reachable. Send `C` while completely still, then `R`, then `G1`.
3. Measure travelled distance, side offset and final heading. Repeat three
   times and save each `P` line plus the physical measurements.
4. Tune wheel diameter from repeatable travel error first. Do not unlock G2/G3
   until straight travel is repeatable; then tune track width from turn tests.

The identical `nanoatmega328_demo_g1` environment will then become L1's
autonomous demo without any core-code change.
