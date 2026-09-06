# Follower 2 — hardware calibration record and test sequence

`follower2` is the full, interactive calibration profile for the second
Follower vehicle. It is deliberately separate from `follower1`: do not copy
Follower 1's wheel ticks, motor/encoder direction, geometry or pin map into
Follower 2 without measurement.

The firmware boots as:

```text
FIRMWARE_PROFILE=F2 (follower2 calibration pending)
```

It supports all safe diagnostic commands (`I`, `C`, `T`, `R`, `D`, `F`, `B`,
`M`, `P`) but blocks `G1`, `G2` and `G3` until its own calibration constants
are explicitly approved in source code.

## Before upload: confirm the F2 pin map

The source currently contains the old Follower 1 reference map only. Confirm
or replace it before connecting motor power:

| Signal | Current temporary reference | Follower 2 actual connection |
| --- | --- | --- |
| Left encoder A/B | D2 / D8 | record before upload |
| Right encoder A/B | D7 / D12 | record before upload |
| L298N left IN1/IN2/ENA | D3 / D4 / D9 | record before upload |
| L298N right IN1/IN2/ENB | D5 / D6 / D10 | record before upload |
| MPU6050 SDA/SCL | A4 / A5 | record before upload |

All logic grounds must be common. Keep the vehicle raised whenever testing a
new motor direction or unknown encoder wire.

## Calibration sequence

1. **Sensor discovery** — upload with external motor power off, open Serial at
   115200, and send `I`. Record MPU presence and address (`0x68` or `0x69`).
2. **Gyro-Z calibration and yaw check** — leave the vehicle completely still,
   send `C`, then `T`. Rotate the vehicle horizontally left/right. `yaw_rel`
   should follow the turn direction and return near zero after an equal reverse
   turn. Gyro X/Y are diagnostic only; yaw control uses Z.
3. **Encoder phase check by hand** — send `R`; turn each wheel by hand; then
   send `D`. Both sides need non-zero, balanced `A_edges` and `B_edges`, low
   `invalid`, and a passed preflight. A phase that stays near zero is a wire or
   connector fault, not a tuning issue.
4. **Motor and logical encoder direction** — with wheels raised, send `R`,
   then `F`, then `P`/`D`. Physical motion must be forward and both counts must
   increase. Update `*_MOTOR_REVERSED` for a backward wheel and
   `*_ENCODER_REVERSED` for a negative forward count. Repeat `R`/`F` until
   both are correct.
5. **Ticks per revolution** — mark each wheel. For each side separately, send
   `R`, rotate exactly ten marked wheel turns in the forward direction, then
   send `D`. Set ticks/revolution to `absolute(count) / 10`. Record the A/B
   totals too; they should be similar.
6. **Geometry and final approval** — measure loaded outside tyre diameter and
   centre-to-centre track. Put F2's own values in `src/main.cpp`, set
   `FOLLOWER2_PATH_CALIBRATION_APPROVED = true`, compile, then begin with G1
   floor tests. Do not try G2 until straight G1 is repeatable.

## Measurement record

| Item | Follower 2 value | Evidence / notes |
| --- | --- | --- |
| MPU address |  | `I` output |
| gyro-Z raw bias | -102.78 raw | `C`, vehicle still |
| yaw sign | left +90.5°, right -90.0°, return +0.4° | Z positive is logical left |
| Left motor direction | forward with `LEFT_MOTOR_REVERSED = false` | `F` raised-wheel test |
| Right motor direction | forward with `RIGHT_MOTOR_REVERSED = false` | `F` raised-wheel test |
| Left encoder direction | forward raw count positive | `LEFT_ENCODER_REVERSED = false` |
| Right encoder direction | forward raw count negative | `RIGHT_ENCODER_REVERSED = true` |
| Left ticks/rev | 1216.2 | 12,162 / 10 marked forward turns |
| Right ticks/rev | 1237.4 | 12,374 / 10 marked forward turns |
| Left A/B/invalid | 6081 / 6083 / 0 | 12,164 valid edges |
| Right A/B/invalid | 6199 / 6187 / 0 | 12,386 valid edges |
| Wheel diameter (mm) |  | loaded measurement |
| Track width (mm) |  | wheel-centre measurement |

After this record is complete, create `follower2-demo` from the approved F2
branch. Its autonomous sequence will be the same pattern as Follower 1 demo:
power on, stillness delay, gyro-Z calibration, reset pose, fixed G1, then stop.

## 2026-09-06 first sensor/encoder result

The MPU and both encoders pass the first acceptance test. The static attitude
stream held `yaw_rel=0.0°` after calibration; the small static roll/pitch and
about 1.08–1.09 g accelerometer magnitude are not used for yaw steering. The
second raised-wheel `F` test produced left `+3903` and right raw `-3708`
counts, confirming the right decoder inversion above. The first `F` command
showed no encoder motion, so its result was excluded; the subsequent moving
test is the accepted direction result.

Paths remain locked because Follower 2's loaded wheel diameter and track width
still need measurement, followed by a cautious ground G1 test.
