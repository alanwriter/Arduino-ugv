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
`M`, `P`). The measured profile enables cautious `G1` and one supervised 700
mm-square `G2` calibration run; that is not final square approval. `G3`
remains blocked until the square's closure error is measured.

## Before upload: confirm the F2 pin map

The tested F2 profile uses the standardized shared map below. Confirm every
wire before connecting motor power:

| Signal | Current temporary reference | Follower 2 actual connection |
| --- | --- | --- |
| Left encoder A/B | D2 / D8 | verified during preflight |
| Right encoder A/B | D7 / D12 | verified during preflight |
| L298N left IN1/IN2/ENA | D3 / D4 / D9 | verified forward raised-wheel test |
| L298N right IN1/IN2/ENB | D5 / D6 / D10 | verified forward raised-wheel test |
| MPU6050 SDA/SCL | A4 / A5 | verified MPU6050 test |

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
6. **Geometry and turn approval** — measure loaded outside tyre diameter and
   centre-to-centre track. F2 uses the preliminary shared values below and
   permits a monitored first G2 square. Measure its closure error before any
   further path approval; G3 remains locked.

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
| Path-turn PWM bias | +10 PWM per driven wheel during `PATH_TURN_DEGREES` | F2 first square needed more turn breakaway torque; no effect on straight/manual commands |
| Left ticks/rev | 1216.2 | 12,162 / 10 marked forward turns |
| Right ticks/rev | 1237.4 | 12,374 / 10 marked forward turns |
| Left A/B/invalid | 6081 / 6083 / 0 | 12,164 valid edges |
| Right A/B/invalid | 6199 / 6187 / 0 | 12,386 valid edges |
| Wheel diameter (mm) | 65, provisional | approximate outside measurement |
| Track width (mm) | 130, provisional | shared three-vehicle wheel-centre assumption |

The standard autonomous target is `nanoatmega328_demo_square` in this branch.
It powers on, waits for stillness, calibrates gyro-Z, resets pose and runs the
shared 700 mm G2 square once. F2 permits this as one supervised first square,
not a final demonstration approval; keep a physical motor-power stop within
reach and leave G3 locked until the closure error is measured.

## 2026-09-06 first sensor/encoder result

The MPU and both encoders pass the first acceptance test. The static attitude
stream held `yaw_rel=0.0°` after calibration; the small static roll/pitch and
about 1.08–1.09 g accelerometer magnitude are not used for yaw steering. The
second raised-wheel `F` test produced left `+3903` and right raw `-3708`
counts, confirming the right decoder inversion above. The first `F` command
showed no encoder motion, so its result was excluded; the subsequent moving
test is the accepted direction result.

The first shared geometry estimate is 65 mm wheel diameter and 130 mm track
width. The user confirmed these physical dimensions are common to F1, F2 and
L1; retain the values as provisional until repeated floor tests refine them.
This enables G1 and one supervised first 700 mm-square G2 calibration run; G3
stays locked until the square's closure error is measured.

## F2 ground-test procedure

1. Upload the current `follower2` branch and confirm the boot line says
   `FIRMWARE_PROFILE=F2`.
2. For the first straight check, use level ground with at least 1 m clear ahead.
   Keep the physical motor-power switch reachable; send `C`, `R`, then `G1`.
   Do not use F as a geometry test: F is open-loop equal PWM; G1 uses wheel
   controllers and gyro-Z heading correction.
3. For the first square, clear at least a 2 m × 2 m area. In the full controller
   send `C`, then `R`, then `G2`; it drives four 700 mm sides with four logical
   left 90° turns. Stop with `S` or the motor-power switch if its path is unsafe.
4. Record return-to-start distance, final heading and each visibly incorrect
   turn. Tune wheel diameter from repeatable straight error first, then tune
   track width from square closure. Do not unlock G3 yet.
