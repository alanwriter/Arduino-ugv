# Three-vehicle calibration and demo cycle

Each physical vehicle has its own branch and measured profile. The autonomous
demo is built from the same approved profile in that vehicle's branch, so it
cannot accidentally use another car's encoder or direction settings.

| Vehicle | Full calibration branch | Autonomous demo build target | Current state |
| --- | --- | --- | --- |
| Follower 1 | `follower1` | `nanoatmega328_demo_square` in `follower1` | Full profile calibrated; G1–G3 approved. |
| Follower 2 | `follower2` | `nanoatmega328_demo_square` in `follower2` | MPU/encoder calibration recorded; G1 approved, square remains gated until G2 is tested. |
| Leader 1 | `leader1` | `nanoatmega328_demo_square` in `leader1` | MPU/encoder calibration recorded; first 700 mm square test authorized. |

## Repeatable cycle for each vehicle

```text
pin map confirmed
  -> MPU6050 address, gyro-Z bias and yaw sign measured
  -> encoder A/B phase integrity measured
  -> motor and logical encoder directions corrected
  -> 10-turn ticks/rev plus wheel geometry measured
  -> full-controller G1 ground test repeated
  -> full-controller G2 700 mm square tested and measured
  -> freeze that vehicle's calibration profile
  -> build its one-shot autonomous square demo environment
  -> run demo, record distance / lateral / heading error
  -> feed repeatable errors back into that vehicle only
```

Do not transfer an encoder direction, ticks/revolution, gyro sign or pin
assignment from one physical vehicle to another just because the parts look
similar. Only the software structure and test method are shared. Wheel diameter
and track have a user-confirmed common initial value of 65 mm / 130 mm for all
three vehicles; retain it as provisional until each vehicle's floor tests prove
or refine it.
