# Three-vehicle calibration and demo cycle

Each physical vehicle has its own branch and its own measured parameters. A
vehicle's autonomous demo is created only from that vehicle's approved full
calibration profile.

| Vehicle | Full calibration branch | Autonomous demo branch | Current state |
| --- | --- | --- | --- |
| Follower 1 | `follower1` | `follower1-demo` | Full profile calibrated; demo image prepared. |
| Follower 2 | `follower2` | `follower2-demo` | MPU/encoder/pin/geometry measurement in progress. |
| Leader 1 | `leader1` | `leader1-demo` | Starts after Follower 2 follows the same process. |

## Repeatable cycle for each vehicle

```text
pin map confirmed
  -> MPU6050 address, gyro-Z bias and yaw sign measured
  -> encoder A/B phase integrity measured
  -> motor and logical encoder directions corrected
  -> 10-turn ticks/rev plus wheel geometry measured
  -> full-controller G1 ground test repeated
  -> freeze that vehicle's calibration branch
  -> derive its one-shot autonomous G1 demo branch
  -> run demo, record distance / lateral / heading error
  -> feed repeatable errors back into that vehicle only
```

Do not transfer an encoder direction, ticks/revolution, tyre diameter, track
width, gyro sign or pin assignment from one physical vehicle to another just
because the parts look similar. Only the software structure and test method are
shared.
