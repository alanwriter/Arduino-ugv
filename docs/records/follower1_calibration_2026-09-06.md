# follower1：Nano #1 校正與診斷紀錄（2026-09-06）

此分支從 `main` 的初始展示版建立。只套用已有量測支持的常數，尚未調整
PID、輪徑、輪距、馬達方向或右輪 ticks/rev。

## 已套用設定

| 設定 | follower1 值 | 依據 |
| --- | ---: | --- |
| `LEFT_TICKS_PER_WHEEL_REVOLUTION` | `1203.5` | 記號對準後手轉左輪 10 圈，`count=12035`。|
| `RIGHT_TICKS_PER_WHEEL_REVOLUTION` | `1200.0` | 暫不修改；需以同一套 10 圈記號流程重測。|
| `GYRO_Z_REVERSED` | `false` | 已觀察到相對 yaw 可正、反向連續變化；下一步以實車「左轉為正」再確認。|

## Encoder 診斷

### 左輪：修復後通過

```text
count=12035, turns=10.029, edges=12039,
A_edges=6017, B_edges=6022, invalid=0
```

- 物理左輪精準轉 10 圈，得到 `1203.5 ticks/rev`。
- A/B edge 比為 `0.999`，無 invalid transition，訊號健康。
- 先前 A 相杜邦線誤觸 D2 旁的 GND／鐵皮，造成 A 相缺失；換線與隔離後恢復。

### 右輪：AB 訊號通過，常數待重測

```text
count=-10382, edges=10394,
A_edges=5197, B_edges=5197, invalid=0
```

- A/B 完全平衡、invalid 為 0，硬體訊號可用。
- 當次沒有以與左輪相同的精準 10 圈記號流程確認，因此不以這筆資料覆寫 1200 ticks/rev。

## MPU6050 診斷

```text
Gyro calibrated. Z raw bias=39.59
```

- 靜止時校正後的 `gyro_z_dps` 顯示為 0，yaw 在短時間內維持穩定。
- 已量到一次約 +90 度的水平旋轉，穩定讀值 `yaw_rel=90.1`。
- 完整往返後相對 yaw 穩定在約 `+0.8` 度；足以進入短距離閉迴路展示。
- 靜止 accel 約 1.11–1.12 g，X/Y gyro 尚未校正；這不影響目前使用 Z gyro 的航向控制，不能視為精密 roll/pitch 校正。

## 分支增加的安全門檻

encoder preflight 現在除了檢查兩相都至少有 20 edge、invalid transition 不超標，
也要求每輪較少的一相至少為另一相的 75%。這避免「只剩 B 相、count 接近 0」的
故障被短暫雜訊誤判通過。

## 下一步

1. 右輪使用輪上記號精準轉 10 圈，重複 3 次（正、反方向都可），取平均設定 `RIGHT_TICKS_PER_WHEEL_REVOLUTION`。
2. 輪子懸空，確認 `F` 的物理前進方向和左右 encoder count 的正負號。
3. 地面先跑 `G1`，量實際行進距離與偏航，再調整有效輪徑／左右比例。
4. 最後才跑 `G3`、`G2`。
