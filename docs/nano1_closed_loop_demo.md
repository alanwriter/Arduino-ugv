# Nano #1：雙 encoder、L298N、MPU6050 閉迴路展示

目前的 src/main.cpp 是週一展示用的單片 Nano #1 韌體：

~~~text
雙 AB encoder ──> 輪速 PID ──> L298N ──> 左／右 TT 馬達
       │                              ^
       └──> 里程計 ──> 預設路徑 ───────┘
MPU6050 gyro Z ──> 航向修正 ───────────┘
~~~

它使用相對里程計（dead reckoning），不是 GPS、SLAM 或絕對定位。從同一個起點出發、在平坦且不太打滑的地面上，足以演示直線、L 型與方形軌跡。

> Nano 完成 setup 後，L298N 的 IN1–IN4 都是 LOW，ENA／ENB PWM 都是 0。馬達只有在傳送 F、B、M... 或 G... 指令後才會動。Nano 在 reset／brownout 期間的腳位是浮動的，因此 ENA／ENB 必須另加硬體保護，不能只依賴程式。

## 接線

### Nano #1 到 encoder 與 L298N

| Nano #1 腳位 | 接線 | 備註 |
| --- | --- | --- |
| D2 | 左 encoder A | INT0 |
| D8 | 左 encoder B | PCINT |
| D7 | 右 encoder A | PCINT |
| D12 | 右 encoder B | PCINT |
| D3、D4 | L298N IN1、IN2 | 左馬達方向；已固定焊接 |
| D5、D6 | L298N IN3、IN4 | 右馬達方向；已固定焊接 |
| D9 | L298N ENA | 左 PWM；**移除 ENA jumper**，ENA 加 10 kΩ 至 GND |
| D10 | L298N ENB | 右 PWM；**移除 ENB jumper**，ENB 加 10 kΩ 至 GND |
| 5V | 兩顆 encoder VCC | 只在已確認 VCC 線時接 |
| GND | 兩顆 encoder GND、L298N GND | 必須共地 |

TT encoder 黃＝VCC、黑＝GND、紅＝A、白＝B，但每顆都必須以小板絲印或已完成的 encoder 測試確認。已知 B 相沒有訊號的 encoder 不可用於本閉迴路版本。

### MPU6050 到 Nano #1

| MPU6050 | Nano #1 |
| --- | --- |
| SDA | A4 |
| SCL | A5 |
| GND | GND |
| VCC | 依 breakout 標示接 5V 或 3.3V |

常見 GY-521 breakout 通常可接 Nano 5V；若板子只標示 3.3V，請接 3.3V，不要猜測。MPU6050 在展示版直接接 Nano #1，暫時不要接 Raspberry Pi 或第二片 Nano 的 I2C。

### 電源（必要）

~~~text
外部馬達電源 +  -> L298N Vs / +12V
外部馬達電源 -  -> L298N GND
Nano #1 GND      -> L298N GND
encoder GND      -> Nano #1 GND
MPU6050 GND      -> Nano #1 GND
~~~

- 馬達電源不可由 Nano 5V 提供。
- Nano 可先由 USB 供電；所有 GND 仍必須共地。
- ENA、ENB 各加一顆 **10 kΩ pull-down 到 GND**。這能確保 Nano 上傳、reset 或 brownout 時 L298N 不會因 enable 腳浮動而意外驅動馬達。
- 接線、拔線、調整 L298N jumper 前先關閉馬達電源。
- 第一次測試請讓輪子懸空，手靠近序列埠並隨時可傳送 S。

## 先量的兩個尺寸

程式目前有可編譯的暫定值，位於 src/main.cpp 頂端：

~~~cpp
constexpr float WHEEL_DIAMETER_MM = 65.0f;
constexpr float WHEEL_TRACK_MM = 130.0f;
~~~

它們不一定是你的實車尺寸，請在跑路徑前更新：

1. WHEEL_DIAMETER_MM：量輪胎有效外徑；若輪胎有壓縮，最準是讓輪子在地面滾一整圈量實際行進距離，再除以 π。
2. WHEEL_TRACK_MM：量左右輪胎中心線的距離。
3. LEFT_TICKS_PER_WHEEL_REVOLUTION 與 RIGHT_TICKS_PER_WHEEL_REVOLUTION：目前先用 1200.0，但兩輪應各自以記號轉 10 圈校正。

所有 tick 都是完整 AB 四倍頻；每輪約 1200 tick。不要把舊版 A 相二倍頻的 600 tick／圈數值混進來。

## 上傳後的安全驗證順序

請依順序進行，**不要直接從 G2 方形路徑開始**。

1. 上傳，開啟 115200 baud Serial Monitor。開機後應顯示 Motors are stopped。
2. 輪子仍不接馬達電源或先懸空，傳送 D。
3. 手轉左右輪，各自確認 A、B edge 都增加、invalid 接近 0。若某輪任一相維持 0，先修接線或換 encoder。
4. 接上馬達電源、輪子懸空，傳送 F。兩輪應同時向車體前方轉，且 1.2 秒後自動停止。當兩輪 A、B 相都累積足夠 edge 時，程式會印出 Encoder preflight passed。
   - 若某顆物理方向錯誤，修改程式的 LEFT_MOTOR_REVERSED 或 RIGHT_MOTOR_REVERSED 為 true 後重新上傳。
5. 傳送 R，再傳送 F，然後 P。邏輯前進時左右 encoder count 都應增加。
   - 若其中一輪遞減，修改對應的 LEFT_ENCODER_REVERSED 或 RIGHT_ENCODER_REVERSED 為 true。
6. 將車靜止放在地面，傳送 C。約 1.5 秒後會印出 gyro bias；傳送 I，應看到 present=yes, calibrated=yes。
7. 校正後，**手持並實際向左旋轉整個車體**，在旋轉時連續傳送 I，確認 gyro_z_dps 為正。不要只讓懸空輪子轉動：那不能驗證 MPU6050 的 gyro 方向。若實際左轉時 gyro_z_dps 為負，將 GYRO_Z_REVERSED 改為 true、重新上傳並再校正。
8. 在空曠平地上從 R 開始，先跑 G1，確認 500 mm 直走和停止都安全，再試 G3，最後才試 G2。

## 序列埠指令

| 指令 | 功能 |
| --- | --- |
| F／B | 以 PWM 80 手動前進／後退，1.2 秒自動停止 |
| M80,80 | 指定左右手動 PWM；範圍 -165 至 165 |
| M-80,80 | 邏輯左原地轉向的手動測試 |
| S | 立即 coast stop，並清除 fault |
| R | 停止、歸零兩輪 encoder 與相對座標 |
| C | 靜止時校正 MPU6050 gyro Z |
| G1 | 直走 500 mm 後停 |
| G2 | 400 mm 方形，四次左轉 90° 後停 |
| G3 | 350 mm L 型：直走、左轉 90°、再直走 |
| P | 目前相對座標、輪速、PWM、fault |
| D | 四個 encoder 腳位與完整 AB 診斷 |
| I | MPU6050 偵測、校正，以及 X／Y／Z gyro 讀值（X、Y 為未校正診斷值；目前航向控制使用 Z） |
| T | 開啟／關閉即時姿態監看（5 Hz）；顯示 roll、pitch、相對 yaw、三軸加速度與 gyro 角速度；車體開始移動時自動停止。每次開啟時相對 yaw 歸零，長時間會受 gyro 漂移影響。 |
| K | tick／輪徑／輪距設定值 |
| ? | 指令說明 |

x 是起點的前方距離，y 是起點左方距離，heading 正值代表邏輯左轉。這是估測值；輪胎打滑、輪徑誤差、輪距誤差與 gyro 漂移都會累積成誤差。

## PID 與路徑如何工作

- 每 10 ms（100 Hz）讀取 encoder 計數差，得到左右輪速。
- 每輪有獨立速度 PID，將目標輪速轉成 D9／D10 PWM。
- 路徑命令有目標加速度限制，不會一開始就直接跳到最高 PWM。
- 直線段以 encoder 距離決定何時減速／停止；MPU6050 的 gyro 與 encoder 轉向估測融合後，修正左右目標速度。
- 轉彎段以融合後的 heading 為終點，接近目標角度時減速，並允許低速修正到約 1° 範圍。
- 路徑只能在雙輪 A、B 相都通過 preflight 後啟動；任一輪未依預期方向累積 encoder tick、長時間不前進、轉不動、段落超時，或 MPU6050 讀取異常時，程式會進入 FAULT 並停止馬達。
- I2C 有 25 ms timeout，主迴圈另有 250 ms watchdog。它們是軟體保護；ENA／ENB 的 10 kΩ pull-down 才是 reset 時的硬體保護。

初始 PID 值與速度刻意保守。若輪子無法起轉，先確認電池、共地、L298N ENA／ENB jumper，再逐步調整 MOTOR_STATIC_PWM。若直走時兩輪速度差大，再調整 WHEEL_PID_KP 與 WHEEL_PID_KI；不要先提高最高 PWM。

## 日後接 Raspberry Pi

這個分工不需要推翻：

~~~text
Raspberry Pi: 路徑點、視覺、較複雜定位
       -> 目標線速度／角速度
Nano #1: encoder ISR、輪速 PID、里程計、安全停止、L298N
~~~

高速輪速 PID 永遠留在 Nano，避免 USB／Linux 排程延遲影響馬達控制。
