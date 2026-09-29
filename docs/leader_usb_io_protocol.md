# Leader 1：Raspberry Pi USB I/O 通訊規格

本文件定義 Raspberry Pi 與 Leader 1 Nano 的**機器可讀**控制協定。它使用 Nano
USB 埠的 CDC serial，並以 `115200 baud, 8-N-1` 傳輸 ASCII；每個輸入和輸出訊息
各佔一行、以 `\n` 結尾。這不是 GPIO、I2C 或 Nano 的 D0/D1 UART 連線。

```text
Raspberry Pi USB host ── USB cable ── Nano USB port
      高階決策／相機／紀錄              encoder、PID、MPU6050、馬達安全停止
```

Pi 應使用穩定的 `/dev/serial/by-id/...` 路徑，不要假設 `/dev/ttyACM0` 或
`/dev/ttyUSB0` 永遠不變。USB 開啟通常會使 Nano reset；因此打開 port 後須等待
約 2 秒，先送 `HELLO` 與 `STOP`，才可查詢或運動。

## Leader 1 韌體設定

此版本建立於 `leader1` 校正資料：

| 參數 | 值 |
| --- | ---: |
| 左／右 encoder ticks per revolution | 1216.4 / 1222.4 |
| 輪徑／輪距 | 65.0 mm / 130.0 mm（暫定） |
| 最大 PWM | ±165 |
| 手動馬達命令最長有效期 | 1200 ms |
| 已開放路徑 | G1（500 mm 直線）、G2（700 mm 方形） |
| 未開放路徑 | G3 |

## 通訊原則

Pi 每次送出的命令格式固定為：

```text
IO,<sequence>,<OPERATION>[,<argument>...]\n
```

- `sequence`：Pi 產生的十進位整數 `0..65535`；每個未完成請求不可重複。
- `OPERATION`：不分大小寫；Pi 程式固定使用大寫。
- Nano 的直接回覆同樣帶回這個 sequence；Pi 必須據此配對，不可把非同步遙測誤認為回覆。
- Nano 原有的開機、診斷和 fault 文字行仍可能出現。Pi 應記錄它們，但只解析 `IO,` 開頭的行。
- 行內不允許空白、JSON 或二進位資料；最大命令長度 39 個 ASCII 字元。

## Pi → Nano：完整輸入參數

| 指令 | 必填參數與範圍 | Nano 動作 | 正常回覆 |
| --- | --- | --- | --- |
| `HELLO` | `version=1` | 協定握手，不會動馬達 | `IO,HELLO,...` |
| `PING` | 無 | 連線存活測試 | `IO,PONG,...` |
| `STATUS` | 無 | 讀取完整運動狀態 | `IO,STATUS,...` |
| `CONFIG` | 無 | 讀取 Leader 校正與限制 | `IO,CONFIG,...` |
| `IMU` | 無 | 讀取 MPU6050 狀態與姿態 | `IO,IMU,...` |
| `ENCODER` | 無 | 讀取兩組 AB encoder 與 edge 計數 | `IO,ENCODER,...` |
| `STOP` | 無 | 立刻 coast stop，清除 fault | `IO,ACK,<sequence>,STOP` |
| `RESET` | 無 | 停止、清 encoder、重設相對 pose／heading | `IO,ACK,...,RESET` |
| `CALIBRATE` | 無 | 停止後量測 300 筆 gyro Z bias；車體必須靜止約 1.5 秒 | `IO,ACK,...,CALIBRATE` |
| `MOTOR` | `left_pwm,right_pwm,hold_ms`；PWM 均為 `-165..165`，有效期 `50..1200` ms | 直接驅動左右輪；到期必停 | `IO,ACK,...,MOTOR` |
| `PATH` | `number=1..3` | 啟動已核准的預設路徑 | `IO,ACK,...,PATH` |
| `TELEMETRY` | `interval_ms=0` 或 `100..2000` | `0` 關閉；其他值開始週期性狀態輸出 | `IO,ACK,...,TELEMETRY` |

範例（`42` 是 Pi 的 sequence）：

```text
IO,42,MOTOR,80,80,500
IO,43,STATUS
IO,44,TELEMETRY,500
IO,45,STOP
```

`MOTOR` 的正值是車體邏輯前進，負值為後退；左右正負不同可原地轉向。它是短暫、
開迴路的人工控制，Nano 無論 Pi 是否仍連著，都會在 `hold_ms` 到期後停止。不可用
連續 `MOTOR` 命令取代 Nano 的 PID 或故障保護。

`PATH` 仍由 Nano 本地執行 PID、encoder preflight、IMU 校正、stall timeout 與
path timeout。它只會在 `IMU present=1`、`calibrated=1`、`encoder_preflight=1` 且
該路徑已核准時開始；否則回傳 `PATH_BLOCKED`。Pi 一收到 fault、鏡頭偵測異常、程式
結束或 USB 有問題時，應嘗試送 `STOP`。

## Nano → Pi：完整輸出參數

所有 `0/1` 欄位以 `1=yes`、`0=no` 表示。數字沒有單位字尾，單位由下表固定。

### 協定與命令回覆

```text
IO,HELLO,<sequence>,1,115200,<profile_boot_message>
IO,PONG,<sequence>,<nano_millis>
IO,ACK,<sequence>,<OPERATION>
IO,ERROR,<sequence>,<code>
```

`ERROR` code 為 `FORMAT`、`VERSION`、`OPERATION`、`MOTOR_RANGE`、`PATH_RANGE`、
`PATH_BLOCKED`、`TELEMETRY_RANGE` 或 `CALIBRATE_FAILED`。收到 `ERROR` 時 Pi 不可
假設動作已執行；應查詢 `STATUS`，必要時送 `STOP`。

### `STATUS` 與非同步 `TELEMETRY`

兩者欄位完全相同；`TELEMETRY` 的 sequence 固定為 `0`，代表它不是某一個請求的回覆。

```text
IO,STATUS,<sequence>,<mode>,<x_mm>,<y_mm>,<heading_deg>,<left_count>,<right_count>,<left_tps>,<right_tps>,<left_target_tps>,<right_target_tps>,<left_pwm>,<right_pwm>,<fault_code>,<encoder_preflight>,<imu_present>,<imu_calibrated>,<path_number>,<path_step>
IO,TELEMETRY,0,...同上欄位...
```

| 欄位 | 說明 |
| --- | --- |
| `mode` | `idle`、`manual`、`path` 或 `fault` |
| `x_mm`, `y_mm`, `heading_deg` | reset 後的相對里程計；非 GPS／SLAM／絕對座標 |
| `left_count`, `right_count` | 4× AB quadrature 累積 tick |
| `left_tps`, `right_tps` | 實測 ticks/s |
| `left_target_tps`, `right_target_tps` | Nano PID 目前目標 ticks/s |
| `left_pwm`, `right_pwm` | 實際套用的有號 PWM |
| `fault_code` | 見下方 fault 對照表 |
| `encoder_preflight` | 已觀測到兩輪 A/B 相且通過健康檢查 |
| `imu_present`, `imu_calibrated` | MPU6050 偵測／gyro Z bias 校正狀態 |
| `path_number`, `path_step` | 非 path 模式均為 `0`；否則為目前 G 編號及從 1 起算的步驟 |

### `CONFIG`

```text
IO,CONFIG,<sequence>,<left_ticks_per_rev>,<right_ticks_per_rev>,<wheel_diameter_mm>,<wheel_track_mm>,<turn_pwm_bias>,<max_pwm>,<max_manual_hold_ms>,<max_approved_path>
```

### `IMU`

```text
IO,IMU,<sequence>,<present>,<calibrated>,<i2c_address_decimal>,<roll_deg>,<pitch_deg>,<gyro_x_dps>,<gyro_y_dps>,<gyro_z_dps>,<gyro_z_bias_raw>,<read_failures>,<consecutive_read_failures>
```

### `ENCODER`

```text
IO,ENCODER,<sequence>,<preflight>,<LA_level>,<LB_level>,<RA_level>,<RB_level>,<left_count>,<left_valid_edges>,<left_a_edges>,<left_b_edges>,<left_invalid>,<right_count>,<right_valid_edges>,<right_a_edges>,<right_b_edges>,<right_invalid>
```

`A_edges`、`B_edges` 應都持續增加且數量接近；`invalid` 應接近 0。若任何一相沒有
edge，請停止並檢查接線，不能透過軟體略過 preflight。

| `fault_code` | 意義 |
| ---: | --- |
| 0 | none |
| 1 / 2 | drive / turn timeout |
| 3 / 4 | drive / turn stall |
| 5 / 6 | left / right encoder 或輪子 stall |
| 7 / 8 | MPU6050 read failure / I2C timeout |

## Pi 端工具與操作流程

安裝相依套件後，使用本專案的 client：

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r python_examples/requirements.txt
ls -l /dev/serial/by-id/
PORT=/dev/serial/by-id/<Nano 的完整名稱>

python3 python_examples/leader_usb_control.py --port "$PORT" status
python3 python_examples/leader_usb_control.py --port "$PORT" imu
python3 python_examples/leader_usb_control.py --port "$PORT" encoders
```

第一次在靜止、架高且可立即斷電的條件下測試：

```bash
# 車體絕對靜止，完成 gyro Z 校正
python3 python_examples/leader_usb_control.py --port "$PORT" calibrate

# 短暫手動測試，500 ms 後 Nano 自停；不要一開始跑路徑
python3 python_examples/leader_usb_control.py --port "$PORT" drive 80 80 --hold-ms 500
python3 python_examples/leader_usb_control.py --port "$PORT" stop
```

確認 IMU 已校正、兩輪 encoder preflight 已通過、四周有安全淨空且有人監看後，才可
測試 G1 或受監看的 G2：

```bash
python3 python_examples/leader_usb_control.py --port "$PORT" path 1
python3 python_examples/leader_usb_control.py --port "$PORT" path 2 --timeout 90
```

此 client 會在結束、例外或 Ctrl-C 後嘗試送 `STOP`。一次只能有一個 Pi 程式開啟此
serial port；PlatformIO Monitor 與其他程式必須關閉。Pi 端程式不應在開機時自動運動，
也不應把密碼或 serial 裝置路徑的私人資訊提交到 Git。
