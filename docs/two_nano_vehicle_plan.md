# 雙 Arduino Nano 車體架構：完整 AB encoder 與定位

本方案使用兩片 Arduino Nano，保留每顆 TT 馬達 encoder 的完整 A/B 相，
並將中斷密集的馬達控制與通訊、姿態、伺服控制分離。

> **日後架構，非週一展示接法。** 目前 src/main.cpp 使用單片 Nano #1，
> 並將 MPU6050 直接接到 Nano #1 的 A4/A5；請先依
> [nano1_closed_loop_demo.md](nano1_closed_loop_demo.md) 接線。等需要
> Raspberry Pi、雲台或無線通訊時，再依本文件把 MPU6050 移到導航 Nano。

> Nano #1 的實際 L298N IN1–IN4 已固定焊接於 D3–D6，並透過 USB Serial
> 連接 Raspberry Pi；此實際腳位表與 encoder 測試方式以
> [current_pin_map.md](current_pin_map.md) 為準。以下驅動 Nano 表已依實際
> 焊接配置更新。

```text
nRF24L01 ─┐
MPU6050 ──┼─ 導航／通訊 Nano（I2C 主機） ── I2C ── 驅動 Nano（I2C 從機） ── L298N ── 兩顆 TT 馬達
SG90 x2 ──┘                                           └─ 兩組 TT encoder AB
```

## 分工

| Nano | 專責工作 | 原因 |
| --- | --- | --- |
| 驅動 Nano | L298N、兩顆馬達的完整 AB encoder、輪速 PID | Encoder 需要快速且穩定的中斷服務，不能被無線與伺服延遲打斷。 |
| 導航／通訊 Nano | nRF24L01、MPU6050、SG90 x2、里程計與 IMU 融合 | 集中接收遙控命令、計算車體姿態／位置，再向驅動 Nano 發送目標輪速。 |

此架構適用輪式里程計、航向角融合與基本 dead reckoning。它不是 SLAM；若要地圖定位仍需額外的外部感測器與運算平台。

## 驅動 Nano 接線

| 模組 | 功能 | 驅動 Nano 腳位 | 備註 |
| --- | --- | ---: | --- |
| 左 TT encoder | A 相、B 相 | D2、D8 | D2 為外部中斷，D8 為 B 相讀取 |
| 右 TT encoder | A 相、B 相 | D7、D12 | D7 使用 PCINT23，D12 為 B 相讀取 |
| L298N | ENA、ENB（PWM） | D9、D10 | 此 Nano 不接 Servo，可使用 Timer1 PWM |
| L298N | IN1、IN2（左馬達方向） | D3、D4 | 已焊接固定 |
| L298N | IN3、IN4（右馬達方向） | D5、D6 | 已焊接固定 |
| 兩 Nano I2C | SDA、SCL | A4、A5 | 目前未接；日後可作從機位址 `0x10` |
| USB 序列 | RX、TX | D0、D1 | 上傳與除錯保留 |

### TT encoder 已驗證線色

| 線色 | 功能 | 左輪接法 | 右輪接法 |
| --- | --- | ---: | ---: |
| 黃 | Encoder VCC | Nano 5V | Nano 5V |
| 白 | Encoder GND | Nano GND | Nano GND |
| 橘 | A 相 | D2 | D7 |
| 綠 | B 相 | D8 | D12 |

馬達紅黑線分別接 L298N 的 OUT1/OUT2（左）及 OUT3/OUT4（右）。

## 導航／通訊 Nano 接線

| 模組 | 功能 | 導航 Nano 腳位 | 備註 |
| --- | --- | ---: | --- |
| MPU6050 | SDA、SCL | A4、A5 | 與驅動 Nano 共用 I2C 匯流排 |
| SG90 雲台 | 水平、垂直訊號 | D9、D10 | 使用 Servo 函式庫 |
| nRF24L01 | SCK、MOSI、MISO | D13、D11、D12 | 硬體 SPI |
| nRF24L01 | CE、CSN | D7、D8 |  |
| nRF24L01 | IRQ（選用） | D2 | 建議接上，便於低延遲收包 |
| 兩 Nano I2C | SDA、SCL | A4、A5 | 此 Nano 為 I2C 主機 |
| USB 序列 | RX、TX | D0、D1 | 上傳與除錯保留 |

## Nano 間 I2C 訊息

導航 Nano 是主機；驅動 Nano 是從機，位址 `0x10`。

| 方向 | 資料 |
| --- | --- |
| 導航 → 驅動 | 左／右目標速度、煞車或停止旗標、控制序號 |
| 驅動 → 導航 | 左／右 AB encoder count、實際輪速、狀態旗標 |

初期先以 50–100 Hz 更新 I2C 命令與 encoder 回報；encoder 中斷與輪速 PID 在驅動 Nano 本地以更高頻率運行。

## 共地與供電（重要）

```text
外部馬達電源 +  → L298N Vs / +12V
外部馬達電源 -  → L298N GND
驅動 Nano GND   → L298N GND
導航 Nano GND   → 驅動 Nano GND
外部伺服 5V GND → 導航 Nano GND
nRF24L01 GND    → 導航 Nano GND
```

- L298N 馬達供電與兩顆 SG90 的 5V 供電都不可由 Nano 5V 腳提供。
- 兩片 Nano、L298N、兩組 encoder、伺服電源與 nRF24L01 必須全數共地。
- nRF24L01 只接 **3.3V**，不可接 5V；VCC/GND 旁加 10–47 µF 電容。
- nRF24L01 的 CE、CSN、SCK、MOSI 建議使用 5V→3.3V 電平轉換；其 MISO（3.3V）可接 Nano D12。
- MPU6050 若是標示可接 5V 的 breakout（如常見 GY-521）可使用 5V；裸晶片或僅標示 3.3V 的板子，必須用 3.3V 並處理 I2C 電位。

## 實作順序

1. 分別上傳測試程式，確認兩片 Nano 都能透過 USB 上傳。
2. 在驅動 Nano 只接一顆 encoder，確認 AB 相計數與轉向；再接第二顆。
3. 接 L298N，先在低 PWM 測試每顆馬達轉向。
4. 接兩 Nano 的 A4↔A4、A5↔A5、GND↔GND，確認 I2C 命令和 encoder 回報。
5. 在導航 Nano 接 MPU6050、SG90 與 nRF24L01。
6. 最後才啟用輪速 PID 與里程計／IMU 融合。
