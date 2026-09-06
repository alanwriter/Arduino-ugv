# 單一 Arduino Nano 車體接線規劃（舊方案）

> 此方案只讀取每顆 encoder 的 A 相。因車體後續需要完整 AB 相定位，
> 已改採兩片 Nano；請使用 [two_nano_vehicle_plan.md](two_nano_vehicle_plan.md)。

本規劃以 **一片 Arduino Nano** 控制：雙 TT 馬達、L298N、MPU6050、
兩顆 SG90 雲台伺服與 nRF24L01 無線通訊。

## Nano 腳位分配

| 模組 | 功能 | Nano 腳位 | 備註 |
| --- | --- | ---: | --- |
| L298N | ENA、ENB（共用馬達 PWM） | D3 | D3 分接到 ENA、ENB；兩個跳線帽都要拔除 |
| L298N | IN1、IN2（左馬達方向） | D4、D5 |  |
| L298N | IN3、IN4（右馬達方向） | D6、D7 |  |
| 左 TT encoder | A、B 相 | D2、D8 | 使用 pin-change interrupt |
| 右 TT encoder | A、B 相 | A0、A1 | A0/A1 當數位輸入，使用 pin-change interrupt |
| SG90 雲台 | 水平、垂直訊號 | D9、D10 | Servo 函式庫使用 Timer1；不可再拿來做 PWM |
| MPU6050 | SDA、SCL | A4、A5 | Nano 的 I²C 腳位 |
| nRF24L01 | SCK、MOSI、MISO | D13、D11、D12 | Nano 硬體 SPI 腳位 |
| nRF24L01 | CE、CSN | A2、A3 | A2/A3 當數位輸出使用 |

所有可用腳位都已分配；D0/D1 仍保留給 USB 序列監控與上傳。


## 已確認的 TT encoder 線色

每顆 TT 馬達 encoder 先依已實測的線色連接；第二顆接線前仍建議先以測試程式確認。

| encoder 線色 | 功能 | 連接位置 |
| --- | --- | --- |
| 黃 | VCC | Nano 5V |
| 白 | GND | Nano GND |
| 橘 | A 相 | 左輪 D2；右輪 A0 |
| 綠 | B 相 | 左輪 D8；右輪 A1 |

馬達紅黑線分別接其所屬 L298N channel 的 OUT1/OUT2 或 OUT3/OUT4。
若方向相反，對調該馬達的兩條紅黑線，或在程式反轉方向設定即可。

## 電源與共地（重要）

```text
外部馬達電源 +  → L298N Vs / +12V
外部馬達電源 -  → L298N GND
Nano GND         → L298N GND
外部伺服 5V GND  → Nano GND
TT encoder 白線  → Nano GND
```

- L298N 的馬達電源、兩顆 SG90 的 5V 電源都不能由 Nano 5V 腳供應。
- 所有 GND 必須共地，否則 encoder 與控制訊號不會可靠。
- D3 必須分接到 L298N 的 ENA、ENB，並移除兩個跳線帽。兩顆馬達會使用相同 PWM 速度，但可透過 IN1–IN4 獨立正轉、反轉與煞車。
- MPU6050 模組若標示可接 5V（常見 GY-521）才接 Nano 5V；若是裸 MPU6050 或僅標示 3.3V，必須用 3.3V 並處理 I²C 電位。

## nRF24L01 電源與接線（重要）

| nRF24L01 腳位 | 連接位置 |
| --- | --- |
| VCC | 穩定的 3.3V，**不可接 5V** |
| GND | Nano GND |
| CE | Nano A2 |
| CSN | Nano A3 |
| SCK | Nano D13 |
| MOSI | Nano D11 |
| MISO | Nano D12 |
| IRQ | 暫不接；目前所有可用腳位皆已分配 |

- 在 nRF24L01 的 VCC、GND 旁加一顆 10–47 µF 電容，降低無線發射時重開機或掉線的機率。
- 標準小型 nRF24L01 可使用穩定的 3.3V；若是帶天線的 PA/LNA 版本，請使用獨立、足夠電流的 3.3V 穩壓器。
- nRF24L01 腳位不是 5V 耐受。Nano 輸出的 CE、CSN、SCK、MOSI 建議經過 5V→3.3V 邏輯電平轉換；nRF 的 MISO 3.3V 可直接接 Nano D12。

## 程式實作注意事項

- 兩顆 SG90 使用 Arduino `Servo` 函式庫時，Timer1 會影響 D9/D10 的 PWM；本規劃已將共用馬達 PWM 放在 D3。
- 兩顆 encoder 都使用完整 A/B 正交解碼。D2、D8、A0、A1 需透過 pin-change interrupt 讀取。
- 若日後需要左右馬達各自做 PID PWM 控制，則不能共用 D3；可在維持一片 Nano 的前提下加 I/O 擴充器，或改用腳位更多的控制板。
- D0、D1 不配置硬體，保留給 USB 上傳與 115200 baud 序列監控。
