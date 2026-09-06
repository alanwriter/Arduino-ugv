# 目前硬體腳位總表（以實際接線為準）

## Nano #1：車體驅動、L298N、雙 TT encoder

此片 Nano 的 L298N IN1–IN4 已焊接固定，以下配置不可變更。

| 腳位 | 目前用途 |
| --- | --- |
| D0、D1 | USB Serial 與 Raspberry Pi；不接其他硬體 |
| D2 | 左輪 encoder A（INT0） |
| D3 | L298N IN1（左輪方向） |
| D4 | L298N IN2（左輪方向） |
| D5 | L298N IN3（右輪方向） |
| D6 | L298N IN4（右輪方向） |
| D7 | 右輪 encoder A（PCINT23） |
| D8 | 左輪 encoder B |
| D9 | L298N ENA（左輪 PWM） |
| D10 | L298N ENB（右輪 PWM） |
| D11 | 未使用 |
| D12 | 右輪 encoder B |
| D13 | 未使用 |
| A0–A3、A6、A7 | 目前未使用；A6/A7 僅能作類比輸入 |
| A4、A5 | 週一的單 Nano 閉迴路展示接 MPU6050 SDA、SCL；日後再改作兩 Nano I2C |

目前的週一展示版使用 Nano #1 直接接 MPU6050，完整接線、校正與安全測試順序請看
[nano1_closed_loop_demo.md](nano1_closed_loop_demo.md)。

TT encoder 線色曾暫定為：黃 = 5V、白 = GND、橘 = A、綠 = B；但線色不是通用規格，因使用者發現另一台車的線序可能不同，現階段必須以 encoder 小板絲印／量測再確認，不能據此直接接 L298N 或 Nano 5V。

## Nano #2：雲台／導航／通訊

目前啟用的是雲台韌體，已實際使用下列腳位：

| 腳位 | 目前用途 |
| --- | --- |
| D9 | 水平 Pan SG90 訊號 |
| D10 | 垂直 Tilt SG90 訊號 |
| D0、D1 | USB Serial；不接其他硬體 |

兩顆 SG90 使用外部 5V，外部電源 GND 與 Nano GND 必須共地。

後續接 nRF24L01 與 MPU6050 時，預留配置如下（尚未由目前雲台韌體使用）：

| 模組 | 腳位 |
| --- | --- |
| nRF24L01 SCK、MOSI、MISO | D13、D11、D12 |
| nRF24L01 CE、CSN、IRQ（選用） | D7、D8、D2 |
| MPU6050 SDA、SCL | A4、A5 |
| 與 Nano #1 I2C | A4、A5（SDA 對 SDA、SCL 對 SCL） |

Nano #2 其餘可用：D3、D4、D5、D6、A0、A1、A2、A3。

## 共地與電源

- L298N 馬達電源使用外部電源，Nano #1 GND 與 L298N GND 共地。
- SG90 使用獨立外部 5V，Nano #2 GND 與伺服電源 GND 共地。
- 兩 Nano、L298N、encoder、伺服與 nRF24L01 最終需共地。
- nRF24L01 只能使用 3.3V，不可接 5V。
