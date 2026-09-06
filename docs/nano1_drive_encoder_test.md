# Nano #1：L298N 與雙 TT Encoder 測試

此文件是 Nano #1 的實際硬體配置，L298N 的 IN1–IN4 已焊接完成，
不得更換腳位。

> **舊版測試架構。** 目前的 src/main.cpp 是單片 Nano #1 的閉迴路展示程式，
> 會在明確命令後驅動馬達；完整接線、硬體 pull-down、安全驗證與指令請改看
> [nano1_closed_loop_demo.md](nano1_closed_loop_demo.md)。

## 固定腳位表

| Nano 腳位 | 用途 |
| --- | --- |
| D0、D1 | USB Serial RX/TX；不接其他設備 |
| D2 | 左 encoder A（INT0） |
| D3–D6 | L298N IN1–IN4（已固定焊接） |
| D7 | 右 encoder A（PCINT23） |
| D8 | 左 encoder B |
| D9、D10 | L298N ENA、ENB PWM |
| D11 | 未使用 |
| D12 | 右 encoder B |
| D13 | 未使用 |
| A0–A3、A6、A7 | 未使用 |
| A4、A5 | 目前單 Nano 展示用 MPU6050 SDA、SCL |

| 馬達 | PWM | 方向 |
| --- | --- | --- |
| 左輪 | ENA = D9 | IN1 = D3、IN2 = D4 |
| 右輪 | ENB = D10 | IN3 = D5、IN4 = D6 |

## 舊版雙 encoder-only 測試

此節描述舊版雙 encoder 測試架構，不是目前 src/main.cpp 的指令介面。若需檢查
目前韌體的四個 encoder 訊號，使用新韌體的 D 指令。

以 115200 baud 開啟 Raspberry Pi 或電腦的 USB Serial 後，可用：

| 指令 | 功能 |
| --- | --- |
| `R` | 左右 encoder 計數歸零 |
| `P` | 立即顯示左右 count |
| `D` | 顯示 LA/LB/RA/RB 四個 encoder 訊號電位與邊緣計數 |
| `?` | 顯示指令說明 |

韌體每 250 ms 輸出一次：

```text
left=123,left_tps=42.0,right=-120,right_tps=-40.0
```

- `left`、`right`：以 A 相 CHANGE 的 2 倍邊緣計數。
- `left_tps`、`right_tps`：每秒計數變化量，正負號表示 AB 相判斷的方向。
- 左輪 D2 使用標準外部中斷；右輪 D7 使用 ATmega328P 的 PCINT23，未使用額外函式庫。
- `D` 的 `edges LA/LB/RA/RB` 用來排除接線問題。傳送 `R` 後手動轉動輪子，對應 A、B 的 edge 值都應增加；若兩者都維持 0，該 encoder 沒有供電或訊號線未接對。

## 測試流程

1. 上傳本韌體並開啟 115200 baud USB Serial。
2. 傳送 `R`。
3. 分別手動轉動左、右輸出軸；對應的 `left` 或 `right` 必須改變。
4. 向相反方向轉動，計數應改變為相反正負號。
5. 若只有正負號與預期方向相反，將 `main.cpp` 的對應
   `REVERSE_LEFT_ENCODER` 或 `REVERSE_RIGHT_ENCODER` 改為 `true`。
