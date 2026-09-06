# TT 馬達、L298N 與 Encoder 接線

此資料夾保存 TT 馬達 encoder 測試程式與測得的實際接線。

| 線色／功能 | 連接位置 | 備註 |
| --- | --- | --- |
| 馬達紅線、黑線 | L298N OUT1、OUT2 | 正反方向不符時可互換 |
| Encoder 黃線（VCC） | Nano 5V | 已實測可正常讀取 encoder |
| Encoder 白線（GND） | Nano GND | 已實測可正常讀取 encoder；必須與 L298N GND 共地 |
| Encoder 橘線（A 相） | Nano D2 | Nano 外部中斷腳位 |
| Encoder 綠線（B 相） | Nano D3 | Nano 外部中斷腳位 |
| L298N IN1 | Nano D8 | 控制馬達方向 |
| L298N IN2 | Nano D9 | 控制馬達方向 |
| L298N ENA | Nano D10 | PWM 速度控制；接線前先拔掉 L298N 的 ENA 跳線帽 |

## 電源與共地

- L298N 的馬達電源使用外部電源，不由 Nano 供應。
- Nano GND、L298N GND、encoder GND 必須相連（共地）。
- 黃線為 encoder VCC（5V）、白線為 encoder GND；橘線為 A 相、綠線為 B 相。
- 連接 D10 到 ENA 時，務必先拔除 ENA 跳線帽，避免與 L298N 板上的 5V 相衝突。

## 序列監控測試

序列監控設定為 115200 baud。

| 指令 | 功能 |
| --- | --- |
| `F` | 以預設 PWM 50 正轉 |
| `B` | 以預設 PWM 50 反轉 |
| `S` | 停止馬達 |
| `M120` | 設定馬達 PWM，範圍 -255 到 255 |
| `R` | Encoder 計數歸零 |
| `P` | 顯示目前 encoder 計數 |
| `D` | 顯示 encoder A（D2）、B（D3）目前的電位 |
| `?` | 顯示指令說明 |

## 量測每圈 Encoder Count

1. 傳送 `R` 將計數歸零。
2. 手動精準轉動輪子或輸出軸一整圈。
3. 傳送 `P`。
4. `count` 的絕對值就是本程式使用的每輸出軸一圈有效計數值。

若正轉時 count 顯示負數，可將 `main.cpp` 中的 `REVERSE_ENCODER`
改成 `true`。
