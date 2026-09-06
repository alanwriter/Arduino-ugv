# Nano #1：單顆 TT encoder 測試

> **封存的診斷流程。** 本文件記錄先前的單顆 encoder 專用韌體；目前的
> src/main.cpp 已改為雙 encoder、L298N、MPU6050 的閉迴路展示程式。
> 請使用 [nano1_closed_loop_demo.md](nano1_closed_loop_demo.md) 的 D 指令
> 做雙輪 AB 相檢查。不要依本文件宣稱的「目前馬達一定被強制關閉」來接線或上傳。

## Nano 接線

| Nano | 接到該顆 encoder |
| --- | --- |
| D2 | A 相 |
| D8 | B 相 |
| 5V | encoder VCC（**僅在已確認 VCC 線時才接**） |
| GND | encoder GND（**僅在已確認 GND 線時才接**） |

馬達的兩條動力線這次不需要接 L298N；可以整組拔開，避免干擾。

> 線色沒有通用規格。先前測試曾暫定黃 = VCC、白 = GND、橘 = A、綠 = B，但若你懷疑線序不同，先不要把未知線接到 Nano 5V 或 L298N，請先用馬達背面 encoder 小板絲印、產品連結或量測確認。

## 操作

1. 關閉 Serial Monitor 後上傳程式。
2. 開啟 115200 baud Serial Monitor。
3. 傳送 `R`，將計數歸零。
4. 用手慢慢轉動該顆輪子，觀察每 0.5 秒輸出的 `count` 和 `rate`。
5. 傳送 `P` 可立即看到 A/B 電位、有效邊緣數與無效跳變數。

範例：

```text
A=1,B=0,count=24,edges=24,A_edges=12,B_edges=12,invalid=0
count=28,rate=8.0 ticks/s
```

- `count`：四倍頻 AB quadrature 計數；反轉時正負號會相反。
- `edges`：成功辨識的 A/B 邊緣數。
- `A_edges`、`B_edges`：各自訊號線實際跳變的次數；手轉時兩者都應增加。若只有其中一個增加，另一條 A/B 線未接對、斷線或沒有訊號。
- `invalid`：A/B 同時跳變或遺漏脈波的次數；手轉很慢時應接近 0。
- 若 `A`、`B` 永遠是 `1`，通常表示 encoder 未供電、A/B 線接錯，或沒有接到對應輸出。

## 指令

| 指令 | 功能 |
| --- | --- |
| `R` | 歸零 count、edges、invalid |
| `P` | 立即列印狀態 |
| `?` | 顯示說明 |

原本的 L298N 電壓測試韌體已完整保存在 `backups/nano1_l298n_voltage_test/main.cpp`。
