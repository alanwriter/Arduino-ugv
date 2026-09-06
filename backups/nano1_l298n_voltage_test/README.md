# Nano #1 L298N 電壓測試備份

這是切換到單顆 encoder 讀值韌體前，`src/main.cpp` 的完整備份。

固定腳位：D3–D6 = IN1–IN4、D9 = ENA PWM、D10 = ENB PWM。

序列指令：`F` 前進、`S` 停止、`0..255` 設定 PWM、`?` 顯示說明。
