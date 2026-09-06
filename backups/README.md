# 專案紀錄區

每個資料夾都是一個可獨立還原的功能版本，內含當時的 `main.cpp`、
`platformio.ini` 與接線／操作說明。

| 資料夾 | 已保存功能 |
| --- | --- |
| `gimbal_serial/` | 二軸 SG90 雲台，可由序列監控指定角度 |
| `tt_motor_encoder/` | 六線 TT 馬達 encoder 讀值與 L298N 測試程式 |
| `nano1_l298n_motor_test/` | Nano #1 固定腳位的雙 L298N 馬達正反轉測試 |

目前的 `src/main.cpp` 已保留為下一個功能的乾淨起點。
