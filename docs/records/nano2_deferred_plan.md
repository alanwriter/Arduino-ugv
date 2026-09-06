# Nano #2 延後規劃紀錄

記錄日期：2026-09-05

## 使用者決定

目前所有實作、接線、測試與除錯只處理 **Nano #1**。

Nano #2 的導航／通訊／雲台構想延後至少一個月，預計於 2026-10-05 後再討論；
在此之前不要為 Nano #2 修改目前的 Nano #1 韌體或接線。

## 目前不可做的事

- 不要把 Nano #1 與 Nano #2 的 A4/A5 直接接在一起。
- 不要假定目前 `src/main.cpp` 已支援 Nano 間 I2C 或 Nano #2 的從機位址。
- 不要把 Nano #2 的預留腳位表當成已實作、可直接運作的韌體介面。

## 未來恢復討論時的議題

1. Nano #2 是否仍需要存在，或由 Raspberry Pi 直接負責導航／通訊。
2. 若保留 Nano #2，它應負責雲台、nRF24L01、MPU6050 或其他工作中的哪些部分。
3. Nano #1 ↔ Nano #2 的通訊方式（USB Serial、I2C 或其他）與明確訊息協定。
4. MPU6050 應保留在 Nano #1，或移到 Nano #2；目前已驗證的 Nano #1 韌體使用
   SDA=A4、SCL=A5，並將 Z 軸作為車體 yaw。
5. 新增第二片控制器後的供電、共地與失效安全策略。

## 參考資料

- `docs/current_pin_map.md`：目前實際 Nano #1 腳位與 Nano #2 的舊預留表。
- `docs/two_nano_vehicle_plan.md`：早期兩 Nano 架構草案；不代表目前已實作。
- `docs/raspberry_pi_handoff.md`：Raspberry Pi 作為上層控制器的交接資料。
