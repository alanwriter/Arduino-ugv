# Nano 雙軸 SG90 雲台：Python 操作手冊

此手冊對應目前的 `src/main.cpp` 雲台韌體。Nano 以 USB Serial 115200 baud
接收 `pan,tilt` 指令，例如 `120,45`。

## 1. 硬體接線

| 功能 | 連接 |
| --- | --- |
| 水平 Pan SG90 訊號 | Nano D9 |
| 垂直 Tilt SG90 訊號 | Nano D10 |
| 兩顆 SG90 V+ | 外部穩定 5V 正極 |
| 兩顆 SG90 GND | 外部 5V GND |
| Nano GND | 外部 5V GND（共地） |

兩顆 SG90 不可由 Nano 的 5V 腳供電。先在雷射關閉時完成所有雲台測試。

## 2. 上傳 Nano 韌體

1. 關閉任何正在使用 Nano 的 Serial Monitor。
2. 在 PlatformIO 對目前專案執行 Upload。
3. 韌體上傳後可用 Serial Monitor（115200 baud）確認：

```text
2-axis gimbal ready.
PAN=90, TILT=90
```

可手動傳送：

```text
120,45    # 先移動 Pan 到 120°，再移動 Tilt 到 45°
P120      # 只移動 Pan
T45       # 只移動 Tilt
C         # 兩軸回 90°
?         # 顯示指令
```

程式會把角度限制在 0–180°。若機構碰到限位，立刻停止，之後再收窄
`MIN_ANGLE`、`MAX_ANGLE` 的範圍。

## 3. 建立 Python 專案環境

在你的 Python 專案資料夾執行：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install pyserial
```

將 [gimbal_serial_control.py](../python_examples/gimbal_serial_control.py) 與
`python_examples/requirements.txt` 複製到 Python 專案；或直接從本工作區執行。

Mac 可用下列方式確認 Nano 的連接埠：

```bash
ls /dev/cu.usbserial* /dev/cu.usbmodem* 2>/dev/null
```

目前常見的 Nano 連接埠是 `/dev/cu.usbserial-120`；拔插 USB 後名稱可能改變。

## 4. Python 指令列操作

**任何 Python 程式開啟連接埠前，先關閉 PlatformIO Serial Monitor。**

一次設定角度：

```bash
python gimbal_serial_control.py --port /dev/cu.usbserial-120 --set 120 45
```

回中央：

```bash
python gimbal_serial_control.py --port /dev/cu.usbserial-120 --center
```

互動模式：

```bash
python gimbal_serial_control.py --port /dev/cu.usbserial-120
```

互動模式可輸入 `120,45`、`P120`、`T45`、`C`、`?`，輸入 `q` 結束。

## 5. 在視覺程式中使用

```python
from gimbal_serial_control import Gimbal

with_gimbal = Gimbal("/dev/cu.usbserial-120")
try:
    print(with_gimbal.set_angles(120, 45))
finally:
    with_gimbal.close()
```

雲台韌體以每度 15 ms 平順移動，較大的角度變化需要數秒。視覺控制時請避免在前一個
命令尚未完成前累積大量不同命令；先完成四角校正與低頻率測試，再調整更新頻率。

## 6. 故障排除

| 現象 | 處理方式 |
| --- | --- |
| `Resource busy`／無法開啟 port | 關閉 PlatformIO Serial Monitor 或其他 Python 程式。 |
| Python 開啟後沒有回覆 | 確認 port、115200 baud，並等待約 2 秒讓 Nano 重置完成。 |
| Nano 重啟、舵機亂抖 | 伺服電源不足；改用獨立穩定 5V，並確認共地。 |
| 方向相反或撞到限位 | 先停止，再交換機構方向或縮小角度限制；不要讓 SG90 長時間卡住。 |
