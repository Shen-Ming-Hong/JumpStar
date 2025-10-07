# 🚀 跳跳星快速參考卡 v1.2.0

> 一頁式快速查詢表，包含所有常用配置與指令

---

## 📌 感測器配置

### 切換感測器（編輯 `src/main.cpp`）

```cpp
// 超音波模式（推薦，偵測範圍大）
#define USE_ULTRASONIC true

// 紅外線模式（精確短距離）
#define USE_ULTRASONIC false
```

### 調整超音波距離

```cpp
const int DETECTION_DISTANCE = 30;  // 公分（10-100）
```

**建議值**：

-   兒童：40 cm
-   成人：30 cm
-   運動員：20 cm

---

## 🔌 硬體接線

### 超音波（HC-SR04）

```
VCC  → Arduino 5V
GND  → Arduino GND
Trig → D3
Echo → D4
```

### 紅外線

```
VCC → Arduino 5V
GND → Arduino GND
OUT → D2
```

### LED 燈條（WS2812B）

```
DIN → 330Ω 電阻 → D6
5V  → 外部 5V 電源（≥2A）
GND → Arduino GND + 外部電源 GND
```

⚠️ **30 顆 LED 必須使用外部電源！**

---

## 💻 常用指令

### 編譯與上傳

```bash
# 編譯專案
pio run

# 上傳到 Arduino
pio run --target upload

# 編譯 + 上傳
pio run -t upload
```

### 監控與除錯

```bash
# 開啟序列埠監控
pio device monitor

# 上傳後自動監控
pio run -t upload && pio device monitor
```

### 清理專案

```bash
# 清理編譯檔案
pio run --target clean
```

---

## 🎨 LED 參數

### 基本設定

```cpp
const int NUM_LEDS = 30;           // LED 數量
const int LED_BRIGHTNESS = 50;     // 亮度（0-255）
const int LED_PIN = 6;             // 資料腳位
```

### 跳躍參數

```cpp
const unsigned long MAX_AIR_TIME = 900;      // 最大滯空時間（ms）
const unsigned long DISPLAY_DURATION = 3000; // 顯示時間（ms）
const unsigned long DEBOUNCE_DELAY = 50;     // 去彈跳（ms）
```

---

## 📊 序列埠輸出解讀

### 超音波模式

```
✅ 使用超音波感測器模式
✅ 偵測距離閾值: 30 cm
[感測器] 數位訊號: LOW (有人在地面) | 距離: 15 cm
[狀態] 玩家離地！
[狀態] 玩家著地！離地時間: 450 ms
[燈效] 點亮 LED 數量: 15 / 30 (滯空時間: 450 ms)
```

### 紅外線模式

```
✅ 使用紅外線感測器模式
[感測器] 數位訊號: LOW (有人在地面)
[狀態] 玩家離地！
[狀態] 玩家著地！離地時間: 320 ms
[燈效] 點亮 LED 數量: 10 / 30 (滯空時間: 320 ms)
```

---

## 🔧 常見問題速查

| 問題           | 可能原因   | 快速解法                     |
| -------------- | ---------- | ---------------------------- |
| LED 不亮       | 電源不足   | 使用外部 5V 2A 電源          |
| 超音波顯示 999 | 接線錯誤   | 檢查 Trig→D3, Echo→D4        |
| 無法偵測跳躍   | 閾值太大   | 降低 `DETECTION_DISTANCE`    |
| 顏色不對       | 色序錯誤   | 改 `GRB` 為 `RGB` 或 `BRG`   |
| 訊號不穩定     | 去彈跳不足 | 增加 `DEBOUNCE_DELAY` 到 100 |
| 距離不準       | 角度不對   | 確保感測器垂直向上           |

---

## 📐 計算公式

### LED 數量計算

```
LED 數量 = (滯空時間 × 30) ÷ 900
範圍：1-30 顆
```

### 顏色計算（動態漸變）

```
色相 (Hue) = 96 - (LED位置 / 總數) × 96
96 = 綠色，64 = 黃色，0 = 紅色
```

### 超音波距離計算

```
距離 (cm) = 回波時間 (μs) ÷ 58
音速 ≈ 340 m/s，來回距離
```

---

## 🎯 測試檢查清單

### 硬體測試

-   [ ] LED 開機彩虹動畫正常
-   [ ] 感測器供電正常（5V）
-   [ ] 所有接線穩固無鬆動

### 感測器測試

-   [ ] 站立時顯示 LOW（有人）
-   [ ] 離地時顯示 HIGH（無人）
-   [ ] 超音波距離值合理（<100cm）

### 功能測試

-   [ ] 能正確偵測離地時刻
-   [ ] 能正確偵測著地時刻
-   [ ] LED 數量符合滯空時間
-   [ ] 顏色平滑漸變（綠 → 黃 → 紅）
-   [ ] 顯示 3 秒後自動清空

---

## 📚 文件快速連結

| 需求            | 文件                      |
| --------------- | ------------------------- |
| 第一次使用      | `QUICK_START.md`          |
| 選擇/切換感測器 | `SENSOR_CONFIGURATION.md` |
| 接線安裝        | `HARDWARE_SETUP.md`       |
| 測試驗證        | `TESTING_GUIDE.md`        |
| 修改程式        | `CODE_DOCUMENTATION.md`   |
| 版本更新        | `CHANGELOG_v1.2.0.md`     |
| 完整說明        | `README.md`               |

---

## ⚙️ 進階配置

### 改變 LED 色序

```cpp
// 常見色序：GRB, RGB, BRG
FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
```

### 調整亮度限制

```cpp
FastLED.setBrightness(LED_BRIGHTNESS);  // 0-255
// 建議：30顆用50，60顆用30
```

### 延長顯示時間

```cpp
const unsigned long DISPLAY_DURATION = 5000;  // 5秒
```

### 調整最大滯空時間

```cpp
const unsigned long MAX_AIR_TIME = 1200;  // 1.2秒
// 影響 LED 映射：時間 × 30 ÷ MAX_AIR_TIME
```

---

## 🔢 規格速查

| 項目         | 數值             |
| ------------ | ---------------- |
| Arduino 型號 | UNO (ATmega328P) |
| Flash 使用   | 9.5 KB / 32 KB   |
| SRAM 使用    | 184 B / 2 KB     |
| LED 數量     | 30 顆            |
| 序列埠速率   | 9600 baud        |
| 去彈跳時間   | 50 ms            |
| Loop 頻率    | ~100 Hz          |
| 超音波範圍   | 2-400 cm         |
| 紅外線範圍   | 3-8 cm           |

---

## 💡 效能優化提示

1. **減少序列埠輸出** → 提升 10-20% 速度
2. **降低 LED 數量** → 節省記憶體
3. **使用外部中斷** → 更快響應（進階）
4. **預先計算色彩** → 減少 CPU 負擔
5. **條件編譯除錯** → 縮小程式體積

---

## 🆘 緊急除錯

### 完全無反應

```bash
# 1. 確認連接埠
pio device list

# 2. 重新上傳
pio run -t upload

# 3. 檢查序列埠
pio device monitor
```

### 重置為預設值

```cpp
#define USE_ULTRASONIC true
const int DETECTION_DISTANCE = 30;
const int NUM_LEDS = 30;
const int LED_BRIGHTNESS = 50;
const unsigned long MAX_AIR_TIME = 900;
const unsigned long DISPLAY_DURATION = 3000;
```

---

**快速參考卡 v1.2.0**  
**列印友善版 | 最後更新：2025-10-07**
