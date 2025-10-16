# 跳跳星 (JumpStar) - AI 編碼指引

## 專案概述

這是一個基於 Arduino UNO 的互動式跳躍感測裝置，使用 PlatformIO 開發環境。專案核心功能是透過**超音波感測器 HC-SR04** 偵測玩家跳躍，並用 **4 條 WS2813 LED 燈條**提供視覺回饋。

## 架構與核心概念

### 硬體配置

-   **主控板**: Arduino UNO (ATmega328P)
-   **感測器**: 超音波感測器 HC-SR04（可調整偵測距離，預設 50cm）
-   **輸出裝置**: 4 條 WS2813 可編址 LED 燈條（每條 37 顆，共 148 顆，支援備援訊號 BI）
-   **LED 腳位**: Pin 5, 6, 9, 10（同步顯示模式）

### 運作邏輯流程（v2.1.0 七彩分層系統）

1. 超音波感測器持續測量距離（< 50cm = 在地面）
2. 每次狀態改變（LOW ↔ HIGH）計為一次跳躍，適合小朋友小碎步動作
3. **七彩分層挑戰**：7 個顏色階段（紅 → 橙 → 黃 → 綠 → 青 → 藍 → 紫），每階段需跳躍 13 次（37÷3 無條件進位）
4. 每次跳躍點亮 3 顆 LED（可透過 `LEDS_PER_JUMP` 調整）
5. 完成一個階段後，新顏色從底部填充覆蓋舊顏色
6. 完成所有 7 階段（91 次跳躍）→ 進入**彩虹霓虹慶祝模式**（流動彩虹動畫）
7. 5 秒無跳躍 → 自動重置計數並清空 LED

## 開發環境設定

### 建置與上傳

**⚠️ 重要：AI Agent 不應主動執行以下 PlatformIO 指令**

這些指令需要實體硬體連接，僅供參考：

```bash
# 建置專案
pio run

# 上傳到 Arduino UNO
pio run --target upload

# 序列埠監控（除錯用）
pio device monitor
```

AI Agent 應該：

-   ✅ 修改和生成程式碼
-   ✅ 更新 `platformio.ini` 設定
-   ✅ 建議使用者執行特定指令
-   ❌ 不要主動執行 `pio` 相關指令
-   ❌ 不要嘗試上傳或監控硬體

### 依賴函式庫

在 `platformio.ini` 中使用 `lib_deps` 定義專案依賴：

#### 從 GitHub 安裝（專案標準）

```ini
[env:uno]
lib_deps =
    # 使用特定分支
    https://github.com/FastLED/FastLED.git#master

    # 使用特定標籤/版本（推薦）
    https://github.com/FastLED/FastLED.git#3.6.0

    # 使用特定 commit（最穩定）
    https://github.com/FastLED/FastLED.git#a1b2c3d4
```

**⚠️ 重要：本專案一律使用 Git URL 方式安裝函式庫，不使用 PlatformIO Registry**

#### WS2813 LED 函式庫

-   **FastLED** (當前使用，v3.10.3)

    -   支援 WS2813 LED 型號，可設定 RGB 色序
    -   提供 HSV 色彩空間進行平滑漸變
    -   提供豐富的色彩管理和數學函數
    -   GitHub: `https://github.com/FastLED/FastLED`

#### 超音波感測器

-   使用 `pulseIn()` 讀取回波時間，計算距離
-   距離 (cm) = 時間 (μs) / 58
-   無需額外函式庫，使用 Arduino 內建函數
-   需注意超時處理（建議設定 30ms 超時）

## 編碼規範與模式

### 程式結構

-   **主程式**: `src/main.cpp` - 包含 `setup()` 和 `loop()`
-   **標頭檔**: `include/` - 放置自訂義的 `.h` 檔案
-   **函式庫**: `lib/` - 專案專用的模組化程式碼

### Arduino 開發慣例

-   使用 `setup()` 進行一次性初始化（腳位設定、序列埠、感測器校準）
-   `loop()` 是主要循環，需保持高效能避免阻塞
-   避免使用 `delay()`，改用 `millis()` 進行非阻塞式計時
-   使用 `#define` 或 `const` 定義硬體腳位和常數

### 狀態管理模式（當前實作）

當前使用**簡化狀態機**，只追蹤穩定狀態變化（適合小朋友快速動作）：

```cpp
bool sensorState = HIGH;          // 當前讀值
bool lastStableState = HIGH;      // 上次穩定狀態
// 任何狀態改變（LOW↔HIGH）都算一次跳躍
if (sensorState != lastStableState) {
    jumpCount++;  // 累積跳躍
    lastStableState = sensorState;
}
```

### 七彩階段計算邏輯（核心演算法）

```cpp
// 階段參數定義
const int NUM_COLOR_STAGES = 7;       // 七彩階段數量
const int LEDS_PER_JUMP = 3;          // 每跳點亮數量
const int JUMPS_PER_STAGE = (NUM_LEDS + LEDS_PER_JUMP - 1) / LEDS_PER_JUMP;  // 無條件進位
const int MAX_JUMP_COUNT = JUMPS_PER_STAGE * NUM_COLOR_STAGES;  // 總跳躍次數

// 當前階段與進度
int currentStage = min(jumpCount / JUMPS_PER_STAGE, NUM_COLOR_STAGES - 1);
int jumpInCurrentStage = jumpCount % JUMPS_PER_STAGE;
int currentLEDCount = min(jumpInCurrentStage * LEDS_PER_JUMP, NUM_LEDS);

// 顏色分配：每個位置根據當前階段選擇顏色
if (position < ledsInCurrentStage) {
    return CHSV(RAINBOW_COLORS[currentStage], 255, 255);  // 當前階段顏色
} else if (currentStage > 0) {
    return CHSV(RAINBOW_COLORS[currentStage - 1], 255, 255);  // 前階段顏色（保留舊色）
}
```

## 關鍵開發注意事項

### 感測器處理

-   超音波感測器使用 `pulseIn()` 讀取回波時間
-   需要考慮訊號去彈跳（debouncing）避免誤觸發
-   距離轉換為數位訊號：< 50cm = LOW（在地面），≥ 50cm = HIGH（離地）
-   超時處理：`pulseIn()` 設定 30ms 超時，返回 0 時視為超出範圍

### LED 控制最佳實務（4 條燈條架構）

-   **多條燈條管理**：使用 2D 陣列 `CRGB leds[NUM_LED_STRIPS][NUM_LEDS]`
-   **同步模式** (`SYNC_MODE = true`)：4 條燈條顯示相同內容，遍歷設定每條
    ```cpp
    for (int i = 0; i < currentLEDCount; i++) {
        CRGB color = getColorForPosition(i, NUM_LEDS);
        for (int strip = 0; strip < NUM_LED_STRIPS; strip++) {
            leds[strip][i] = color;
        }
    }
    ```
-   **FastLED 初始化**：每條燈條獨立 `addLeds()`，使用 RGB 色序
    ```cpp
    FastLED.addLeds<WS2813, 5, RGB>(leds[0], NUM_LEDS);   // 燈條 1
    FastLED.addLeds<WS2813, 6, RGB>(leds[1], NUM_LEDS);   // 燈條 2
    // ... Pin 9, 10 同理
    ```
-   **七彩顏色定義**：使用 HSV 色相值陣列，色相間距加大增加辨識度
    ```cpp
    const uint8_t RAINBOW_COLORS[7] = {0, 28, 64, 96, 140, 170, 200};
    ```
-   **彩虹霓虹動畫**：循環更新色相值製造流動效果
    ```cpp
    rainbowHue += RAINBOW_HUE_STEP;  // 每 30ms 更新
    uint8_t hue = rainbowHue + (i * 256 / NUM_LEDS);  // 每顆 LED 錯開色相
    ```
-   WS2813 BI 腳位實測可空接，無需連接
-   亮度限制 50（約 20%）避免電流過大（4 條 ×37 顆=148 顆，最大 8.88A）

### 時間計算與計數系統

-   使用 `millis()` 取得毫秒級時間戳
-   注意 `millis()` 約 50 天後會溢位，需處理環繞問題
-   **階段式計數**：總跳躍次數 = `JUMPS_PER_STAGE × NUM_COLOR_STAGES`（當前 13×7=91 次）
-   LED 點亮計算公式（見上方核心演算法）
-   閒置逾時：5 秒無跳躍自動重置（在彩虹模式下也適用）

## 專案特定模式

### 中文註解友善

本專案支援繁體中文註解與命名，編碼使用 UTF-8。

### 硬體測試流程

1. 先用序列埠輸出驗證感測器讀值
2. 確認感測器正常後再整合 LED 控制
3. 分階段測試：感測器 → 計時邏輯 → LED 效果

## 已完成功能（v2.1.0）

核心功能已實作完成：

1. ✅ 4 條 WS2813 LED 燈條同步控制（148 顆 LED）
2. ✅ 七彩分層系統（7 個顏色階段，階段式挑戰）
3. ✅ 彩虹霓虹慶祝模式（完成所有階段後觸發）
4. ✅ 超音波感測器讀取與距離計算（去彈跳處理）
5. ✅ 簡化狀態機（適合快速小碎步動作）
6. ✅ 靈活 LED 點亮配置（`LEDS_PER_JUMP` 可調整）
7. ✅ 自動階段計算（根據燈條長度與點亮數量）
8. ✅ HSV 色彩空間漸變（色相間距優化）
9. ✅ 閒置自動重置機制（包含彩虹模式）
10. ✅ LED 開機診斷測試（逐顆掃描、三原色測試）

## 關鍵參數調整指南

修改遊戲體驗時，調整這些參數：

```cpp
// 燈條配置
const int NUM_LED_STRIPS = 4;              // 燈條數量
const int NUM_LEDS = 37;                   // 每條 LED 數量
const bool SYNC_MODE = true;               // true=同步, false=獨立控制

// 遊戲難度
const int NUM_COLOR_STAGES = 7;            // 顏色階段數（影響總挑戰長度）
const int LEDS_PER_JUMP = 3;               // 每跳點亮數量（越大越容易）
// JUMPS_PER_STAGE 和 MAX_JUMP_COUNT 會自動計算

// 顏色定義（HSV 色相值 0-255）
const uint8_t RAINBOW_COLORS[7] = {0, 28, 64, 96, 140, 170, 200};

// 彩虹動畫速度
const unsigned long RAINBOW_UPDATE_INTERVAL = 30;  // ms
const uint8_t RAINBOW_HUE_STEP = 2;               // 色相變化量

// 其他
const int DETECTION_DISTANCE = 50;         // 偵測距離閾值（cm）
const unsigned long IDLE_TIMEOUT = 5000;   // 閒置重置時間（ms）
```

## 待實作功能參考

參考 `README.md` 中的待開發清單：

1. 獨立控制模式實作（4 條燈條顯示不同內容）
2. 紅外線感測器整合與快速切換
3. 多種遊戲模式（計時、限時、雙人 PK）
4. 最高紀錄保存（EEPROM）
5. 難度等級選擇（階段數、點亮數量）
6. 外部按鈕控制（模式切換、重置）
7. 音效輸出（配合階段完成）
8. 階段完成慶祝動畫
