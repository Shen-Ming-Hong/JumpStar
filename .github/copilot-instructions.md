# 跳跳星 (JumpStar) - AI 編碼指引

## 專案概述

這是一個基於 Arduino UNO 的互動式跳躍感測裝置，使用 PlatformIO 開發環境。專案核心功能是透過紅外線感測器偵測玩家跳躍，並用 WS2812B LED 燈條提供視覺回饋。

## 架構與核心概念

### 硬體配置

-   **主控板**: Arduino UNO (ATmega328P)
-   **感測器**: 紅外線數位感測器（用於地面偵測）
-   **輸出裝置**: WS2812B 可編址 LED 燈條

### 運作邏輯流程

1. 紅外線感測器持續監測玩家是否在地面
2. 偵測到玩家離地 → 開始計時
3. 偵測到玩家著地 → 停止計時，計算離地時間
4. 根據離地時間推算跳躍高度
5. 依高度映射對應的 LED 燈光效果

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

#### WS2812B LED 函式庫選擇

-   **FastLED** (推薦)

    -   更高效能，支援多種 LED 型號
    -   提供豐富的色彩管理和數學函數
    -   GitHub: `https://github.com/FastLED/FastLED`

-   **Adafruit NeoPixel**
    -   更簡單易用，適合初學者
    -   較佔記憶體，但穩定性高
    -   GitHub: `https://github.com/adafruit/Adafruit_NeoPixel`

#### 紅外線感測器

-   使用數位 I/O (`digitalRead()`)，通常無需額外函式庫
-   如需進階功能可考慮去彈跳函式庫：`https://github.com/thomasfredericks/Bounce2`

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

### 狀態管理建議

對於跳躍偵測，建議使用狀態機模式：

```cpp
enum JumpState { GROUNDED, IN_AIR, LANDING };
JumpState currentState = GROUNDED;
unsigned long jumpStartTime = 0;
```

## 關鍵開發注意事項

### 感測器處理

-   紅外線感測器讀取使用 `digitalRead()`
-   需要考慮訊號去彈跳（debouncing）避免誤觸發
-   建議實作校準機制以適應不同環境光線

### LED 控制最佳實務

-   WS2812B 需要精確時序，某些操作會短暫中斷
-   更新 LED 時使用緩衝區模式，計算完再一次送出
-   考慮亮度限制以避免電流過大

### 時間計算

-   使用 `millis()` 取得毫秒級時間戳
-   注意 `millis()` 約 50 天後會溢位，需處理環繞問題
-   跳躍時間到高度的映射公式需要實測校準

## 專案特定模式

### 中文註解友善

本專案支援繁體中文註解與命名，編碼使用 UTF-8。

### 硬體測試流程

1. 先用序列埠輸出驗證感測器讀值
2. 確認感測器正常後再整合 LED 控制
3. 分階段測試：感測器 → 計時邏輯 → LED 效果

## 待實作功能參考

參考 `README.md` 中的待開發清單，優先順序：

1. 紅外線感測器基礎讀取與校準
2. 跳躍狀態偵測邏輯（含去彈跳）
3. 計時與高度計算
4. WS2812B 初始化與基本控制
5. 高度與燈效映射算法
6. 進階遊戲模式設計
