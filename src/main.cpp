#include <Arduino.h>
#include <FastLED.h>

// ============================================================================
// 硬體常數定義
// ============================================================================

// ========== 超音波感測器設定 ==========
const int TRIG_PIN = 3;            // 超音波 Trig 腳位（D3）
const int ECHO_PIN = 4;            // 超音波 Echo 腳位（D4）
const int DETECTION_DISTANCE = 50; // 偵測距離閘值（公分）

// ========== WS2813 LED 燈條設定 ==========
const int NUM_LED_STRIPS = 4;                       // LED 燈條數量
const int LED_PINS[NUM_LED_STRIPS] = {5, 6, 9, 10}; // LED 資料腳位 (D5, D6, D9, D10)
const int NUM_LEDS = 37;                            // 每條燈條的 LED 燈珠數量
const int LED_BRIGHTNESS = 50;                      // 亮度限制(0-255,建議不超過 50% 避免電流過載)
const bool SYNC_MODE = true;                        // 同步模式:true=4條顯示相同, false=獨立控制

// ⚠️ WS2813 實際接線配置(4 條燈條版本):
//
// 每條 WS2813 燈條的訊號線(輸入端):
//   - DI (Data Input)   → Arduino Pin (5/6/9/10)(主要資料輸入)
//   - BI (Backup Input) → 空接(本專案測試可正常運作)
//   - GND              → Arduino GND + 電源 GND(必須共地!)
//   - 5V               → 外部電源 5V(不要用 Arduino 供電!)
//
// 🔧 4 條燈條接線方式:
//    燈條 1: Arduino Pin 5  ------------ DI
//    燈條 2: Arduino Pin 6  ------------ DI
//    燈條 3: Arduino Pin 9  ------------ DI
//    燈條 4: Arduino Pin 10 ------------ DI
//    所有 BI ----------------------- 空接(不連接)
//    Arduino GND ------------------ GND (所有燈條共地)
//    外部電源 5V ------------------ 5V (所有燈條)
//    外部電源 GND ----------------- GND (與 Arduino GND 共地)
//
// 💡 WS2813 雙訊號說明:
//    - BI 是備援訊號輸入,當 DI 損壞時可自動切換到 BI
//    - 本專案測試結果:BI 空接也能正常工作
//    - 如需使用備援功能:將 BI 連接到 GND 或與 DI 並聯
//
// ⚡ 電源注意事項(4 條燈條):
//    - 4 條 × 37 顆 = 148 顆 LED
//    - 最大電流:148 × 60mA = 8.88A
//    - 實際使用(50% 亮度):約 4-5A
//    - 建議使用 5V 10A 以上電源供應器
//    - 大量 LED 建議每 50-100 顆注入一次電源

// 序列埠速率
const long SERIAL_BAUD = 9600;

// 去彈跳時間閾值（毫秒）
const unsigned long DEBOUNCE_DELAY = 25;

// 跳躍計數參數
const int NUM_COLOR_STAGES = 7;                                             // 七彩階段數量
const int LEDS_PER_JUMP = 3;                                                // 每次跳躍點亮的 LED 數量（可調整：1, 2, 3...）
const int JUMPS_PER_STAGE = (NUM_LEDS + LEDS_PER_JUMP - 1) / LEDS_PER_JUMP; // 每個顏色階段所需的跳躍次數
const int MAX_JUMP_COUNT = JUMPS_PER_STAGE * NUM_COLOR_STAGES;              // 總跳躍次數
const unsigned long IDLE_TIMEOUT = 5000;                                    // 無跳躍逾時時間（5 秒）
const int BLINK_COUNT = 5;                                                  // 熄滅前閃爍次數
const unsigned long BLINK_INTERVAL = 300;                                   // 閃爍間隔時間（毫秒）

// 七彩顏色定義(使用 HSV 色相值,間距加大以增加差異性)
const uint8_t RAINBOW_COLORS[NUM_COLOR_STAGES] = {
    0,   // 紅色 (Red)
    28,  // 橙色 (Orange) - 原本 32,調整為更橘
    64,  // 黃色 (Yellow)
    96,  // 綠色 (Green)
    140, // 青色 (Cyan) - 原本 128,調整為更藍
    170, // 藍色 (Blue) - 原本 160,調整為更深藍
    200  // 紫紅色 (Magenta) - 原本 192,調整為偏紅的紫
};

// 彩虹霓虹燈效參數
const unsigned long RAINBOW_UPDATE_INTERVAL = 30; // 霓虹燈更新間隔(毫秒)
const uint8_t RAINBOW_HUE_STEP = 2;               // 每次色相變化量

// ============================================================================
// 全域變數
// ============================================================================

// LED 陣列(4 條燈條)
CRGB leds[NUM_LED_STRIPS][NUM_LEDS];

// 感測器狀態（通用）
bool sensorState = HIGH;            // 當前感測器讀值
bool lastSensorState = HIGH;        // 上次感測器讀值
bool lastStableState = HIGH;        // 上次穩定狀態（用於偵測狀態改變）
unsigned long lastDebounceTime = 0; // 上次去彈跳時間戳

// 超音波相關變數
long ultrasonicDistance = 0; // 當前測得距離（cm）

// 跳躍計數相關
unsigned long jumpStartTime = 0;     // 跳躍開始時間
unsigned long lastJumpTime = 0;      // 上次跳躍時間
int jumpCount = 0;                   // 累積跳躍次數
int currentLEDCount = 0;             // 目前點亮的 LED 數量
bool isBlinking = false;             // 是否正在閃爍中
bool isRainbowMode = false;          // 是否進入彩虹霓虹模式
uint8_t rainbowHue = 0;              // 彩虹霓虹色相值
unsigned long lastRainbowUpdate = 0; // 上次霓虹燈更新時間

// ============================================================================
// 函式宣告
// ============================================================================

// 感測器讀取函式
bool readSensorWithDebounce(); // 讀取感測器（含去彈跳）
long readUltrasonicDistance(); // 讀取超音波距離（cm）
bool readUltrasonicSensor();   // 讀取超音波感測器

// 狀態機和燈效
void updateJumpState();                            // 更新跳躍狀態機
void checkIdleTimeout();                           // 檢查逾時並執行閃爍
void displayCurrentLEDs();                         // 顯示當前累積的 LED
void blinkAndClear();                              // 閃爍後清空
CRGB getColorForPosition(int position, int total); // 根據位置取得漸變顏色
void clearLEDs();                                  // 清空 LED
void testLEDPattern();                             // LED 測試動畫
void displayRainbowEffect();                       // 顯示彩虹霓虹燈效
void updateRainbowEffect();                        // 更新彩虹霓虹動畫

// ============================================================================
// 初始化設定
// ============================================================================

void setup()
{
  // 初始化序列埠（用於除錯）
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("=== 跳跳星系統啟動 ==="));
  Serial.println(F("系統初始化中..."));

  // 初始化超音波感測器
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println(F("✅ 使用超音波感測器模式"));
  Serial.print(F("✅ 偵測距離閘值: "));
  Serial.print(DETECTION_DISTANCE);
  Serial.println(F(" cm"));

  // 初始化 4 條 LED 燈條
  // 注意：某些 WS2813 使用 RGB 色序，某些使用 GRB，請根據實際情況調整
  FastLED.addLeds<WS2813, 5, RGB>(leds[0], NUM_LEDS);  // 燈條 1 (Pin 5)
  FastLED.addLeds<WS2813, 6, RGB>(leds[1], NUM_LEDS);  // 燈條 2 (Pin 6)
  FastLED.addLeds<WS2813, 9, RGB>(leds[2], NUM_LEDS);  // 燈條 3 (Pin 9)
  FastLED.addLeds<WS2813, 10, RGB>(leds[3], NUM_LEDS); // 燈條 4 (Pin 10)
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  Serial.println(F("✓ 4 條LED 燈條初始化完成"));
  Serial.print(F("✓ 燈條數量: "));
  Serial.println(NUM_LED_STRIPS);
  Serial.print(F("✓ 每條 LED 數量: "));
  Serial.println(NUM_LEDS);
  Serial.print(F("✓ 總 LED 數量: "));
  Serial.println(NUM_LED_STRIPS * NUM_LEDS);
  Serial.print(F("✓ 模式: "));
  Serial.println(SYNC_MODE ? F("同步模式") : F("獨立控制模式"));

  // 執行 LED 測試動畫
  Serial.println(F("執行 LED 測試動畫..."));
  // testLEDPattern();

  Serial.println(F("系統就緒！開始監測跳躍..."));
  Serial.println();
}

// ============================================================================
// 主迴圈
// ============================================================================

void loop()
{
  // 讀取感測器狀態（含去彈跳）
  sensorState = readSensorWithDebounce();

  // 即時顯示感測器數位訊號（用於觀察）
  static unsigned long lastPrintTime = 0;
  static bool lastPrintedState = sensorState;

  // 每 200ms 或狀態改變時顯示
  if (millis() - lastPrintTime >= 200 || sensorState != lastPrintedState)
  {
    Serial.print(F("[感測器] 數位訊號: "));
    Serial.print(sensorState ? "HIGH" : "LOW");
    Serial.print(F(" ("));
    Serial.print(sensorState ? "無人/跳躍中" : "有人在地面");
    Serial.print(F(")"));

    // 顯示超音波距離
    Serial.print(F(" | 距離: "));
    Serial.print(ultrasonicDistance);
    Serial.print(F(" cm"));

    Serial.println();

    lastPrintTime = millis();
    lastPrintedState = sensorState;
  }

  // 如果在彩虹霓虹模式,持續更新動畫
  if (isRainbowMode)
  {
    updateRainbowEffect();
  }
  else
  {
    // 更新跳躍狀態機
    updateJumpState();

    // 檢查逾時(非閃爍狀態下才檢查)
    if (!isBlinking)
    {
      checkIdleTimeout();
    }
  }

  // 小延遲以避免過度頻繁的感測器讀取
  delay(10);
}

// ============================================================================
// 感測器讀取（通用介面，含去彈跳）
// ============================================================================

bool readSensorWithDebounce()
{
  // 讀取超音波感測器
  bool reading = readUltrasonicSensor();

  // 如果讀值改變，重置去彈跳計時器
  if (reading != lastSensorState)
  {
    lastDebounceTime = millis();
  }

  // 如果讀值穩定超過去彈跳時間，更新感測器狀態
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY)
  {
    sensorState = reading;
  }

  lastSensorState = reading;
  return sensorState;
}

// ============================================================================
// 超音波感測器讀取
// ============================================================================

bool readUltrasonicSensor()
{
  // 讀取超音波距離
  ultrasonicDistance = readUltrasonicDistance();

  // 將距離轉換為數位訊號邏輯（模擬紅外線行為）
  // 距離 < 閾值 = 有人在地面 (LOW)
  // 距離 >= 閾值 = 無人/跳躍中 (HIGH)
  if (ultrasonicDistance > 0 && ultrasonicDistance < DETECTION_DISTANCE)
  {
    return LOW; // 有人在地面
  }
  else
  {
    return HIGH; // 無人/跳躍中
  }
}

// ============================================================================
// 讀取超音波距離（公分）
// ============================================================================

long readUltrasonicDistance()
{
  // 發送 10us 的觸發脈衝
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 讀取回波時間（微秒）
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms 超時

  // 計算距離：距離(cm) = 時間(μs) / 58
  // 音速 = 340 m/s，來回距離 = 時間 * 速度 / 2
  long distance = duration / 58;

  // 超時或超出範圍返回 999
  if (duration == 0 || distance > 400)
  {
    return 999;
  }

  return distance;
}

// ============================================================================
// 跳躍狀態機更新（簡化版：任何狀態改變都算跳躍）
// ============================================================================

void updateJumpState()
{
  // 簡化邏輯：只要感測器狀態改變就算跳躍
  // LOW → HIGH（離地）或 HIGH → LOW（著地）都算一次跳躍
  // 適合小朋友的小碎步動作

  // 偵測狀態改變（從穩定狀態變化到新狀態）
  if (sensorState != lastStableState)
  {
    // 更新穩定狀態
    lastStableState = sensorState;

    // 記錄跳躍時間
    lastJumpTime = millis();

    // 增加跳躍計數
    jumpCount++;

    // 計算當前顏色階段和該階段的跳躍次數
    int currentStage = min(jumpCount / JUMPS_PER_STAGE, NUM_COLOR_STAGES - 1);
    int jumpInCurrentStage = jumpCount % JUMPS_PER_STAGE;

    // 計算當前應點亮的 LED 數量
    // 每個階段最多點亮 NUM_LEDS，當前階段每跳一次亮 LEDS_PER_JUMP 顆
    if (jumpInCurrentStage == 0 && jumpCount > 0)
    {
      // 階段完成，全部點亮
      currentLEDCount = NUM_LEDS;
    }
    else
    {
      // 當前階段進行中，根據跳躍次數點亮對應數量
      currentLEDCount = min(jumpInCurrentStage * LEDS_PER_JUMP, NUM_LEDS);
    }

    // 輸出跳躍資訊
    Serial.print(F("[跳躍] 狀態改變: "));
    Serial.print(sensorState ? "LOW→HIGH (離地)" : "HIGH→LOW (著地)");
    Serial.print(F(" | 累積次數: "));
    Serial.print(jumpCount);
    Serial.print(F(" / "));
    Serial.print(MAX_JUMP_COUNT);
    Serial.print(F(" | 階段: "));
    Serial.print(currentStage + 1);
    Serial.print(F("/"));
    Serial.print(NUM_COLOR_STAGES);
    Serial.print(F(" ("));
    Serial.print(jumpInCurrentStage > 0 ? jumpInCurrentStage : JUMPS_PER_STAGE);
    Serial.print(F("/"));
    Serial.print(JUMPS_PER_STAGE);
    Serial.print(F(")"));
    Serial.print(F(" | LED: "));
    Serial.print(currentLEDCount);
    Serial.print(F(" / "));
    Serial.println(NUM_LEDS);

    // 檢查是否達到最大跳躍次數(完成所有階段)
    if (jumpCount >= MAX_JUMP_COUNT)
    {
      Serial.println(F("\n🎉🎉🎉 恭喜完成所有階段!進入彩虹霓虹模式!🎉🎉🎉\n"));
      isRainbowMode = true;
      displayRainbowEffect();
    }
    else
    {
      // 顯示當前累積的燈效
      displayCurrentLEDs();
    }
  }
}

// ============================================================================
// 檢查逾時並直接重置
// ============================================================================

void checkIdleTimeout()
{
  // 如果有累積跳躍且超過逾時時間
  if (currentLEDCount > 0 && (millis() - lastJumpTime) > IDLE_TIMEOUT)
  {
    Serial.println(F("[逾時] 超過閒置時間,直接重置跳躍計數..."));

    // 清空 LED
    clearLEDs();

    // 重置計數和狀態
    jumpCount = 0;
    currentLEDCount = 0;
    isRainbowMode = false;
    rainbowHue = 0;

    Serial.println(F("[重置] 已清空 LED 並重置跳躍計數"));
  }
}

// ============================================================================
// 根據 LED 位置取得顏色（七彩分層效果）
// ============================================================================

CRGB getColorForPosition(int position, int totalLEDs)
{
  // 七彩分層效果：每個階段填滿 NUM_LEDS 數量後才換下一個顏色
  // 新顏色從底部開始填充，逐漸覆蓋舊顏色

  // 計算當前處於第幾個階段（0-6）
  int currentStage = min(jumpCount / JUMPS_PER_STAGE, NUM_COLOR_STAGES - 1);

  // 計算當前階段已點亮的 LED 數量（每跳一次亮 LEDS_PER_JUMP 顆）
  int jumpInCurrentStage = jumpCount % JUMPS_PER_STAGE;
  int ledsInCurrentStage = min(jumpInCurrentStage * LEDS_PER_JUMP, NUM_LEDS);

  // 判斷當前 LED 位置應該顯示哪種顏色
  if (position < ledsInCurrentStage)
  {
    // 在當前階段的填充範圍內，顯示當前階段顏色
    return CHSV(RAINBOW_COLORS[currentStage], 255, 255);
  }
  else if (currentStage > 0)
  {
    // 超出當前階段範圍，顯示前一階段的顏色（保持舊顏色）
    return CHSV(RAINBOW_COLORS[currentStage - 1], 255, 255);
  }
  else
  {
    // 第一階段且尚未填充到此位置，不顯示
    return CRGB::Black;
  }
}

// ============================================================================
// 顯示當前累積的 LED
// ============================================================================

void displayCurrentLEDs()
{
  // 不需要清空 LED，讓已點亮的保持亮著，只更新顏色即可
  // clearLEDs();  // ← 移除此行避免閃爍

  // 計算當前顏色階段
  int currentStage = min(jumpCount / JUMPS_PER_STAGE, NUM_COLOR_STAGES - 1);
  int jumpInCurrentStage = jumpCount % JUMPS_PER_STAGE;

  // 輸出燈效資訊
  Serial.print(F("[燈效] 點亮 LED 數量: "));
  Serial.print(currentLEDCount);
  Serial.print(F(" / "));
  Serial.print(NUM_LEDS);
  Serial.print(F(" | 當前階段: "));
  Serial.print(currentStage + 1);
  Serial.print(F("/"));
  Serial.print(NUM_COLOR_STAGES);
  Serial.print(F(" (進度: "));
  Serial.print(jumpInCurrentStage);
  Serial.print(F("/"));
  Serial.print(JUMPS_PER_STAGE);
  Serial.println(F(")"));

  // 同步模式:4 條燈條顯示相同內容
  if (SYNC_MODE)
  {
    // 計算顏色並同步設定到 4 條燈條
    for (int i = 0; i < currentLEDCount; i++)
    {
      CRGB color = getColorForPosition(i, NUM_LEDS);
      for (int strip = 0; strip < NUM_LED_STRIPS; strip++)
      {
        leds[strip][i] = color;
      }
    }
  }
  else
  {
    // 獨立控制模式(預留給未來擴展)
    // TODO: 實作不同燈條的獨立燈效
    for (int strip = 0; strip < NUM_LED_STRIPS; strip++)
    {
      for (int i = 0; i < currentLEDCount; i++)
      {
        leds[strip][i] = getColorForPosition(i, NUM_LEDS);
      }
    }
  }

  FastLED.show();
}

// ============================================================================
// 閃爍後清空（逾時效果）
// ============================================================================

void blinkAndClear()
{
  isBlinking = true;

  Serial.print(F("[閃爍] 開始閃爍 "));
  Serial.print(BLINK_COUNT);
  Serial.println(F(" 次..."));

  // 閃爍指定次數
  for (int i = 0; i < BLINK_COUNT; i++)
  {
    // 熄滅階段
    clearLEDs();

    // 分段檢查感測器（避免錯過跳躍）
    for (int t = 0; t < BLINK_INTERVAL; t += 10)
    {
      delay(10);

      // 檢查是否有新的跳躍（狀態改變）
      bool currentReading = readSensorWithDebounce();
      if (currentReading != lastStableState)
      {
        Serial.println(F("[閃爍] 偵測到新跳躍，中斷閃爍！"));

        // 恢復當前燈效
        displayCurrentLEDs();
        isBlinking = false;

        // updateJumpState 會處理計數邏輯
        return; // 立即結束閃爍
      }
    }

    // 點亮階段（顯示當前累積的燈數）- 同步到 4 條燈條
    for (int j = 0; j < currentLEDCount; j++)
    {
      CRGB color = getColorForPosition(j, currentLEDCount);
      for (int strip = 0; strip < NUM_LED_STRIPS; strip++)
      {
        leds[strip][j] = color;
      }
    }
    FastLED.show();

    // 分段檢查感測器（避免錯過跳躍）
    for (int t = 0; t < BLINK_INTERVAL; t += 10)
    {
      delay(10);

      // 檢查是否有新的跳躍（狀態改變）
      bool currentReading = readSensorWithDebounce();
      if (currentReading != lastStableState)
      {
        Serial.println(F("[閃爍] 偵測到新跳躍，中斷閃爍！"));

        // 恢復當前燈效
        displayCurrentLEDs();
        isBlinking = false;

        // updateJumpState 會處理計數邏輯
        return; // 立即結束閃爍
      }
    }
  }

  // 閃爍完成後才清空
  clearLEDs();

  // 重置計數和狀態
  jumpCount = 0;
  currentLEDCount = 0;
  isBlinking = false;
  isRainbowMode = false;
  rainbowHue = 0;

  Serial.println(F("[閃爍] 完成,已重置跳躍計數"));
}

// ============================================================================
// 清空 LED
// ============================================================================

void clearLEDs()
{
  FastLED.clear();
  FastLED.show();
}

// ============================================================================
// LED 測試動畫(逐顆點亮測試 - 4 條燈條版本)
// ============================================================================

void testLEDPattern()
{
  Serial.println(F("開始 LED 診斷測試..."));
  Serial.print(F("燈條數量: "));
  Serial.println(NUM_LED_STRIPS);
  Serial.print(F("每條 LED 數量: "));
  Serial.println(NUM_LEDS);
  Serial.print(F("LED 腳位: "));
  for (int strip = 0; strip < NUM_LED_STRIPS; strip++)
  {
    Serial.print(LED_PINS[strip]);
    if (strip < NUM_LED_STRIPS - 1)
      Serial.print(F(", "));
  }
  Serial.println();
  Serial.print(F("亮度設定: "));
  Serial.println(LED_BRIGHTNESS);

  // 測試 5:逐顆點亮掃描(4 條同步)
  Serial.println(F("測試 5: 逐顆掃描 (4 條同步)..."));
  clearLEDs();
  for (int i = 0; i < NUM_LEDS; i++)
  {
    for (int strip = 0; strip < NUM_LED_STRIPS; strip++)
    {
      leds[strip][i] = CRGB(255, 255, 255);
    }
    FastLED.show();
    delay(50);

    // 每 10 顆報告進度
    if ((i + 1) % 10 == 0)
    {
      Serial.print(F("  已掃描到第 "));
      Serial.print(i + 1);
      Serial.println(F(" 顆"));
    }
  }

  delay(1000);

  // 測試 6:三原色全亮測試(4 條同步)
  Serial.println(F("測試 6: 三原色測試 (4 條同步)..."));

  // 紅色
  Serial.println(F("  全部紅色"));
  for (int strip = 0; strip < NUM_LED_STRIPS; strip++)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[strip][i] = CRGB(255, 0, 0);
    }
  }
  FastLED.show();
  delay(1000);

  // 綠色
  Serial.println(F("  全部綠色"));
  for (int strip = 0; strip < NUM_LED_STRIPS; strip++)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[strip][i] = CRGB(0, 255, 0);
    }
  }
  FastLED.show();
  delay(1000);

  // 藍色
  Serial.println(F("  全部藍色"));
  for (int strip = 0; strip < NUM_LED_STRIPS; strip++)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[strip][i] = CRGB(0, 0, 255);
    }
  }
  FastLED.show();
  delay(1000);

  // 清空
  Serial.println(F("\n測試完成！清空 LED..."));
  clearLEDs();
  delay(500);

  Serial.println(F("=========================================="));
  Serial.println(F("診斷結果:"));
  Serial.println(F("如果只看到第一顆燈亮:"));
  Serial.println(F("  1. 檢查 BI 腳位是否接地"));
  Serial.println(F("  2. 檢查資料線是否正確連接"));
  Serial.println(F("  3. 嘗試將色序改為 GRB"));
  Serial.println(F("  4. 檢查電源供應是否足夠"));
  Serial.println(F("==========================================\n"));
}

// ============================================================================
// 顯示彩虹霓虹燈效(完成所有階段後的慶祝動畫)
// ============================================================================

void displayRainbowEffect()
{
  Serial.println(F("[彩虹霓虹] 開始彩虹霓虹燈效!"));

  // 同步模式:4 條燈條顯示相同的彩虹效果
  if (SYNC_MODE)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      // 讓每顆 LED 的色相錯開,形成彩虹漸層
      uint8_t hue = rainbowHue + (i * 256 / NUM_LEDS);
      CRGB color = CHSV(hue, 255, 255);

      // 同步設定到 4 條燈條
      for (int strip = 0; strip < NUM_LED_STRIPS; strip++)
      {
        leds[strip][i] = color;
      }
    }
  }
  else
  {
    // 獨立控制模式(預留給未來擴展)
    // TODO: 實作不同燈條的獨立彩虹效果(例如不同相位偏移)
    for (int strip = 0; strip < NUM_LED_STRIPS; strip++)
    {
      for (int i = 0; i < NUM_LEDS; i++)
      {
        uint8_t hue = rainbowHue + (i * 256 / NUM_LEDS);
        leds[strip][i] = CHSV(hue, 255, 255);
      }
    }
  }

  FastLED.show();
  lastRainbowUpdate = millis();
}

// ============================================================================
// 更新彩虹霓虹動畫(持續循環色相變化)
// ============================================================================

void updateRainbowEffect()
{
  // 檢查是否到達更新間隔
  if (millis() - lastRainbowUpdate >= RAINBOW_UPDATE_INTERVAL)
  {
    // 更新色相值(0-255 循環)
    rainbowHue += RAINBOW_HUE_STEP;

    // 同步模式:4 條燈條顯示相同的動畫
    if (SYNC_MODE)
    {
      for (int i = 0; i < NUM_LEDS; i++)
      {
        // 讓每顆 LED 的色相錯開,形成流動的彩虹效果
        uint8_t hue = rainbowHue + (i * 256 / NUM_LEDS);
        CRGB color = CHSV(hue, 255, 255);

        // 同步設定到 4 條燈條
        for (int strip = 0; strip < NUM_LED_STRIPS; strip++)
        {
          leds[strip][i] = color;
        }
      }
    }
    else
    {
      // 獨立控制模式(預留給未來擴展)
      for (int strip = 0; strip < NUM_LED_STRIPS; strip++)
      {
        for (int i = 0; i < NUM_LEDS; i++)
        {
          uint8_t hue = rainbowHue + (i * 256 / NUM_LEDS);
          leds[strip][i] = CHSV(hue, 255, 255);
        }
      }
    }

    FastLED.show();
    lastRainbowUpdate = millis();
  }

  // 在彩虹模式下也檢查逾時,讓使用者可以重新開始
  if (isRainbowMode && (millis() - lastJumpTime) > IDLE_TIMEOUT)
  {
    Serial.println(F("[彩虹霓虹] 超過閒置時間,退出彩虹模式..."));

    // 清空 LED
    clearLEDs();

    // 重置所有狀態
    jumpCount = 0;
    currentLEDCount = 0;
    isRainbowMode = false;
    rainbowHue = 0;

    Serial.println(F("[重置] 已退出彩虹模式,可重新開始跳躍"));
  }
}