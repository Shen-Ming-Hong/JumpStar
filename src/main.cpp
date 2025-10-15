#include <Arduino.h>
#include <FastLED.h>

// ============================================================================
// 硬體常數定義
// ============================================================================

// ========== 感測器選擇 ==========
// 設定為 true 使用超音波，設定為 false 使用紅外線
#define USE_ULTRASONIC true

// ========== 紅外線感測器設定 ==========
const int IR_SENSOR_PIN = 2; // 紅外線感測器腳位（D2）

// ========== 超音波感測器設定 ==========
const int TRIG_PIN = 3;            // 超音波 Trig 腳位（D3）
const int ECHO_PIN = 4;            // 超音波 Echo 腳位（D4）
const int DETECTION_DISTANCE = 30; // 偵測距離閘值（公分）

// ========== WS2813 LED 燈條設定 ==========
const int LED_PIN = 6;         // LED 資料腳位
const int NUM_LEDS = 300;      // LED 燈珠數量（300 顆）
const int LED_BRIGHTNESS = 50; // 亮度限制（0-255，建議不超過 50% 避免電流過載）

// ⚠️ WS2813 實際接線配置（已測試可運作）：
//
// WS2813 燈條有 4 條訊號線（輸入端）：
//   - DI (Data Input)   → Arduino Pin 6（主要資料輸入）
//   - BI (Backup Input) → 空接（本專案測試可正常運作）
//   - GND              → Arduino GND + 電源 GND（必須共地！）
//   - 5V               → 外部電源 5V（不要用 Arduino 供電！）
//
// 🔧 目前使用的接線方式：
//    Arduino Pin 6 ------------ DI
//    BI ------------------------ 空接（不連接）
//    Arduino GND -------------- GND (燈條)
//    外部電源 5V --------------- 5V (燈條)
//    外部電源 GND -------------- GND (與 Arduino GND 共地)
//
// 💡 WS2813 雙訊號說明：
//    - BI 是備援訊號輸入，當 DI 損壞時可自動切換到 BI
//    - 本專案測試結果：BI 空接也能正常工作
//    - 如需使用備援功能：將 BI 連接到 GND 或與 DI 並聯
//
// ⚡ 電源注意事項：
//    - 300 顆 LED 最大電流：300 × 60mA = 18A
//    - 實際使用（50% 亮度）：約 6A
//    - 建議使用 5V 10A 以上電源供應器
//    - 大量 LED 建議每 50-100 顆注入一次電源

// 序列埠速率
const long SERIAL_BAUD = 9600;

// 去彈跳時間閾值（毫秒）
const unsigned long DEBOUNCE_DELAY = 50;

// 跳躍計數參數
const int MAX_JUMP_COUNT = 20;            // 滿燈所需的跳躍次數（根據國小生體能設定）
const unsigned long IDLE_TIMEOUT = 5000;  // 無跳躍逾時時間（5 秒）
const int BLINK_COUNT = 5;                // 熄滅前閃爍次數
const unsigned long BLINK_INTERVAL = 300; // 閃爍間隔時間（毫秒）

// ============================================================================
// 全域變數
// ============================================================================

// LED 陣列
CRGB leds[NUM_LEDS];

// 跳躍狀態列舉
enum JumpState
{
  GROUNDED, // 在地面
  IN_AIR,   // 空中
  LANDING   // 著地（過渡狀態）
};

// 當前狀態
JumpState currentState = GROUNDED;

// 感測器狀態（通用）
bool sensorState = HIGH;            // 當前感測器讀值
bool lastSensorState = HIGH;        // 上次感測器讀值
unsigned long lastDebounceTime = 0; // 上次去彈跳時間戳

// 超音波相關變數
long ultrasonicDistance = 0; // 當前測得距離（cm）

// 跳躍計數相關
unsigned long jumpStartTime = 0; // 跳躍開始時間
unsigned long lastJumpTime = 0;  // 上次跳躍時間
int jumpCount = 0;               // 累積跳躍次數
int currentLEDCount = 0;         // 目前點亮的 LED 數量
bool isBlinking = false;         // 是否正在閃爍中

// ============================================================================
// 函式宣告
// ============================================================================

// 感測器讀取函式
bool readSensorWithDebounce(); // 讀取感測器（含去彈跳）
long readUltrasonicDistance(); // 讀取超音波距離（cm）
bool readIRSensor();           // 讀取紅外線感測器
bool readUltrasonicSensor();   // 讀取超音波感測器

// 狀態機和燈效
void updateJumpState();                            // 更新跳躍狀態機
void checkIdleTimeout();                           // 檢查逾時並執行閃爍
void displayCurrentLEDs();                         // 顯示當前累積的 LED
void blinkAndClear();                              // 閃爍後清空
CRGB getColorForPosition(int position, int total); // 根據位置取得漸變顏色
void clearLEDs();                                  // 清空 LED
void testLEDPattern();                             // LED 測試動畫

// ============================================================================
// 初始化設定
// ============================================================================

void setup()
{
  // 初始化序列埠（用於除錯）
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("=== 跳跳星系統啟動 ==="));
  Serial.println(F("系統初始化中..."));

#if USE_ULTRASONIC
  // 超音波模式
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println(F("✅ 使用超音波感測器模式"));
  Serial.print(F("✅ 偵測距離閘值: "));
  Serial.print(DETECTION_DISTANCE);
  Serial.println(F(" cm"));
#else
  // 紅外線模式
  pinMode(IR_SENSOR_PIN, INPUT);
  Serial.println(F("✅ 使用紅外線感測器模式"));
#endif

  // 初始化 LED 燈條
  // 注意：某些 WS2813 使用 RGB 色序，某些使用 GRB，請根據實際情況調整
  FastLED.addLeds<WS2813, LED_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  Serial.println(F("✓ LED 燈條初始化完成"));

  // 執行 LED 測試動畫
  Serial.println(F("執行 LED 測試動畫..."));
  testLEDPattern();

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

#if USE_ULTRASONIC
    // 超音波模式額外顯示距離
    Serial.print(F(" | 距離: "));
    Serial.print(ultrasonicDistance);
    Serial.print(F(" cm"));
#endif

    Serial.println();

    lastPrintTime = millis();
    lastPrintedState = sensorState;
  }

  // 更新跳躍狀態機
  updateJumpState();

  // 檢查逾時（非閃爍狀態下才檢查）
  if (!isBlinking)
  {
    checkIdleTimeout();
  }

  // 小延遲以避免過度頻繁的感測器讀取
  delay(10);
}

// ============================================================================
// 感測器讀取（通用介面，含去彈跳）
// ============================================================================

bool readSensorWithDebounce()
{
  // 根據配置選擇對應的感測器
  bool reading;

#if USE_ULTRASONIC
  reading = readUltrasonicSensor();
#else
  reading = readIRSensor();
#endif

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
// 紅外線感測器讀取
// ============================================================================

bool readIRSensor()
{
  // 直接讀取紅外線數位訊號
  // LOW = 有人在感應範圍（在地面）
  // HIGH = 無人（跳躍中）
  return digitalRead(IR_SENSOR_PIN);
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
// 跳躍狀態機更新
// ============================================================================

void updateJumpState()
{
  // 感測器邏輯：LOW = 有人在感應範圍（在地面），HIGH = 無人（跳躍中）
  // 持續 LOW = 玩家在地面，尚未跳躍
  // 變成 HIGH = 玩家離地，開始跳躍
  // 重新變回 LOW = 玩家落地

  switch (currentState)
  {
  case GROUNDED:
    // 如果感測器變成 HIGH（無人），代表玩家離地
    if (sensorState == HIGH)
    {
      jumpStartTime = millis();
      currentState = IN_AIR;
      Serial.println(F("[狀態] 玩家離地！"));
    }
    break;

  case IN_AIR:
    // 如果感測器變回 LOW（有人），代表玩家著地
    if (sensorState == LOW)
    {
      unsigned long airTime = millis() - jumpStartTime;
      currentState = LANDING;

      Serial.print(F("[狀態] 玩家著地！離地時間: "));
      Serial.print(airTime);
      Serial.println(F(" ms"));

      // 增加跳躍計數
      jumpCount++;
      lastJumpTime = millis();

      // 計算當前應點亮的 LED 數量（按比例分配）
      // 公式：LED數量 = (跳躍次數 / 滿燈所需次數) × 燈條總數
      // 範例：30顆燈條: 20次跳躍滿燈，每跳一次亮 1.5 顆
      //       60顆燈條: 20次跳躍滿燈，每跳一次亮 3 顆
      currentLEDCount = min((jumpCount * NUM_LEDS) / MAX_JUMP_COUNT, NUM_LEDS);

      Serial.print(F("[計數] 累積跳躍次數: "));
      Serial.print(jumpCount);
      Serial.print(F(" / "));
      Serial.print(MAX_JUMP_COUNT);
      Serial.print(F(" | LED 數量: "));
      Serial.print(currentLEDCount);
      Serial.print(F(" / "));
      Serial.println(NUM_LEDS);

      // 顯示當前累積的燈效
      displayCurrentLEDs();
    }
    break;

  case LANDING:
    // 著地狀態是過渡狀態，立即轉回地面狀態
    currentState = GROUNDED;
    break;
  }
}

// ============================================================================
// 檢查逾時並執行閃爍清空
// ============================================================================

void checkIdleTimeout()
{
  // 如果有累積跳躍且超過逾時時間
  if (currentLEDCount > 0 && (millis() - lastJumpTime) > IDLE_TIMEOUT)
  {
    Serial.println(F("[逾時] 超過閒置時間，準備閃爍後熄滅..."));
    blinkAndClear();
  }
}

// ============================================================================
// 根據 LED 位置取得顏色（動態漸變）
// ============================================================================

CRGB getColorForPosition(int position, int totalLEDs)
{
  // 動態漸變：綠色 → 黃色 → 紅色
  // 使用 HSV 色彩空間進行平滑過渡
  // 色相 (Hue): 96 (綠) → 64 (黃) → 0 (紅)

  // 計算當前位置的比例 (0.0 到 1.0)
  float ratio = (float)position / (float)(totalLEDs - 1);

  // 色相範圍：96 (綠色) 到 0 (紅色)
  uint8_t hue = 96 - (uint8_t)(ratio * 96);

  return CHSV(hue, 255, 255); // 全飽和度、全亮度
}

// ============================================================================
// 顯示當前累積的 LED
// ============================================================================

void displayCurrentLEDs()
{
  clearLEDs();

  // 輸出燈效資訊
  Serial.print(F("[燈效] 點亮 LED 數量: "));
  Serial.print(currentLEDCount);
  Serial.print(F(" / "));
  Serial.println(NUM_LEDS);

  // 動態設定每顆 LED 的顏色（漸變效果）
  for (int i = 0; i < currentLEDCount; i++)
  {
    leds[i] = getColorForPosition(i, currentLEDCount);
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

      // 檢查是否有新的跳躍（玩家離地）
      bool currentReading = readSensorWithDebounce();
      if (currentReading == HIGH && sensorState == HIGH && currentState == GROUNDED)
      {
        Serial.println(F("[閃爍] 偵測到新跳躍，中斷閃爍！"));

        // 恢復當前燈效
        displayCurrentLEDs();

        // 啟動跳躍狀態
        jumpStartTime = millis();
        currentState = IN_AIR;
        isBlinking = false;

        Serial.println(F("[狀態] 玩家離地！"));
        return; // 立即結束閃爍
      }
    }

    // 點亮階段（顯示當前累積的燈數）
    for (int j = 0; j < currentLEDCount; j++)
    {
      leds[j] = getColorForPosition(j, currentLEDCount);
    }
    FastLED.show();

    // 分段檢查感測器（避免錯過跳躍）
    for (int t = 0; t < BLINK_INTERVAL; t += 10)
    {
      delay(10);

      // 檢查是否有新的跳躍（玩家離地）
      bool currentReading = readSensorWithDebounce();
      if (currentReading == HIGH && sensorState == HIGH && currentState == GROUNDED)
      {
        Serial.println(F("[閃爍] 偵測到新跳躍，中斷閃爍！"));

        // 恢復當前燈效
        displayCurrentLEDs();

        // 啟動跳躍狀態
        jumpStartTime = millis();
        currentState = IN_AIR;
        isBlinking = false;

        Serial.println(F("[狀態] 玩家離地！"));
        return; // 立即結束閃爍
      }
    }
  }

  // 閃爍完成後才清空
  clearLEDs();

  // 重置計數
  jumpCount = 0;
  currentLEDCount = 0;
  isBlinking = false;

  Serial.println(F("[閃爍] 完成，已重置跳躍計數"));
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
// LED 測試動畫（逐顆點亮測試）
// ============================================================================

void testLEDPattern()
{
  Serial.println(F("開始 LED 診斷測試..."));
  Serial.print(F("LED 數量設定: "));
  Serial.println(NUM_LEDS);
  Serial.print(F("LED 腳位: "));
  Serial.println(LED_PIN);
  Serial.print(F("亮度設定: "));
  Serial.println(LED_BRIGHTNESS);

  // 測試 1：點亮第一顆（紅色）
  Serial.println(F("\n測試 1: 第一顆 LED (紅色)"));
  leds[0] = CRGB(255, 0, 0);
  FastLED.show();
  delay(1000);

  // 測試 2：點亮前 3 顆（綠色）
  Serial.println(F("測試 2: 前 3 顆 LED (綠色)"));
  leds[0] = CRGB(0, 255, 0);
  leds[1] = CRGB(0, 255, 0);
  leds[2] = CRGB(0, 255, 0);
  FastLED.show();
  delay(1000);

  // 測試 3：點亮前 10 顆（藍色）
  Serial.println(F("測試 3: 前 10 顆 LED (藍色)"));
  for (int i = 0; i < 10; i++)
  {
    leds[i] = CRGB(0, 0, 255);
  }
  FastLED.show();
  delay(1000);

  // 測試 4：全部點亮（白色）
  Serial.println(F("測試 4: 全部點亮 (白色)"));
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(255, 255, 255);
  }
  FastLED.show();
  delay(2000);

  // 測試 5：逐顆點亮掃描
  Serial.println(F("測試 5: 逐顆掃描..."));
  clearLEDs();
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(255, 255, 255);
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

  // 測試 6：三原色全亮測試
  Serial.println(F("測試 6: 三原色測試..."));

  // 紅色
  Serial.println(F("  全部紅色"));
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(255, 0, 0);
  }
  FastLED.show();
  delay(1000);

  // 綠色
  Serial.println(F("  全部綠色"));
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 255, 0);
  }
  FastLED.show();
  delay(1000);

  // 藍色
  Serial.println(F("  全部藍色"));
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 255);
  }
  FastLED.show();
  delay(1000);

  // 清空
  Serial.println(F("\n測試完成！清空 LED..."));
  clearLEDs();
  delay(500);

  Serial.println(F("=========================================="));
  Serial.println(F("診斷結果："));
  Serial.println(F("如果只看到第一顆燈亮："));
  Serial.println(F("  1. 檢查 BI 腳位是否接地"));
  Serial.println(F("  2. 檢查資料線是否正確連接"));
  Serial.println(F("  3. 嘗試將色序改為 GRB"));
  Serial.println(F("  4. 檢查電源供應是否足夠"));
  Serial.println(F("==========================================\n"));
}