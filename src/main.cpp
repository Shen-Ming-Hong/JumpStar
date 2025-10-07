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

// ========== WS2812B LED 燈條設定 ==========
const int LED_PIN = 6;         // LED 資料腳位
const int NUM_LEDS = 30;       // LED 燈珠數量（30 顆）
const int LED_BRIGHTNESS = 50; // 亮度限制（0-255，建議不超過 50% 避免電流過載）

// 序列埠速率
const long SERIAL_BAUD = 9600;

// 去彈跳時間閾值（毫秒）
const unsigned long DEBOUNCE_DELAY = 50;

// 跳躍高度計算參數
const unsigned long MAX_AIR_TIME = 900;      // 最大滯空時間ms（滿燈）
const unsigned long DISPLAY_DURATION = 3000; // 燈效顯示時間（3 秒）

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

// 計時相關
unsigned long jumpStartTime = 0; // 跳躍開始時間
unsigned long airTime = 0;       // 離地時間（毫秒）

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
int calculateLEDCount(unsigned long airTime);      // 計算應點亮的 LED 數量
void displayJumpEffect(unsigned long airTime);     // 顯示跳躍燈效
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
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
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
      airTime = millis() - jumpStartTime;
      currentState = LANDING;

      Serial.print(F("[狀態] 玩家著地！離地時間: "));
      Serial.print(airTime);
      Serial.println(F(" ms"));

      // 顯示跳躍燈效
      displayJumpEffect(airTime);
    }
    break;

  case LANDING:
    // 著地狀態是過渡狀態，立即轉回地面狀態
    currentState = GROUNDED;
    break;
  }
}

// ============================================================================
// 計算應點亮的 LED 數量（線性映射）
// ============================================================================

int calculateLEDCount(unsigned long airTime)
{
  // 滯空 3 秒（3000ms）= 30 顆 LED 全亮
  // 線性比例：LED 數量 = (滯空時間 / 最大時間) × 總 LED 數

  if (airTime >= MAX_AIR_TIME)
  {
    return NUM_LEDS; // 超過 3 秒就顯示滿燈
  }

  // 計算比例，至少點亮 1 顆 LED
  int ledCount = (airTime * NUM_LEDS) / MAX_AIR_TIME;
  return max(1, ledCount); // 確保至少有 1 顆 LED
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
// 顯示跳躍燈效
// ============================================================================

void displayJumpEffect(unsigned long airTime)
{
  clearLEDs();

  // 計算應點亮的 LED 數量
  int ledCount = calculateLEDCount(airTime);

  // 輸出燈效資訊
  Serial.print(F("[燈效] 點亮 LED 數量: "));
  Serial.print(ledCount);
  Serial.print(F(" / "));
  Serial.print(NUM_LEDS);
  Serial.print(F(" (滯空時間: "));
  Serial.print(airTime);
  Serial.println(F(" ms)"));

  // 動態設定每顆 LED 的顏色（漸變效果）
  for (int i = 0; i < ledCount; i++)
  {
    leds[i] = getColorForPosition(i, ledCount);
  }

  FastLED.show();

  // 顯示 3 秒後清空
  delay(DISPLAY_DURATION);
  clearLEDs();

  Serial.println(F("[燈效] 清空，等待下一次跳躍..."));
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
// LED 測試動畫（開機彩虹效果）
// ============================================================================

void testLEDPattern()
{
  // 彩虹漸變效果
  for (int hue = 0; hue < 255; hue += 5)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CHSV(hue + (i * 10), 255, 255);
    }
    FastLED.show();
    delay(20);
  }

  // 清空
  clearLEDs();
  delay(500);
}