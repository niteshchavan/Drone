#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "EspNowRcLink/Transmitter.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Analog joystick pins (stick 1 = left, stick 2 = right)
const int JOY1_X_PIN = 36; // throttle (left stick up/down)
const int JOY1_Y_PIN = 39; // yaw    (left stick left/right) 
const int JOY2_X_PIN = 35; // roll   (right stick left/right)
const int JOY2_Y_PIN = 34; // pitch  (right stick up/down)
const int AUX1_PIN = 26; // toggle switch -> channel 5 (Arm)
const int AUX2_PIN = 27; // toggle switch -> channel 6 (Flight Mode)


// ADC and RC signal ranges
const int ANALOG_MIN = 0;
const int ANALOG_MAX = 4095;
const int RC_MIN = 1000;
const int RC_MAX = 2000;
const int RC_CENTER = 1500;
const int RC_HALF_SPAN = (RC_MAX - RC_MIN) / 2; // 500 us

// Tuning parameters
const uint32_t SEND_INTERVAL_MS = 20;      // 50 Hz update for smoother feel
const float FILTER_ALPHA = 0.15f;          // 0 < alpha <= 1, lower = smoother 25 tha
const float DEADZONE_FRACTION = 0.05f;     // ±5% stick deadzone around center
const int SWITCH_DEBOUNCE_MS = 120;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

class EmaFilter {
public:
  explicit EmaFilter(float a = FILTER_ALPHA) : alpha(a), initialized(false), value(0.0f) {}
  float update(float sample) {
    if (!initialized) {
      value = sample;
      initialized = true;
    } else {
      value = alpha * sample + (1.0f - alpha) * value;
    }
    return value;
  }
private:
  float alpha;
  bool initialized;
  float value;
};

EspNowRcLink::Transmitter tx;

struct StickCenters {
  float yaw;
  float roll;
  float pitch;
};

struct StickTrims {
  int yaw;
  int roll;
  int pitch;
};

static StickCenters centers = {
  ANALOG_MIN + ((ANALOG_MAX - ANALOG_MIN) * 0.5f),
  ANALOG_MIN + ((ANALOG_MAX - ANALOG_MIN) * 0.5f),
  ANALOG_MIN + ((ANALOG_MAX - ANALOG_MIN) * 0.5f)
};

static StickTrims trims = {
  0,   // yaw trim (µs)
  0,   // roll trim (µs)
  0    // pitch trim (µs)
};

static float measureStickCenter(int pin, size_t samples = 32) {
  uint32_t total = 0;
  for (size_t i = 0; i < samples; ++i) {
    total += analogRead(pin);
    delay(2);
  }
  float center = static_cast<float>(total) / static_cast<float>(samples);
  return constrain(center, static_cast<float>(ANALOG_MIN), static_cast<float>(ANALOG_MAX));
}

static float readFiltered(int pin, EmaFilter &filter) {
  int raw = analogRead(pin);
  raw = constrain(raw, ANALOG_MIN, ANALOG_MAX);
  return filter.update(static_cast<float>(raw));
}

static int mapCenteredAxis(float rawSample, float center, bool invert = false, int trimUs = 0) {
  float offset = rawSample - center;
  float span = (offset >= 0.0f)
                 ? (static_cast<float>(ANALOG_MAX) - center)
                 : (center - static_cast<float>(ANALOG_MIN));
  if (span < 1.0f) span = 1.0f;
  float centered = constrain(offset / span, -1.0f, 1.0f);
  if (invert) centered = -centered;

  if (fabsf(centered) < DEADZONE_FRACTION) {
    centered = 0.0f;
  } else {
    float sign = (centered < 0.0f) ? -1.0f : 1.0f;
    float magnitude = (fabsf(centered) - DEADZONE_FRACTION) / (1.0f - DEADZONE_FRACTION);
    centered = sign * constrain(magnitude, 0.0f, 1.0f);
  }

  // 40% linear + 60% exponential
  float expo = 0.6f;
  float curved = (1.0f - expo) * centered + expo * centered * centered * centered;

  int rcValue = RC_CENTER + static_cast<int>(curved * RC_HALF_SPAN) + trimUs;

  //int rcValue = RC_CENTER + static_cast<int>(centered * RC_HALF_SPAN) + trimUs;
  return constrain(rcValue, RC_MIN, RC_MAX);
}

static int mapThrottleAxis(float rawSample, bool invert = false) {
  float span = static_cast<float>(ANALOG_MAX - ANALOG_MIN);
  float normalized = (rawSample - ANALOG_MIN) / span;      // 0.0 .. 1.0
  normalized = constrain(normalized, 0.0f, 1.0f);
  if (invert) normalized = 1.0f - normalized;
  int rcValue = RC_MIN + static_cast<int>(normalized * (RC_MAX - RC_MIN));
  return constrain(rcValue, RC_MIN, RC_MAX);
}

// Updated readSwitchChannel to accept a dynamic pin input
static int readSwitchChannel(int pin) {
  // Track independent states for our different switches using static lists
  static int channelValues[2] = {RC_MIN, RC_MIN};
  static bool lastPressed[2] = {false, false};
  static uint32_t lastToggleMs[2] = {0, 0};

  // Select memory slot: AUX1 (idx 0) or AUX2 (idx 1)
  int idx = (pin == AUX2_PIN) ? 1 : 0;

  bool pressed = (digitalRead(pin) == LOW);
  uint32_t now = millis();

  if (pressed && !lastPressed[idx] && (now - lastToggleMs[idx]) > static_cast<uint32_t>(SWITCH_DEBOUNCE_MS)) {
    channelValues[idx] = (channelValues[idx] == RC_MIN) ? RC_MAX : RC_MIN;
    lastToggleMs[idx] = now;
  }

  lastPressed[idx] = pressed;
  return channelValues[idx];
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
    while (1);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 25);
  display.println("Connecting...");
  display.display();

  pinMode(AUX1_PIN, INPUT_PULLUP);
  pinMode(AUX2_PIN, INPUT_PULLUP);

  tx.begin(true);
#if defined(ESP32) || defined(ESP_PLATFORM)
  analogReadResolution(12);
#endif
  centers.yaw = measureStickCenter(JOY1_Y_PIN);
  centers.roll = measureStickCenter(JOY2_X_PIN);
  centers.pitch = measureStickCenter(JOY2_Y_PIN);
}

void display_update(int channels[])
{
    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Left column
    display.setCursor(0, 0);
    display.print("Roll:");
    display.print(channels[0]);

    display.setCursor(0, 16);
    display.print("Pitch:");
    display.print(channels[1]);

    display.setCursor(68, 16); //0, 32
    display.print("Thr:");
    display.print(channels[2]);

    // Right column
    display.setCursor(68, 0);
    display.print("Yaw:");
    display.print(channels[3]);

    display.setCursor(0, 32); //68, 16
    display.print("ARM:");
    display.print(channels[4] == 2000 ? "ON" : "OFF");

    display.setCursor(68, 32);
    display.print("MODE:");
    display.print(channels[5] == 2000 ? "ACRO" : "ANGLE");

    display.display();
}


void loop() {
  static uint32_t lastSendMs = 0;
  static uint32_t lastDisplayMs = 0;
  static EmaFilter yawFilter;
  static EmaFilter throttleFilter;
  static EmaFilter rollFilter;
  static EmaFilter pitchFilter;

  uint32_t now = millis();
  if (now - lastSendMs < SEND_INTERVAL_MS) {
    tx.update();
    return;
  }
  lastSendMs = now;

  float yawSample = readFiltered(JOY1_Y_PIN, yawFilter);
  float throttleSample = readFiltered(JOY1_X_PIN, throttleFilter);
  float rollSample = readFiltered(JOY2_X_PIN, rollFilter);
  float pitchSample = readFiltered(JOY2_Y_PIN, pitchFilter);

  int channels[6];
  channels[0] = mapCenteredAxis(rollSample, centers.roll, true, trims.roll);
  channels[1] = mapCenteredAxis(pitchSample, centers.pitch, true, trims.pitch);
  channels[2] = mapThrottleAxis(throttleSample, true);
  channels[3] = mapCenteredAxis(yawSample, centers.yaw, true, trims.yaw);
  channels[4] = readSwitchChannel(AUX1_PIN);   // ARM
  channels[5] = readSwitchChannel(AUX2_PIN);   // FLIGHT MODE

  for (uint8_t i = 0; i < 6; ++i) {
    tx.setChannel(i, channels[i]);
  }
  tx.commit();

  Serial.print("RC:");
  for (uint8_t i = 0; i < 6; ++i) {
    Serial.print(' ');
    Serial.print(channels[i]);
  }
  Serial.println();

  if (millis() - lastDisplayMs >= 100) {   // Update OLED every 100 ms (10 Hz)
    lastDisplayMs = millis();
    display_update(channels);
  }

  tx.update();
}

