#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define ROLL_PIN   35
#define PITCH_PIN  34
#define THROTTLE_PIN 36
#define YAW_PIN 39
#define AUX1_BUTTON 26   // Arm
#define AUX2_BUTTON 27   // Flight Mode
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

bool aux1State = false;   // false = Disarmed
bool aux2State = false;   // false = Angle, true = Acro

bool lastAux1 = HIGH;
bool lastAux2 = HIGH;

unsigned long lastDebounce1 = 0;
unsigned long lastDebounce2 = 0;
const unsigned long debounceTime = 50;

unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 100; // Refresh screen every 100ms

uint32_t lastTelemetryTime = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

typedef struct {
  uint16_t roll;
  uint16_t pitch;
  uint16_t throttle;
  uint16_t yaw;
  uint16_t aux1;
  uint16_t aux2;
  uint16_t aux3;
  uint16_t aux4;
  uint32_t counter;
} ControlPacket;

ControlPacket packet;

typedef struct {
    uint16_t batteryMv;
    uint8_t linkQuality;
} TelemetryPacket;

TelemetryPacket telemetry = {0}; 

uint8_t receiverMAC[] = {0xcc, 0x50, 0xe3, 0x5c, 0x43, 0x48};

void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len)
{
    if (len == sizeof(TelemetryPacket))
    {
        lastTelemetryTime = millis();
        memcpy(&telemetry, incomingData, sizeof(telemetry));
    }
}

void updateDisplay()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("RC TRANSMITTER");
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);

  display.setCursor(0, 15);
  bool telemetryOk = (millis() - lastTelemetryTime) < 1000;
  display.print("Batt: ");
  if (!telemetryOk) display.println("N/A");
  else {
      display.print(telemetry.batteryMv / 1000.0, 2);
      display.println(" V");
  }

  display.setCursor(0, 28);
  display.print("LQ: ");
  if (!telemetryOk) display.println("LOST");
  else {
      display.print(telemetry.linkQuality);
      display.println("%");
  }

  display.setCursor(0, 42);
  display.print("State: ");
  display.println(aux1State ? "ARMED" : "DISARMED");
  
  display.setCursor(0, 54);
  display.print("Mode: ");
  display.println(aux2State ? "ACRO" : "ANGLE");

  display.display();
}

uint16_t readADC(uint8_t pin)
{
    analogRead(pin); 
    delayMicroseconds(10);
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++)
    {
        sum += analogRead(pin);
        delayMicroseconds(5);
    }
    return sum >> 3; 
}

void setup()
{
  Serial.begin(115200);
  analogSetWidth(12);

  analogSetPinAttenuation(ROLL_PIN, ADC_11db);
  analogSetPinAttenuation(PITCH_PIN, ADC_11db);
  analogSetPinAttenuation(THROTTLE_PIN, ADC_11db);
  analogSetPinAttenuation(YAW_PIN, ADC_11db);

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

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) return;

  pinMode(AUX1_BUTTON, INPUT_PULLUP);
  pinMode(AUX2_BUTTON, INPUT_PULLUP);

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);
}

void updateButtons()
{
    bool b1 = digitalRead(AUX1_BUTTON);
    bool b2 = digitalRead(AUX2_BUTTON);

    if (lastAux1 == HIGH && b1 == LOW)
    {
        if (millis() - lastDebounce1 > debounceTime)
        {
            aux1State = !aux1State;
            lastDebounce1 = millis();
        }
    }

    if (lastAux2 == HIGH && b2 == LOW)
    {
        if (millis() - lastDebounce2 > debounceTime)
        {
            aux2State = !aux2State;
            lastDebounce2 = millis();
        }
    }

    lastAux1 = b1;
    lastAux2 = b2;

    packet.aux1 = aux1State ? 2000 : 1000;
    packet.aux2 = aux2State ? 2000 : 1000;
}

// Helper function to apply an Expo curve right inside the Transmitter
int applyTransmitterExpo(int inputVal) {
  // Translate standard 1000-2000 range to a temporary -500 to +500 workspace
  float x = (inputVal - 1500) / 500.0;
  
  // Cubic curve calculation (0.5 linear + 0.5 exponential curve blend)
  // Higher values on the right multiplier create softer centers.
  float y = (0.3 * x) + (0.7 * x * x * x);
  
  int outputVal = 1500 + (y * 500.0);
  return constrain(outputVal, 1000, 2000);
}

void loop()
{
  updateButtons();
  
  int rollADC      = readADC(ROLL_PIN);
  int pitchADC     = readADC(PITCH_PIN);
  int throttleADC  = readADC(THROTTLE_PIN);
  int yawADC       = readADC(YAW_PIN);

  // 1. Inverted mapping with physical calibration offsets
  int cleanRoll     = map(rollADC,     0, 4095, 2000, 1000) - 63; 
  int cleanPitch    = map(pitchADC,    0, 4095, 2000, 1000) - 68; 
  int cleanThrottle = map(throttleADC, 0, 4095, 2000, 1000); 
  int cleanYaw      = map(yawADC,      0, 4095, 2000, 1000) - 45;

  // 2. Hardware Deadband to lock center position tight at 1500
  if (abs(cleanRoll - 1500)  < 12) cleanRoll  = 1500;
  if (abs(cleanPitch - 1500) < 12) cleanPitch = 1500;
  if (abs(cleanYaw - 1500)   < 12) cleanYaw   = 1500;

  // 3. Apply the internal Expo curve to reduce stick sensitivity
  packet.roll     = applyTransmitterExpo(cleanRoll);
  packet.pitch    = applyTransmitterExpo(cleanPitch);
  packet.yaw      = applyTransmitterExpo(cleanYaw);
  packet.throttle = constrain(cleanThrottle, 1000, 2000); // Throttle stays normal
    // Snap throttle to exactly 1000 when near minimum
    if (packet.throttle < 1050) {
        packet.throttle = 1000;
    }
  Serial.printf("R:%d P:%d T:%d Y:%d\n", 
                packet.roll, 
                packet.pitch, 
                packet.throttle, 
                packet.yaw);
                
  packet.aux3 = 1000;
  packet.aux4 = 1000;
  packet.counter++;

  esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));

  if (millis() - lastDisplayUpdate >= displayInterval)
  {
      lastDisplayUpdate = millis();
      updateDisplay();
  }

  delay(20); 
}