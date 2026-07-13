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

TelemetryPacket telemetry = {0}; // Initialize with 0

uint8_t receiverMAC[] = {0xcc, 0x50, 0xe3, 0x5c, 0x43, 0x48};

void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
  //Serial.print("Send Status: ");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}
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
  
  // Header / Title
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("RC TRANSMITTER");
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);

  // Battery Status
  display.setCursor(0, 15);
  display.setTextSize(1);

  bool telemetryOk = (millis() - lastTelemetryTime) < 1000;
  display.print("Batt: ");

  if (!telemetryOk)
  {
      display.println("N/A");
  }
  else
  {
      display.print(telemetry.batteryMv / 1000.0, 2);
      display.println(" V");
  }

  // Link Quality (Signal Strength)
  display.setCursor(0, 28);
  display.print("LQ: ");

  if (!telemetryOk)
  {
      display.println("LOST");
  }
  else
  {
      display.print(telemetry.linkQuality);
      display.println("%");
  }

  // System Status Info
  display.setCursor(0, 42);
  display.print("State: ");
  display.println(aux1State ? "ARMED" : "DISARMED");
  
  display.setCursor(0, 54);
  display.print("Mode: ");
  display.println(aux2State ? "ACRO" : "ANGLE");

  display.display();
}

void setup()
{
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

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  pinMode(AUX1_BUTTON, INPUT_PULLUP);
  pinMode(AUX2_BUTTON, INPUT_PULLUP);

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
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

void loop()
{
  updateButtons();
  
  int rollADC   = analogRead(ROLL_PIN);
  int pitchADC  = analogRead(PITCH_PIN);
  int throttleADC = analogRead(THROTTLE_PIN);
  int yawADC    = analogRead(YAW_PIN);

  packet.roll   = map(rollADC, 0, 4095, 2000, 1000);
  packet.pitch  = map(pitchADC, 0, 4095, 2000, 1000);
  packet.throttle = map(throttleADC, 0, 4095, 2000, 1000);
  packet.yaw    = map(yawADC, 0, 4095, 2000, 1000);

  packet.aux3 = 1000;
  packet.aux4 = 1000;
  packet.counter++;

  esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));

  // Periodically refresh OLED screen without lag
  if (millis() - lastDisplayUpdate >= displayInterval)
  {
      lastDisplayUpdate = millis();
      updateDisplay();
  }

  delay(20);   // ~50 Hz
}