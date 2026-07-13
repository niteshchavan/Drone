#include <Arduino.h>
#include <ESP8266WiFi.h>
#define BATTERY_PIN A0



extern "C" {
#include <espnow.h>
}

HardwareSerial &ibus = Serial1;

uint16_t ch[14];
uint32_t lastPacketTime = 0;

const float ADC_REF = 3.3;      // NodeMCU A0 range
const float R1 = 30000.0;
const float R2 = 7500.0;

float batteryVoltage = 0.0;
uint8_t currentLinkQuality = 0;

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

TelemetryPacket telemetry;


uint8_t transmitterMAC[] = {0x8c, 0x94, 0xdf, 0x6d, 0x67, 0xc4}; //8c:94:df:6d:67:c4 ESP-32 TX


float readBatteryVoltage()
{
    long total = 0;

    for (int i = 0; i < 20; i++)
    {
        total += analogRead(BATTERY_PIN);
    }

    float adc = total / 20.0f;

    float a0Voltage = (adc / 1023.0f) * ADC_REF;

    float batteryVoltage = a0Voltage * ((R1 + R2) / R2);
    batteryVoltage *= 1.0265;

    return batteryVoltage;
}

void sendIBUS()
{
  uint8_t data[32];

  data[0] = 0x20;
  data[1] = 0x40;

  for (int i = 0; i < 14; i++)
  {
    data[2 + i * 2] = ch[i] & 0xFF;
    data[3 + i * 2] = ch[i] >> 8;
  }

  uint16_t checksum = 0xFFFF;

  for (int i = 0; i < 30; i++)
    checksum -= data[i];

  data[30] = checksum & 0xFF;
  data[31] = checksum >> 8;

  ibus.write(data, 32);
}

void onDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len)
{
  if (len != sizeof(ControlPacket))
    return;

  memcpy(&packet, incomingData, sizeof(packet));

  lastPacketTime = millis();

// Map RSSI if your framework supports it, or use a rolling frame-counter check.
  // Alternatively, since we just successfully received a packet:
  // We can read the instantaneous RSSI of the station:
  int8_t rssi = WiFi.RSSI();

  // Map RSSI (typically -90 dBm [bad] to -30 dBm [perfect]) to 0-100%
  if (rssi <= -90) currentLinkQuality = 0;
  else if (rssi >= -30) currentLinkQuality = 100;
  else currentLinkQuality = map(rssi, -90, -30, 0, 100);

  ch[0] = packet.roll;
  ch[1] = packet.pitch;
  ch[2] = packet.throttle;
  ch[3] = packet.yaw;

  ch[4] = packet.aux1;
  ch[5] = packet.aux2;
  ch[6] = packet.aux3;
  ch[7] = packet.aux4;

  //Serial.print("Throttle: ");
  //Serial.println(ch[2]);
}

void setup()
{
  Serial.begin(115200);
  ibus.begin(115200);

  
  
  for (int i = 0; i < 14; i++)
    ch[i] = 1000;

  ch[0] = 1500;
  ch[1] = 1500;
  ch[2] = 1000;
  ch[3] = 1500;
  ch[4] = 1000;

  WiFi.mode(WIFI_STA);

  Serial.print("ESP8266 MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != 0)
  {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  
  if (esp_now_add_peer(transmitterMAC, ESP_NOW_ROLE_CONTROLLER, 0, NULL, 0) != 0)
  {
      Serial.println("Failed to add transmitter peer");
      return;
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.println("ESP-NOW + iBUS Ready");
}

void battery()
{
    static unsigned long lastBattery = 0;

    if (millis() - lastBattery >= 500)
    {
        lastBattery = millis();

        float voltage = readBatteryVoltage();

        telemetry.batteryMv = (uint16_t)(voltage * 1000.0f + 0.5f);
        telemetry.linkQuality = currentLinkQuality; // Assign the global link quality

        esp_now_send(transmitterMAC,
                     (uint8_t *)&telemetry,
                     sizeof(telemetry));
    }
}

void loop()
{
  // Failsafe check
  if (millis() - lastPacketTime > 500)
  {
      ch[0] = 1500; // Roll center
      ch[1] = 1500; // Pitch center
      ch[2] = 1000; // Throttle minimum
      ch[3] = 1500; // Yaw center
      ch[4] = 1000; // Disarm
  }

  // Send iBUS data frames at standard non-blocking intervals (~7-10ms)
  static unsigned long lastIbusTime = 0;
  if (millis() - lastIbusTime >= 7)
  {
      lastIbusTime = millis();
      sendIBUS();
  }

  // Handle telemetry transmissions
  battery();
}