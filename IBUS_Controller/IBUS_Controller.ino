#include <Arduino.h>
#include <ESP8266WiFi.h>

extern "C" {
#include <espnow.h>
}

HardwareSerial &ibus = Serial1;

uint16_t ch[14];
uint32_t lastPacketTime = 0;

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

  ch[0] = packet.roll;
  ch[1] = packet.pitch;
  ch[2] = packet.throttle;
  ch[3] = packet.yaw;

  ch[4] = packet.aux1;
  ch[5] = packet.aux2;
  ch[6] = packet.aux3;
  ch[7] = packet.aux4;

  Serial.print("Throttle: ");
  Serial.println(ch[2]);
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
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("ESP-NOW + iBUS Ready");
}

void loop()
{
  if (millis() - lastPacketTime > 500)
  {
      // Lost transmitter for more than 500 ms

      ch[0] = 1500; // Roll center
      ch[1] = 1500; // Pitch center
      ch[2] = 1000; // Throttle minimum
      ch[3] = 1500; // Yaw center

      ch[4] = 1000; // Disarm (AUX1 low)
  }
  sendIBUS();
  delay(7);
}