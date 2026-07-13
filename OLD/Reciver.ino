// Code 1 : Sending Text (Receiver)
// Library: TMRh20/RF24 (https://github.com/tmrh20/RF24/)
/*
	ESP32
	MOSI → GPIO23 -ESP-D23
	MISO → GPIO19 - ESP-D19
	SCK → GPIO18 - ESP-D18
	CSN → GPIO5 - ESP-D5
	CE → GPIO4 ESP-D4
*/

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

RF24 radio(4, 5); // CE, CSN
const byte address[6] = "00001";

void setup() {
  Serial.begin(115200);
  
  Serial.println();
  Serial.println("================================");
  Serial.println("NRF24L01 Connection Test ESP32");
  Serial.println("================================");

  radio.begin();
  radio.printDetails();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
};

void loop() {
    if (radio.available()) {
        char txt[32] = "";
        radio.read(&txt, sizeof(txt));
        Serial.print("Received: ");
        Serial.println(txt);
    }
}
