//ESP8266
// Code 1 : Sending Text (Transmitter)
// Library: TMRh20/RF24 (https://github.com/tmrh20/RF24/)
/*
	ESP8266
	MOSI → GPIO13 -ESP8266-D7
	MISO → GPIO12 - ESP8266-D6
	SCK → GPIO14 - ESP8266-D5
	CSN → GPIO4 - ESP8266-D2
	CE → GPIO5 - ESP8266-D1
*/


#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

RF24 radio(D1, D2); // CE, CSN
const byte address[6] = "00001";

void setup() {
  Serial.begin(115200);
  
  Serial.println();
  Serial.println("================================");
  Serial.println("NRF24L01 Connection Test ESP8266");
  Serial.println("================================");
  
  radio.begin();
  radio.printDetails();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
};

void loop() {
  const char txt[] = "Hello World";
  radio.write(&txt, sizeof(txt));
  delay(1000);
};
