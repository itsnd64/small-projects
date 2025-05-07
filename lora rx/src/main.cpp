#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128, 64, &Wire, 1);

uint32_t packetCounter = 0;
unsigned long lastPacketTime = 0;
uint32_t timeSinceLastMsg = 0;

void setup() {
  Wire.setPins(23, 22);
  SPI.begin(14, 26, 27, 12);
  LoRa.setPins(12, 2, 4);

  Serial.begin(115200);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {for (;;);}

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  while (!LoRa.begin(433E6)) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("LoRa Init Fail");
    display.display();
    delay(500);
  }
  display.println("LoRa Init OK");
  display.display();

  LoRa.writeRegister(0x01, 0x80 | 0x00);
  delay(10);
  LoRa.writeRegister(0x09, 0b11111111);
  LoRa.writeRegister(0x4D, 0x87);
  LoRa.writeRegister(0x3C, 00);
  LoRa.writeRegister(0x1D, 0x48);
  LoRa.writeRegister(0x1E, 0xA0);
  LoRa.writeRegister(0x36, 0x0c);
  delay(10);
  LoRa.writeRegister(0x01, 0x80 | 0x01);
  delay(10);
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize == 0) return;

  packetCounter++;

  unsigned long currentTime = millis();
  if (lastPacketTime != 0) timeSinceLastMsg = currentTime - lastPacketTime;
  lastPacketTime = currentTime;

  String receivedMessage = "";
  while (LoRa.available()) receivedMessage += (char)LoRa.read();

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  long freqError = LoRa.packetFrequencyError();

  uint8_t modemConfig1 = LoRa.readRegister(0x1D);
  uint8_t modemConfig2 = LoRa.readRegister(0x1E);
  uint8_t modemConfig3 = LoRa.readRegister(0x26);
  uint8_t paConfig = LoRa.readRegister(0x09);
  uint8_t payloadLength = LoRa.readRegister(0x22);

  uint8_t BW = (modemConfig1 & 0xF0) >> 4;
  uint8_t SF = (modemConfig2 >> 4) & 0x0F;
  uint8_t CR = (modemConfig1 & 0x0E) >> 1;
  uint8_t TXPower = (paConfig & 0x0F);

  String BW_str;
  switch (BW) {
    case 0x00: BW_str = "7.8"; break;
    case 0x01: BW_str = "10.4"; break;
    case 0x02: BW_str = "15.6"; break;
    case 0x03: BW_str = "20.8"; break;
    case 0x04: BW_str = "31.25"; break;
    case 0x05: BW_str = "41.7"; break;
    case 0x06: BW_str = "62.5"; break;
    case 0x07: BW_str = "125"; break;
    case 0x08: BW_str = "250"; break;
    case 0x09: BW_str = "500"; break;
    default: BW_str = "??"; break;
  }

  String SF_str;
  SF_str = "SF" + String(SF + 6);

  String CR_str;
  switch (CR) {
    case 0x01: CR_str = "4/5"; break;
    case 0x02: CR_str = "4/6"; break;
    case 0x03: CR_str = "4/7"; break;
    case 0x04: CR_str = "4/8"; break;
    default: CR_str = "??"; break;
  }

  Serial.println("Received packet:");
  Serial.println("Message: " + receivedMessage);
  Serial.println("Packet counter: " + String(packetCounter));
  Serial.println("RSSI: " + String(rssi));
  Serial.println("SNR: " + String(snr));
  Serial.println("Time since last msg: " + String(timeSinceLastMsg, 2) + "s");
  Serial.println();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Msg: \"%s\" Num: %u\n", receivedMessage, packetCounter);
  display.printf("RSSI: %i SNR: %.02f\n", rssi, snr);
  display.printf("Last msg: %ums\n", timeSinceLastMsg);
  display.printf("Freq error: %i Hz\n", freqError);
  display.printf("BW: %s SF: %s\n", BW_str, SF_str);
  display.printf("CR: %s Length: %u\n", CR_str, payloadLength);
  display.display();
}
