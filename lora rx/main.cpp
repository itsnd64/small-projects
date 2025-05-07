#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int packetCounter = 0;
unsigned long lastPacketTime = 0;
float timeSinceLastMsg = 0;

#define NSS 12
#define RST 2
#define DI0 4

#define REG_PA_CONFIG 0x09
#define REG_PA_DAC 0x4D
#define PA_DAC_HIGH 0x87
#define REG_LNA 0x0c
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_MODEM_CONFIG_3 0x26
#define REG_OP_MODE 0x01
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01

void setup() {
  Serial.begin(115200);

  Wire.setPins(23, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {for (;;);}
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("LoRa Receiver");
  display.display();
  SPI.begin(14, 26, 27, 12);
  LoRa.setPins(NSS, RST, DI0);
  while (!LoRa.begin(433E6)) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("LoRa Init Fail");
    display.display();
    delay(500);
  }
  display.println("LoRa Init OK");
  display.display();

  LoRa.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
  delay(10);
  LoRa.writeRegister(REG_PA_CONFIG, 0b11111111);
  LoRa.writeRegister(REG_PA_DAC, PA_DAC_HIGH);
  LoRa.writeRegister(REG_LNA, 00);
  uint8_t BW = 0x48, SF = 0xA0, AGC = 0x0c;
  LoRa.writeRegister(REG_MODEM_CONFIG_1, BW);
  LoRa.writeRegister(REG_MODEM_CONFIG_2, SF);
  LoRa.writeRegister(REG_MODEM_CONFIG_3, AGC);
  delay(10);
  LoRa.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
  delay(10);
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize == 0) return;

  packetCounter++;

  unsigned long currentTime = millis();
  if (lastPacketTime != 0) {
    timeSinceLastMsg = (currentTime - lastPacketTime) / 1000.0;
  }
  lastPacketTime = currentTime;

  String receivedMessage = "";
  while (LoRa.available()) {
    receivedMessage += (char)LoRa.read();
  }

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  Serial.println("Received packet:");
  Serial.println("Message: " + receivedMessage);
  Serial.println("Packet counter: " + String(packetCounter));
  Serial.println("RSSI: " + String(rssi));
  Serial.println("SNR: " + String(snr));
  Serial.println("Time since last msg: " + String(timeSinceLastMsg, 2) + "s");
  Serial.println();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Packet received:");
  display.println("Message: \"" + receivedMessage + "\"");
  display.println("Counter: " + String(packetCounter));
  display.println("RSSI: " + String(rssi));
  display.println("SNR: " + String(snr));
  display.println("Last msg: " + String(timeSinceLastMsg, 2) + "s");
  display.println(LoRa.packetFrequencyError());
  display.display();
}
