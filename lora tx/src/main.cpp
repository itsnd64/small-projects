#include <SPI.h>
#include <LoRa.h>

#define NSS 16
#define RST 22
#define DI0 23

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
  SPI.begin(4, 15, 2, 16);
  LoRa.setPins(NSS, RST, DI0);
  while (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    delay(500);
  }
  Serial.println("LoRa Initializing Successful!");
  LoRa.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
  delay(10);
  LoRa.writeRegister(REG_PA_CONFIG, 0b11111111); // That's for the transceiver
  LoRa.writeRegister(REG_PA_DAC, PA_DAC_HIGH); // That's for the transceiver
  LoRa.writeRegister(REG_LNA, 00); // TURN OFF LNA FOR TRANSMIT
  uint8_t BW = 0x48, SF = 0xA0, AGC = 0x0c; // SF 10 BW 125KHz AGC
  LoRa.writeRegister(REG_MODEM_CONFIG_1, BW);
  LoRa.writeRegister(REG_MODEM_CONFIG_2, SF);
  LoRa.writeRegister(REG_MODEM_CONFIG_3, AGC);
  delay(10);
  LoRa.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
  delay(10);
}
int i = 0;
void loop() {
  Serial.println("Sending packet...");
  ++i;
  // Send a message
  LoRa.beginPacket();
  LoRa.print(i); // Message to send
  LoRa.endPacket();

  Serial.println("Packet sent!");
}