#include "ESP8266WiFi.h"

int ch = 1;

uint8_t packet[] = {
    0x40, 0x00, // Frame Control: Probe Request
    0x00, 0x00, // Duration
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // Destination
    0x96, 0x96, 0x96, 0x96, 0x96, 0x96, // Source
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // BSSID
    0x00, 0x00, // Sequence number

    // Tags
    0x00, 0x00,                         // SSID Parameter Set (SSID Length = 0 for broadcast)
    0x01, 0x08, 0x82, 0x84, 0x8b, 0x96, // Supported Rates (1, 2, 5.5, 11 Mbps)
    0x12, 0x24, 0x48, 0x6c              // Extended Supported Rates (18, 36, 72, 108 Mbps)
};

void setup() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
}

void loop() {
  ch = (ch % 13) + 1;
  for (int i = 10; i <= 16; i++) packet[i] = random(256);

  wifi_set_channel(ch);
  wifi_send_pkt_freedom(packet, sizeof(packet), 0);
  delay(1);
}
