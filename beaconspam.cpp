#include <Arduino.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>
int ch = 1;
// Beacon Packet buffer
uint8_t packet[128] = {
  0x80, 0x00, 0x00, 0x00, // Frame Control
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // Destination: broadcast
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Source: will be filled by hardware
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // BSSID: will be filled by hardware
  0x00, 0x00, // Sequence Control
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Timestamp (to be set by hardware)
  0x64, 0x00, // Beacon interval (example value)
  0x01, 0x04, // Capability info (example value)
  0x00, 0x08, // SSID tag number and length (8 bytes for "TestSSID")
  'H', 'a', 'o', ' ', 'g', 'a', 'y', ' ', // SSID
  0x01, // Supported rates
  0x01, 0x01, 0x04 // DS Parameter set
};


void setup() {
  esp_task_wdt_init(6000,false);
  nvs_flash_init();
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE);
}

void loop() {
  ch = (ch % 13) + 1;
  for (int i = 10; i <= 21; i++) {
    packet[i] = random(256);
  }
  packet[56] = ch;
  esp_wifi_80211_tx(WIFI_IF_STA, packet, 57, false);
  delayMicroseconds(70);
}
