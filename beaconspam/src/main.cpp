#include <Arduino.h>
#include <esp_wifi.h>
#include "nvs_flash.h"

void printPacket(const uint8_t *buffer, uint16_t len) {
  Serial.print("Received packet: ");
  for (int i = 0; i < len; i++) {
    if (i % 16 == 0) {
      Serial.println();
    }
    Serial.printf("%02X ", buffer[i]); // Print each byte in hexadecimal
  }
  Serial.println("\n");
}

void promiscuousCallback(void *buf, wifi_promiscuous_pkt_type_t type) {
  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
  uint8_t *payload = pkt->payload;
  uint16_t pktlen = pkt->rx_ctrl.sig_len;
  if (type == WIFI_PKT_MGMT) pktlen -= 4;

  // Beacon frame has type/subtype 0x80
  Serial.println("Packet Recieved!");
  printPacket(payload, pktlen);
}

void setup() {
  Serial.begin(115200);

  nvs_flash_init();
  esp_netif_init();
  esp_event_loop_create_default();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Set WiFi promiscuous mode
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);

  Serial.println("ESP32 in promiscuous mode, waiting for beacon frames...");
}

void loop() {
  // Keep the loop empty, everything is handled in the promiscuous callback
}
