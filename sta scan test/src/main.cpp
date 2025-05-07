#include "esp_wifi.h"
#include "nvs_flash.h"
#include <esp_task_wdt.h>
#include <Arduino.h>
#include <map>
#include <vector>
#include <array>

struct APinfo_t {
  std::array<uint8_t, 6> bssid;
  std::vector<std::array<uint8_t, 6>> STAbssids;
  uint8_t channel;
};

std::map<uint8_t, std::vector<APinfo_t>> apinfo;

bool macValid(const uint8_t *mac) {
  static const uint8_t zeroMac[6] = {0};
  static const uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  return (memcmp(mac, zeroMac, 6) != 0 && memcmp(mac, broadcastMac, 6) != 0) || mac[0] & 0x01;
}

void cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;

  if (type != WIFI_PKT_DATA || pkt->rx_ctrl.sig_len < 28) return;

  uint8_t* macFrom = pkt->payload + 10;
  uint8_t* macTo   = pkt->payload + 4;

  if (!macValid(macTo) || !macValid(macFrom)) return;

  std::array<uint8_t, 6> macArrayFrom;
  std::array<uint8_t, 6> macArrayTo;
  memcpy(macArrayFrom.data(), macFrom, 6);
  memcpy(macArrayTo.data(), macTo, 6);

  for (auto& [channel, aps] : apinfo) {
    for (auto& ap : aps) {
      if (ap.bssid == macArrayTo) {
        bool staExists = false;
        for (const auto& sta : ap.STAbssids) {
          if (memcmp(macArrayFrom.data(), sta.data(), 6) == 0) {
            staExists = true;
            break;
          }
        }
        if (!staExists) {
          ap.STAbssids.push_back(macArrayFrom);
          Serial.println("Found STA");
          Serial.printf("AP: %02x:%02x:%02x:%02x:%02x:%02x\n", 
                        macArrayTo[0], macArrayTo[1], macArrayTo[2], 
                        macArrayTo[3], macArrayTo[4], macArrayTo[5]);
          Serial.printf("STA: %02x:%02x:%02x:%02x:%02x:%02x\n", 
                        macArrayFrom[0], macArrayFrom[1], macArrayFrom[2], 
                        macArrayFrom[3], macArrayFrom[4], macArrayFrom[5]);
        }
        return;
      }
    }
  }
}

void scan() {
  Serial.println(F("Scanning..."));
  esp_wifi_set_promiscuous(false);
  digitalWrite(2, HIGH);
  wifi_scan_config_t scan_config = {
    .ssid = NULL,
    .bssid = NULL,
    .channel = 0,
    .show_hidden = true,
    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    .scan_time = { .active = { .min = 10, .max = 20 } }
  };
  ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
  uint16_t ap_num = 0;
  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));
  apinfo.clear();
  Serial.printf("Total APs found: %d\n", ap_num);
  wifi_ap_record_t ap_records[ap_num];
  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));

  for (int i = 0; i < ap_num; i++) {
    wifi_ap_record_t ap = ap_records[i];
    Serial.printf("%d | SSID: %s, BSSID: %02x:%02x:%02x:%02x:%02x:%02x, Channel: %d, RSSI: %d, Authmode: %d\n", 
                  i, ap.ssid, 
                  ap.bssid[0], ap.bssid[1], ap.bssid[2], 
                  ap.bssid[3], ap.bssid[4], ap.bssid[5], 
                  ap.primary, ap.rssi, ap.authmode);

    APinfo_t info;
    std::copy(std::begin(ap.bssid), std::end(ap.bssid), info.bssid.begin());
    info.channel = ap.primary;
    apinfo[ap.primary].push_back(info);
  }
  digitalWrite(2, LOW);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(cb);
}

void setup() {
  Serial.begin(115200);
  esp_task_wdt_init(6000, false);
  pinMode(2, OUTPUT);
  pinMode(0, INPUT_PULLUP);
  nvs_flash_init();
  tcpip_adapter_init();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  wifi_promiscuous_filter_t filter;
  filter.filter_mask = WIFI_PROMIS_FILTER_MASK_DATA;
  esp_wifi_set_promiscuous_filter(&filter);
  scan();
}
unsigned long previousMillis = 0;

void loop() {
  unsigned long currentMillis = millis();

  for (const auto& ap : apinfo) {
    uint8_t channel = ap.first;
    const std::vector<APinfo_t>& aps_in_channel = ap.second;

    for (const auto& ssidInfo : aps_in_channel) {
      esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

      while (currentMillis - previousMillis <= 1000) {
        currentMillis = millis();
        
        if (digitalRead(0) == LOW) {
          Serial.println("AP Info:");
          for (const auto& [channel, aps] : apinfo) {
            for (const auto& ap : aps) {
              Serial.printf("Channel: %d, BSSID: %02x:%02x:%02x:%02x:%02x:%02x\n", 
                            ap.channel, 
                            ap.bssid[0], ap.bssid[1], ap.bssid[2], 
                            ap.bssid[3], ap.bssid[4], ap.bssid[5]);
              Serial.println("STAs:");
              for (const auto& sta : ap.STAbssids) {
                Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n", sta[0], sta[1], sta[2], sta[3], sta[4], sta[5]);
              }
            }
          }
          delay(200);
        }
      }
      
      previousMillis = currentMillis;
    }
  }
}
