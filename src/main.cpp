#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Heltec MAC (receiver)
uint8_t receiverMac[] = {0x3C, 0x0F, 0x02, 0xEE, 0x09, 0x5C};

typedef struct {
  int value;
} Data;

Data data;

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Optional but helps stability
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;     // auto
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  data.value++;

  esp_now_send(receiverMac, (uint8_t*)&data, sizeof(data));

  Serial.print("Sent: ");
  Serial.println(data.value);

  delay(1000);
}