#include <WiFi.h>

void checkMacAddress(){
    WiFi.mode(WIFI_STA);
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
}