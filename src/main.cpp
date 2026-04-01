#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== OLED config =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== LoRa pins (SX1276) =====
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 23
#define LORA_DIO0 33

// ===== Matching Heltec settings =====
#define LORA_FREQUENCY 915E6
#define LORA_BW 250E3
#define LORA_SF 9
#define LORA_TX_POWER 0

void setup() {
  Serial.begin(115200);

  // ===== OLED init =====
  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Booting...");
  display.display();

  // ===== LoRa SPI init =====
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed!");
    display.println("LoRa FAIL");
    display.display();
    while (true);
  }

  // Match Heltec parameters
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setTxPower(LORA_TX_POWER);

  Serial.println("LoRa init OK");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("LoRa Ready");
  display.display();
}

void loop() {
  String message = "Hello SX1276";

  Serial.println("Sending: " + message);

  // Send LoRa packet
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();

  // Display on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Sending:");
  display.println(message);
  display.display();

  delay(2000);
}