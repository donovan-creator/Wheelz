#include "esp_camera.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include "wifi_config.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

void startCameraServer();

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP()); // Camera stream URL

  // Initialize camera
  camera_config_t config;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed!");
    return;
  }

  startCameraServer();
  Serial.println("Camera ready");
}

void loop() {
  delay(10000);
}