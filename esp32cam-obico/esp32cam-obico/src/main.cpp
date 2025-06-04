#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// Wi-Fi credentials
const char* ssid = "shs-0941";
const char* password = "L3TSG0SH0PP1NG";

// Manually defined pin map for ESP32-WROOM wired to OV2640
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     32
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define LED_GPIO_NUM       4  // optional onboard LED or external status LED

WebServer server(80);

void flashLED(int count, int delayMs) {
  pinMode(LED_GPIO_NUM, OUTPUT);
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_GPIO_NUM, HIGH);
    delay(delayMs);
    digitalWrite(LED_GPIO_NUM, LOW);
    delay(delayMs);
  }
}

void startCameraServer() {
  server.on("/stream", HTTP_GET, []() {
    WiFiClient client = server.client();
    String response = "HTTP/1.1 200 OK\r\n";
    response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    server.sendContent(response);
    while (client.connected()) {
      camera_fb_t* fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("[ERROR] Camera capture failed");
        break;
      }
      response = "--frame\r\nContent-Type: image/jpeg\r\n\r\n";
      server.sendContent(response);
      server.sendContent((const char*)fb->buf, fb->len);
      server.sendContent("\r\n");
      esp_camera_fb_return(fb);
      delay(50);
    }
  });
  server.begin();
  Serial.println("[INFO] Camera stream available at /stream");
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(500);
  Serial.println("[BOOT] ESP32-WROOM OV2640 CAM setup starting...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("[INFO] Connecting to SSID: %s\n", ssid);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries++ < 20) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[ERROR] Wi-Fi connection failed.");
    flashLED(10, 100);
    return;
  }

  Serial.printf("\n[INFO] Wi-Fi connected. IP address: %s\n", WiFi.localIP().toString().c_str());

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Safe default config (no PSRAM)
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  Serial.println("[INFO] Initializing camera...");
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[ERROR] Camera init failed: %s (0x%x)\n", esp_err_to_name(err), err);
    Serial.println("[HINT] Check wiring, power, camera compatibility.");
    flashLED(4, 300);
    return;
  }

  Serial.println("[INFO] Camera init OK. Starting server...");
  digitalWrite(LED_GPIO_NUM, HIGH); // LED on = ready
  startCameraServer();
}

void loop() {
  server.handleClient();
}
