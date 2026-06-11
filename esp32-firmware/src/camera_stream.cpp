// ============================================================
//  camera_stream.cpp
// ============================================================
#include <Arduino.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "config.h"
#include "camera_stream.h"

static WebServer         server(STREAM_HTTP_PORT);
static SemaphoreHandle_t camMutex = NULL;
static const char*       PART_BOUNDARY = "frame";

static bool isValidJPEG(camera_fb_t* fb) {
  if (!fb || fb->len < 4) return false;
  if (fb->buf[0] != 0xFF || fb->buf[1] != 0xD8) return false;
  if (fb->buf[fb->len - 2] != 0xFF || fb->buf[fb->len - 1] != 0xD9) return false;
  return true;
}

// Continuous multipart/x-mixed-replace MJPEG. Blocks for the life of the
// client connection — which is why all control logic lives in its own
// task and loop() does nothing but pump this server.
static void handleJPGStream() {
  WiFiClient client = server.client();
  String hdr = "HTTP/1.1 200 OK\r\n"
               "Content-Type: multipart/x-mixed-replace; boundary=" +
               String(PART_BOUNDARY) + "\r\n\r\n";
  server.sendContent(hdr);

  while (client.connected()) {
    if (camMutex == NULL) break;
    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(200)) != pdTRUE) { delay(10); continue; }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb || !isValidJPEG(fb)) {
      if (fb) esp_camera_fb_return(fb);
      xSemaphoreGive(camMutex);
      delay(10);
      continue;
    }

    String part = "--" + String(PART_BOUNDARY) + "\r\n"
                  "Content-Type: image/jpeg\r\n"
                  "Content-Length: " + String(fb->len) + "\r\n\r\n";
    server.sendContent(part);
    client.write(fb->buf, fb->len);
    server.sendContent("\r\n");

    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);

    if (!client.connected()) break;
    delay(30);   // ~30 FPS cap
  }
}

static void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>ESP32-CAM</title></head>"
                "<body style='background:#111;color:#eee;font-family:sans-serif;text-align:center;'>"
                "<h1>ESP32-CAM — Doer</h1>"
                "<p>Streaming to the PC (Thinker) for YOLO. Motor commands arrive over UDP.</p>"
                "<img src=\"/stream\" style=\"max-width:100%;height:auto;\"/></body></html>";
  server.send(200, "text/html", html);
}

bool cameraInit() {
  camMutex = xSemaphoreCreateMutex();
  if (camMutex == NULL) return false;

  camera_config_t config;
  config.ledc_channel  = LEDC_CHANNEL_0;
  config.ledc_timer    = LEDC_TIMER_0;
  config.pin_d0        = Y2_GPIO_NUM;  config.pin_d1  = Y3_GPIO_NUM;
  config.pin_d2        = Y4_GPIO_NUM;  config.pin_d3  = Y5_GPIO_NUM;
  config.pin_d4        = Y6_GPIO_NUM;  config.pin_d5  = Y7_GPIO_NUM;
  config.pin_d6        = Y8_GPIO_NUM;  config.pin_d7  = Y9_GPIO_NUM;
  config.pin_xclk      = XCLK_GPIO_NUM;
  config.pin_pclk      = PCLK_GPIO_NUM;
  config.pin_vsync     = VSYNC_GPIO_NUM;
  config.pin_href      = HREF_GPIO_NUM;
  config.pin_sccb_sda  = SIOD_GPIO_NUM;
  config.pin_sccb_scl  = SIOC_GPIO_NUM;
  config.pin_pwdn      = PWDN_GPIO_NUM;
  config.pin_reset     = RESET_GPIO_NUM;
  config.xclk_freq_hz  = 20000000;
  config.pixel_format  = PIXFORMAT_JPEG;
  config.frame_size    = FRAMESIZE_QVGA;   // YOLO downsizes anyway; QVGA = low latency/bandwidth
  config.jpeg_quality  = 12;
  config.fb_count      = 2;
  return esp_camera_init(&config) == ESP_OK;
}

void streamServerBegin() {
  server.on("/",       HTTP_GET, handleRoot);
  server.on("/stream", HTTP_GET, handleJPGStream);
  server.begin();
  Serial.printf("[NET] MJPEG server started on :%d/stream\n", STREAM_HTTP_PORT);
}

void streamServerHandle() {
  server.handleClient();
}
